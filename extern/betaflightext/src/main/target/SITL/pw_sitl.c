/*
 * propwash target support — derived from Betaflight 4.5.2
 * src/main/target/SITL/sitl.c (GPL-3.0, see repository LICENSE).
 *
 * Differences from stock (recorded in ../patches/sitl-to-pw_sitl.diff):
 *  - Deterministic time: micros64()/millis64() are driven purely by the
 *    pw_micros_passed counter, which only the sim tick advances. No wall
 *    clock enters the firmware. delay()/delayMicroseconds() arm
 *    pw_sleep_timer instead of sleeping (SimITL semantics).
 *  - No UDP threads, no fdm/rc/servo packet transport: state is injected
 *    and motor output read directly by the in-process C++ glue
 *    (core/sim/bf.cpp) around each scheduler() call.
 *  - The dyad TCP thread for serial UARTs (MSP/CLI/Configurator) is kept.
 *  - eeprom path is settable at runtime (pw_set_eeprom_path) instead of
 *    the compile-time EEPROM_FILENAME macro.
 *  - systemReset() re-runs init() in-process instead of exit().
 *  - motorsPwm[] is exported (pw_motors_pwm) for the glue layer.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>
#include <pthread.h>

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "timer_def.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "config/config.h"
#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

#include "fc/init.h"

#include "dyad.h"

uint32_t SystemCoreClock;

/* ------------------------------------------------------------------ */
/* propwash deterministic time                                          */
/* ------------------------------------------------------------------ */

uint64_t pw_micros_passed = 0;   /* advanced only by the sim tick        */
int64_t  pw_sleep_timer   = 0;   /* armed by delay(), consumed per tick  */

uint64_t micros64(void)
{
    return pw_micros_passed;
}

uint64_t millis64(void)
{
    return pw_micros_passed / 1000;
}

uint32_t micros(void)
{
    return pw_micros_passed & 0xFFFFFFFF;
}

uint32_t millis(void)
{
    return (pw_micros_passed / 1000) & 0xFFFFFFFF;
}

uint32_t getCycleCounter(void)
{
    return (uint32_t)(pw_micros_passed & 0xFFFFFFFF);
}

void microsleep(uint32_t usec)
{
    pw_sleep_timer = usec;
}

void delayMicroseconds(uint32_t us)
{
    microsleep(us);
}

void delay(uint32_t ms)
{
    microsleep(ms * 1000);
}

/* Wall-clock variants kept for completeness (nothing in the linked set
 * calls them; stock main.c did). */
static struct timespec start_time;

uint64_t nanos64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000000000ULL + ts.tv_nsec) -
           (start_time.tv_sec * 1000000000ULL + start_time.tv_nsec);
}

uint64_t micros64_real(void)
{
    return nanos64_real() / 1000;
}

uint64_t millis64_real(void)
{
    return nanos64_real() / 1000000;
}

void delayMicroseconds_real(uint32_t us)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = us * 1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return clockCycles;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros;
}

/* ------------------------------------------------------------------ */
/* system                                                               */
/* ------------------------------------------------------------------ */

static pthread_t tcpWorker;
static bool workerRunning = true;
static bool tcpThreadStarted = false;

int targetParseArgs(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    return 0;
}

int lockMainPID(void)
{
    return 0;
}

// propwash: dyad is single-threaded by design, but this worker owns the dyad
// event pump while the Betaflight scheduler (the sim thread) also calls dyad
// from serial_tcp.c — dyad_listen() at serial init and dyad_write() on every
// CLI/MSP response. Those unsynchronised accesses to dyad's global stream list
// race: the listener stream can be left unserviced so onAccept never fires and
// the CLI/MSP goes dead (reproduced 100% on GitHub macOS runners, masked by
// thread-timing locally). pwDyadMutex serialises every dyad call; serial_tcp.c
// takes it via pw_dyad_lock/unlock below.
static pthread_mutex_t pwDyadMutex = PTHREAD_MUTEX_INITIALIZER;

void pw_dyad_lock(void)   { pthread_mutex_lock(&pwDyadMutex); }
void pw_dyad_unlock(void) { pthread_mutex_unlock(&pwDyadMutex); }

static void *tcpThread(void *data)
{
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    // non-blocking poll so the lock is never held across a blocking select();
    // the usleep below sets the effective poll rate and yields to the sim thread
    dyad_setUpdateTimeout(0.0f);

    while (workerRunning) {
        pw_dyad_lock();
        dyad_update();
        pw_dyad_unlock();
        // ~2 kHz poll: keeps CLI/MSP latency sub-ms without busy-spinning, and
        // hands pwDyadMutex to the sim thread between polls (nanosleep via
        // <time.h>, already used here — avoids <unistd.h>, which perturbs the
        // macOS feature-test macros and hides clock_gettime/nanosleep)
        nanosleep(&(struct timespec){0, 500000}, NULL);
    }

    dyad_shutdown();
    printf("[pw] tcpThread end\n");
    return NULL;
}

void systemInit(void)
{
    printf("[pw][system] init\n");

    clock_gettime(CLOCK_MONOTONIC, &start_time);

    SystemCoreClock = 500 * 1000000; /* virtual 500 MHz */
    pw_micros_passed = 0;
    pw_sleep_timer = 0;

    if (!tcpThreadStarted) {
        int ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
        if (ret != 0) {
            fprintf(stderr, "[pw] failed to create tcp thread\n");
            exit(1);
        }
        tcpThreadStarted = true;
    }
}

void systemReset(void)
{
    printf("[pw][system] reset -> re-init\n");
    init();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    printf("[pw][system] reset-to-bootloader ignored\n");
}

void timerInit(void)
{
}

void failureMode(failureMode_e mode)
{
    fprintf(stderr, "[pw][failureMode] %d\n", mode);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    fprintf(stderr, "[pw][failure LED] %d\n", mode);
}

/* ------------------------------------------------------------------ */
/* PWM / motors                                                         */
/* ------------------------------------------------------------------ */

pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

/* exported to the glue layer: value - idlePulse, i.e. 0..1000 */
int16_t pw_motors_pwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    UNUSED(servoConfig);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    motorPwmDevice.enabled = false;
}

static bool pwmEnableMotors(void)
{
    motorPwmDevice.enabled = true;
    return true;
}

static void pwmWriteMotor(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_MOTORS) {
        pw_motors_pwm[index] = value - idlePulse;
    }
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    motorPwmDevice.enabled = false;
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

static void pwmCompleteMotorUpdate(void)
{
    /* nothing to transport — glue reads pw_motors_pwm in-process */
}

void pwmWriteServo(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_SERVOS) {
        servosPwm[index] = value;
    }
}

static motorDevice_t motorPwmDevice = {
    .vTable = {
        .postInit = motorPostInitNull,
        .convertExternalToMotor = pwmConvertFromExternal,
        .convertMotorToExternal = pwmConvertToExternal,
        .enable = pwmEnableMotors,
        .disable = pwmDisableMotors,
        .isMotorEnabled = pwmIsMotorEnabled,
        .decodeTelemetry = motorDecodeTelemetryNull,
        .write = pwmWriteMotor,
        .writeInt = pwmWriteMotorInt,
        .updateComplete = pwmCompleteMotorUpdate,
        .shutdown = pwmShutdownPulsesForAllMotors,
    }
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    UNUSED(motorConfig);
    UNUSED(useUnsyncedPwm);

    printf("[pw] initialized motor count %d\n", motorCount);

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        motors[motorIndex].enabled = true;
    }
    motorPwmDevice.count = motorCount;
    motorPwmDevice.initialized = true;
    motorPwmDevice.enabled = false;

    return &motorPwmDevice;
}

/* ------------------------------------------------------------------ */
/* ADC                                                                  */
/* ------------------------------------------------------------------ */

uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

/* stack check symbols */
char _estack;
char _Min_Stack_Size;

/* ------------------------------------------------------------------ */
/* virtual EEPROM (settable path)                                       */
/* ------------------------------------------------------------------ */

static FILE *eepromFd = NULL;
static char pw_eeprom_path[512] = EEPROM_FILENAME;

void pw_set_eeprom_path(const char *path)
{
    snprintf(pw_eeprom_path, sizeof(pw_eeprom_path), "%s", path);
}

void FLASH_Unlock(void)
{
    if (eepromFd != NULL) {
        /* in-process re-init (systemReset) can arrive here with the file
         * still open — recover instead of bailing */
        fclose(eepromFd);
        eepromFd = NULL;
    }

    // open or create
    eepromFd = fopen(pw_eeprom_path, "r+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[pw][FLASH_Unlock] loaded '%s', size = %zu / %zu\n", pw_eeprom_path, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[pw][FLASH_Unlock] failed to load '%s'\n", pw_eeprom_path);
            return;
        }
    } else {
        printf("[pw][FLASH_Unlock] created '%s', size = %zu\n", pw_eeprom_path, sizeof(eepromData));
        if ((eepromFd = fopen(pw_eeprom_path, "w+")) == NULL) {
            fprintf(stderr, "[pw][FLASH_Unlock] failed to create '%s'\n", pw_eeprom_path);
            return;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[pw][FLASH_Unlock] write failed: %s\n", strerror(errno));
        }
    }
}

void FLASH_Lock(void)
{
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[pw][FLASH_Lock] saved '%s'\n", pw_eeprom_path);
    } else {
        fprintf(stderr, "[pw][FLASH_Lock] eeprom is not unlocked\n");
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address)
{
    UNUSED(Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t value)
{
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData))) {
        *((uint32_t*)addr) = value;
    }
    return FLASH_COMPLETE;
}

/* ------------------------------------------------------------------ */
/* IO stubs                                                             */
/* ------------------------------------------------------------------ */

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
}

void debugInit(void)
{
}

void unusedPinsInit(void)
{
}

void IOHi(IO_t io)
{
    UNUSED(io);
}

void IOLo(IO_t io)
{
    UNUSED(io);
}

bool useDshotTelemetry = false;

/*
 * TARGET_PREINIT hook — runs right after config load, before motor/mixer
 * init. Forces settings that cannot work on the simulator regardless of
 * what the loaded diff says (the real FC diff uses DSHOT600 + RPM filter;
 * the fake motor device is PWM-like and has no ESC telemetry — with DSHOT
 * left on, ARMING_DISABLED_BOOT_GRACE_TIME never clears because DSHOT
 * streaming commands never become available).
 */
void targetPreInit(void)
{
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_STANDARD;
    motorConfigMutable()->dev.useUnsyncedPwm = true;
    printf("[pw] targetPreInit: forced motor protocol PWM (SITL override)\n");
}

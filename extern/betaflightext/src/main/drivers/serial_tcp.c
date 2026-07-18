/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

/*
 * propwash: vendored from stock drivers/serial_tcp.c (BF 4.5.2), the ONLY
 * change being dyad thread-safety. dyad is single-threaded by design, but
 * propwash pumps dyad_update() on a dedicated worker thread (pw_sitl.c's
 * tcpThread) while the Betaflight scheduler (the sim thread) calls dyad here
 * from tcpReconfigure() (dyad_newStream/dyad_listenEx at init) and tcpDataOut()
 * (dyad_write on every CLI/MSP response). Those concurrent, unsynchronised
 * accesses to dyad's global stream list are a data race: on some hosts the
 * listener stream is never serviced by dyad_update() — onAccept never fires,
 * the kernel silently accepts the TCP connection into its listen backlog, and
 * the CLI/MSP appears dead (bind succeeds, no data ever flows). It reproduced
 * 100% on GitHub's macOS runners and passed locally purely on thread-timing
 * luck. pw_dyad_lock/unlock (defined in pw_sitl.c) serialise every dyad call.
 * See extern/betaflightext/patches/serial-tcp-dyad-lock.diff.
 *
 * NOTE: the onAccept/onData/onClose callbacks run *inside* dyad_update() (on
 * the worker thread, which already holds the lock), so they must NOT re-lock —
 * the mutex is non-recursive.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
// full path (this vendored copy lives outside drivers/, and adding drivers/ to
// the include path would shadow the system <time.h> with Betaflight's)
#include "drivers/serial_tcp.h"

// propwash: serialise dyad access against pw_sitl.c's tcpThread (dyad_update).
extern void pw_dyad_lock(void);
extern void pw_dyad_unlock(void);

#define BASE_PORT 5760

static const struct serialPortVTable tcpVTable; // Forward
static tcpPort_t tcpSerialPorts[SERIAL_PORT_COUNT];
static bool tcpPortInitialized[SERIAL_PORT_COUNT];
static bool tcpStart = false;
bool tcpIsStart(void)
{
    return tcpStart;
}
static void onData(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    tcpDataIn(s, (uint8_t*)e->data, e->size);
}
static void onClose(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    fprintf(stderr, "[CLS]UART%u: %d,%d\n", s->id + 1, s->connected, s->clientCount);
    // propwash: only clear the slot if the CURRENT client closed. A stale
    // close event (from a client we already dropped in onAccept) must not wipe
    // the new connection's slot. e->stream is the connection that closed.
    if (s->conn == e->stream) {
        s->conn = NULL;
        s->clientCount = 0;
        s->connected = false;
    }
}
static void onAccept(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    fprintf(stderr, "New connection on UART%u, %d\n", s->id + 1, s->clientCount);

    // propwash: newest client wins. Stock BF rejects a new connection while
    // one already exists, but propwash's CLI/MSP reconnects constantly (the
    // Configurator, and every test phase re-opens the CLI) and onClose — which
    // frees the slot — can lag the reconnect on a slow/busy host. The port
    // then stays permanently "full" and the CLI/MSP goes dead (this is why
    // diff_roundtrip's post-save readback returned nothing on the macOS CI
    // runner while a later raw probe worked). So drop any stale client and
    // take the new connection instead of rejecting it.
    if (s->conn != NULL) {
        dyad_close(s->conn);
    }
    s->connected = true;
    s->clientCount = 1;
    s->conn = e->remote;
    fprintf(stderr, "[NEW]UART%u: %d,%d\n", s->id + 1, s->connected, s->clientCount);
    dyad_setNoDelay(e->remote, 1);
    dyad_setTimeout(e->remote, 120);
    dyad_addListener(e->remote, DYAD_EVENT_DATA, onData, e->udata);
    dyad_addListener(e->remote, DYAD_EVENT_CLOSE, onClose, e->udata);
}
static tcpPort_t* tcpReconfigure(tcpPort_t *s, int id)
{
    if (tcpPortInitialized[id]) {
        fprintf(stderr, "port is already initialized!\n");
        return s;
    }

    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        fprintf(stderr, "TX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return NULL;
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return NULL;
    }

    tcpStart = true;
    tcpPortInitialized[id] = true;

    s->connected = false;
    s->clientCount = 0;
    s->id = id;
    s->conn = NULL;
    // propwash: creating a stream and starting the listener mutates dyad's
    // global stream list — must not race the worker thread's dyad_update().
    pw_dyad_lock();
    s->serv = dyad_newStream();
    dyad_setNoDelay(s->serv, 1);
    dyad_addListener(s->serv, DYAD_EVENT_ACCEPT, onAccept, s);

    // propwash: bind host. On Windows getaddrinfo(NULL, ...) returns IPv6
    // first, and IPv6 sockets are v6-only by default (IPV6_V6ONLY), so a NULL
    // host makes dyad bind [::]:port and REFUSE the IPv4 127.0.0.1 that the
    // Configurator, pw_cli.py and the tests connect to. Bind IPv4 explicitly
    // there; elsewhere keep the stock all-interfaces (NULL) behaviour.
#if defined(_WIN32)
    const char *bindHost = "0.0.0.0";
#else
    const char *bindHost = NULL;
#endif
    int listenErr = dyad_listenEx(s->serv, bindHost, BASE_PORT + id + 1, 10);
    pw_dyad_unlock();
    if (listenErr == 0) {
        fprintf(stderr, "bind port %u for UART%u\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
    } else {
        fprintf(stderr, "bind port %u for UART%u failed!!\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
    }
    return s;
}

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    tcpPort_t *s = NULL;

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
    if (id >= 0 && id < SERIAL_PORT_COUNT) {
    s = tcpReconfigure(&tcpSerialPorts[id], id);
    }
#endif
    if (!s)
        return NULL;

    s->port.vTable = &tcpVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t count;
    pthread_mutex_lock(&s->rxLock);
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
    pthread_mutex_unlock(&s->rxLock);

    return count;
}

uint32_t tcpTotalTxBytesFree(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t bytesUsed;

    pthread_mutex_lock(&s->txLock);
    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }
    uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;
    pthread_mutex_unlock(&s->txLock);

    return bytesFree;
}

bool isTcpTransmitBufferEmpty(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);
    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;
    pthread_mutex_unlock(&s->txLock);
    return isEmpty;
}

uint8_t tcpRead(serialPort_t *instance)
{
    uint8_t ch;
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }
    pthread_mutex_unlock(&s->rxLock);

    return ch;
}

void tcpWrite(serialPort_t *instance, uint8_t ch)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }
    pthread_mutex_unlock(&s->txLock);

    tcpDataOut(s);
}

void tcpDataOut(tcpPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    if (s->conn == NULL) return;
    pthread_mutex_lock(&s->txLock);
    // propwash: dyad_write mutates the stream's write buffer — serialise
    // against the worker thread's dyad_update().
    pw_dyad_lock();

    if (s->port.txBufferHead < s->port.txBufferTail) {
        // send data till end of buffer
        int chunk = s->port.txBufferSize - s->port.txBufferTail;
        dyad_write(s->conn, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        s->port.txBufferTail = 0;
    }
    int chunk = s->port.txBufferHead - s->port.txBufferTail;
    if (chunk)
        dyad_write(s->conn, (const void*)&s->port.txBuffer[s->port.txBufferTail], chunk);
    s->port.txBufferTail = s->port.txBufferHead;

    pw_dyad_unlock();
    pthread_mutex_unlock(&s->txLock);
}

void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    while (size--) {
//        printf("%c", *ch);
        s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
        if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferHead = 0;
        } else {
            s->port.rxBufferHead++;
        }
    }
    pthread_mutex_unlock(&s->rxLock);
//    printf("\n");
}

static const struct serialPortVTable tcpVTable = {
        .serialWrite = tcpWrite,
        .serialTotalRxWaiting = tcpTotalRxBytesWaiting,
        .serialTotalTxFree = tcpTotalTxBytesFree,
        .serialRead = tcpRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isTcpTransmitBufferEmpty,
        .setMode = NULL,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
};

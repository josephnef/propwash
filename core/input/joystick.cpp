#include "joystick.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#if defined(__linux__)
#include <sys/ioctl.h>
#include <linux/joystick.h>
#define PW_JS_LINUX 1
#else
#define PW_JS_LINUX 0
#endif

namespace pw {

// ----------------------------------------------------------- portable bits
// (no OS joystick API — calibration file parsing + axis scaling are shared)

float Joystick::apply(int axis, int raw) const
{
    const float span = (float)(mHi[axis] - mLo[axis]);
    if (span == 0.0f) return 0.0f;
    float n = 2.0f * (raw - mLo[axis]) / span - 1.0f;
    if (n < -1.0f) n = -1.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

const char* Joystick::defaultCalPath()
{
    static char path[256];
    const char* home = getenv("HOME");
    snprintf(path, sizeof(path), "%s/.config/propwash/joystick.cal",
             home ? home : ".");
    return path;
}

bool Joystick::loadCalibration(const char* path)
{
    FILE* f = fopen(path, "r");
    if (!f) return false;
    char line[128];
    int loaded = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        int axis, lo, hi;
        if (sscanf(line, "%d %d %d", &axis, &lo, &hi) == 3 && axis >= 0 && axis < 8) {
            mLo[axis] = lo;
            mHi[axis] = hi;
            loaded++;
        }
    }
    fclose(f);
    if (loaded) printf("[pw][js] loaded calibration from %s (%d axes)\n", path, loaded);
    return loaded > 0;
}

#if PW_JS_LINUX
// ----------------------------------------------------- Linux kernel js API

static bool nameContainsEdgeTX(const char* name)
{
    char lower[128];
    size_t n = strnlen(name, sizeof(lower) - 1);
    for (size_t i = 0; i < n; i++) lower[i] = (char)tolower((unsigned char)name[i]);
    lower[n] = 0;
    return strstr(lower, "edgetx") != nullptr || strstr(lower, "opentx") != nullptr;
}

static int tryOpen(const char* path, char* nameOut, size_t nameLen)
{
    int fd = ::open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) return -1;
    if (ioctl(fd, JSIOCGNAME(nameLen), nameOut) < 0) {
        snprintf(nameOut, nameLen, "unknown");
    }
    return fd;
}

bool Joystick::open(const char* devPath)
{
    close();

    if (devPath) {
        mFd = tryOpen(devPath, mName, sizeof(mName));
        if (mFd >= 0) {
            printf("[pw][js] %s: %s\n", devPath, mName);
            return true;
        }
        fprintf(stderr, "[pw][js] cannot open %s\n", devPath);
        return false;
    }

    // scan: prefer an EdgeTX handset, else first joystick
    int fallbackFd = -1;
    char fallbackName[128] = {0};
    char fallbackPath[32] = {0};

    for (int i = 0; i < 16; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/input/js%d", i);
        char name[128] = {0};
        int fd = tryOpen(path, name, sizeof(name));
        if (fd < 0) continue;

        if (nameContainsEdgeTX(name)) {
            if (fallbackFd >= 0) ::close(fallbackFd);
            mFd = fd;
            memcpy(mName, name, sizeof(mName));
            printf("[pw][js] %s: %s\n", path, mName);
            return true;
        }
        if (fallbackFd < 0) {
            fallbackFd = fd;
            memcpy(fallbackName, name, sizeof(fallbackName));
            memcpy(fallbackPath, path, sizeof(fallbackPath));
        } else {
            ::close(fd);
        }
    }

    if (fallbackFd >= 0) {
        mFd = fallbackFd;
        memcpy(mName, fallbackName, sizeof(mName));
        printf("[pw][js] %s: %s (no EdgeTX device, using first joystick)\n", fallbackPath, mName);
        return true;
    }
    return false;
}

bool Joystick::poll()
{
    if (mFd < 0) return false;

    bool changed = false;
    struct js_event ev;
    while (read(mFd, &ev, sizeof(ev)) == sizeof(ev)) {
        if ((ev.type & JS_EVENT_AXIS) && ev.number < 8) {
            mChannels[ev.number] = apply(ev.number, ev.value);
            changed = true;
        }
    }
    return changed;
}

bool Joystick::calibrate(const char* path)
{
    if (mFd < 0) {
        fprintf(stderr, "[pw][js] no joystick open to calibrate\n");
        return false;
    }

    int rawMin[8], rawMax[8], rawRest[8];
    int cur[8] = {0};
    for (int i = 0; i < 8; i++) { rawMin[i] = 32767; rawMax[i] = -32767; rawRest[i] = 0; }

    auto drain = [&]() {
        struct js_event ev;
        while (read(mFd, &ev, sizeof(ev)) == sizeof(ev)) {
            if ((ev.type & JS_EVENT_AXIS) && ev.number < 8) {
                cur[ev.number] = ev.value;
                if (ev.value < rawMin[ev.number]) rawMin[ev.number] = ev.value;
                if (ev.value > rawMax[ev.number]) rawMax[ev.number] = ev.value;
            }
        }
    };

    printf("\n=== propwash joystick calibration ===\n");
    printf("1) Move ALL sticks and switches through their FULL range\n");
    printf("   (throttle fully up AND down, roll/pitch/yaw to every corner,\n");
    printf("    flip every switch both ways). You have 10 seconds...\n");
    fflush(stdout);
    for (int s = 10; s > 0; s--) {
        for (int k = 0; k < 20; k++) { drain(); usleep(50000); }
        printf("   %d\n", s - 1); fflush(stdout);
    }

    printf("\n2) Now set the SAFE resting pose and hold still:\n");
    printf("   throttle FULL DOWN, sticks CENTRED, switches to DISARMED.\n");
    printf("   Recording in 3 seconds...\n");
    fflush(stdout);
    for (int k = 0; k < 60; k++) { drain(); usleep(50000); }  // 3 s settle
    drain();
    for (int i = 0; i < 8; i++) rawRest[i] = cur[i];

    // Orient each axis so the resting end maps to -1: pick lo = the extreme
    // nearest the rest value, hi = the other extreme. For self-centring
    // sticks the rest sits mid-range and orientation is immaterial.
    FILE* f = fopen(path, "w");
    if (!f) {
        // try to create the directory
        char dir[256];
        snprintf(dir, sizeof(dir), "%s/.config/propwash", getenv("HOME") ? getenv("HOME") : ".");
        char mk[300];
        snprintf(mk, sizeof(mk), "%s/.config", getenv("HOME") ? getenv("HOME") : ".");
        mkdir(mk, 0755);
        mkdir(dir, 0755);
        f = fopen(path, "w");
    }
    if (!f) { fprintf(stderr, "[pw][js] cannot write %s\n", path); return false; }

    fprintf(f, "# propwash joystick calibration: axis lo hi  (raw lo->-1, hi->+1)\n");
    const char* label[8] = {"roll", "pitch", "throttle", "yaw", "ch5", "ch6", "ch7", "ch8"};
    for (int i = 0; i < 8; i++) {
        if (rawMax[i] <= rawMin[i]) { rawMin[i] = -32767; rawMax[i] = 32767; }
        bool restNearMin = std::abs(rawRest[i] - rawMin[i]) <= std::abs(rawRest[i] - rawMax[i]);
        int lo = restNearMin ? rawMin[i] : rawMax[i];
        int hi = restNearMin ? rawMax[i] : rawMin[i];
        mLo[i] = lo; mHi[i] = hi;
        fprintf(f, "%d %d %d\n", i, lo, hi);
        printf("   axis %d (%-8s): rest=%6d  range=[%6d,%6d] -> %s\n",
               i, label[i], rawRest[i], rawMin[i], rawMax[i],
               (lo > hi) ? "inverted" : "normal");
    }
    fclose(f);
    printf("\nSaved calibration to %s\n", path);
    printf("Throttle-down and disarmed switches now map to -1 (armable).\n\n");
    return true;
}

void Joystick::close()
{
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
}

#else
// ------------------------------------------------- non-Linux stub backend
// No kernel `js` device here (the Linux joystick API is Linux-only). On
// macOS the RadioMaster/EdgeTX handset is read by the Godot client instead
// (Godot's cross-platform Input API) and injected as RC over the wire — the
// server's RC priority falls through to the client packet when the core has
// no local joystick.

bool Joystick::open(const char* devPath)
{
    (void)devPath;
    return false;
}

bool Joystick::poll() { return false; }

bool Joystick::calibrate(const char* path)
{
    (void)path;
    fprintf(stderr, "[pw][js] --js-calibrate is Linux-only; on this platform "
                    "the client reads the handset directly\n");
    return false;
}

void Joystick::close() { mFd = -1; }

#endif  // PW_JS_LINUX

} // namespace pw

#include "joystick.h"

#include <cstdio>
#include <cstring>
#include <cctype>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

namespace pw {

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
            mChannels[ev.number] = (float)ev.value / 32767.0f;
            changed = true;
        }
    }
    return changed;
}

void Joystick::close()
{
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
}

} // namespace pw

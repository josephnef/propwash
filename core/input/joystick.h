/**
 * RC input from a Linux joystick device (RadioMaster Pocket / any EdgeTX
 * handset in USB Joystick mode). Uses the kernel js API — no dependencies.
 *
 * EdgeTX default "Classic" mapping: axes 0-7 = CH1-CH8 (AETR + switches),
 * full range ±32767 (verified on this rig, see cinelog35-v3 docs/SIM.md).
 * EdgeTX 2.9+ "Advanced" mode can remap axes — hence the env override.
 */

#ifndef PW_JOYSTICK_H
#define PW_JOYSTICK_H

#include <cstdint>

namespace pw {

class Joystick {
  public:
    /**
     * Open a joystick device. If devPath is nullptr, scans /dev/input/js0-15
     * and picks the first whose name contains "EdgeTX" (case-insensitive),
     * falling back to the first joystick present.
     * \return true if a device was opened.
     */
    bool open(const char* devPath = nullptr);

    bool isOpen() const { return mFd >= 0; }
    const char* name() const { return mName; }

    /**
     * Drain pending events and update the channel state.
     * \return true if any axis changed.
     */
    bool poll();

    /** Normalised channel values -1..1, axes 0-7. */
    const float* channels() const { return mChannels; }

    void close();
    ~Joystick() { close(); }

  private:
    int mFd = -1;
    char mName[128] = {0};
    float mChannels[8] = {0, 0, -1.0f, 0, -1.0f, -1.0f, -1.0f, -1.0f};
};

} // namespace pw

#endif // PW_JOYSTICK_H

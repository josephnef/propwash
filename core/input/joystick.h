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

    /**
     * Load per-axis calibration (raw lo/hi) from a file. Without it the raw
     * axis is used unscaled, which fails when an axis (typically throttle)
     * is inverted or doesn't span the full range. \return true if loaded.
     */
    bool loadCalibration(const char* path);

    /**
     * Interactive timed calibration: records each axis' extremes, then the
     * resting position (throttle-down / sticks-centred / switches-disarmed)
     * so the low/disarmed end always maps to -1. Writes the file. Blocking.
     */
    bool calibrate(const char* path);

    /** Default calibration path: $HOME/.config/propwash/joystick.cal */
    static const char* defaultCalPath();

    void close();
    ~Joystick() { close(); }

  private:
    float apply(int axis, int raw) const;

    int mFd = -1;
    char mName[128] = {0};
    float mChannels[8] = {0, 0, -1.0f, 0, -1.0f, -1.0f, -1.0f, -1.0f};
    // per-axis mapping: raw==lo -> -1, raw==hi -> +1 (lo>hi inverts)
    int mLo[8] = {-32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767};
    int mHi[8] = { 32767,  32767,  32767,  32767,  32767,  32767,  32767,  32767};
};

} // namespace pw

#endif // PW_JOYSTICK_H

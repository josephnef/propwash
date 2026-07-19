#ifndef CONTACT_MATERIALS_H
#define CONTACT_MATERIALS_H

#include <cstdint>

namespace SimITL {

  /* Contact material response, indexed by PwSurfaceType. Each contact point
   * is a penalty spring-damper evaluated every 50 us sub-tick:
   *
   *   Fn = clamp(k*depth - c*vn, 0, CONTACT_FMAX)      (no adhesion)
   *   Ft = -vt_hat * min(mu*Fn, CONTACT_KT*|vt|)       (Coulomb, viscous cap)
   *
   * c = 2*zeta*sqrt(k*m) with m = 0.33 kg (whole quad). A corner contact has
   * a smaller effective mass, which makes the same c MORE damped — always the
   * stable, deader side (reads as the frame absorbing edge hits).
   *
   * Stability at dt = 50 us: the stiffest material (k = 15000) on the corner
   * effective mass (~0.12 kg) gives omega = sqrt(k/m_eff) ~= 353 rad/s, so
   * omega*dt ~= 0.018 — two orders of magnitude under the symplectic-Euler
   * limit of 2. The friction cap satisfies KT*dt/m_eff ~= 0.17 < 1, so
   * friction cannot reverse the tangential velocity within one tick.
   * Static penetration at rest: mg/(4k) ~= 0.1 mm. */
  struct ContactMaterial {
    float k;   // spring stiffness, N/m
    float c;   // normal damping, N*s/m
    float mu;  // Coulomb friction coefficient
  };

  // index = PwSurfaceType (GROUND, GATE, TREE, OBJECT)
  inline constexpr ContactMaterial CONTACT_MATERIALS[4] = {
    /* GROUND dirt/grass */ { 8000.0f, 72.0f, 0.90f }, // zeta 0.70, e ~ 0.05
    /* GATE   plastic    */ {15000.0f, 49.0f, 0.35f }, // zeta 0.35, e ~ 0.31
    /* TREE   trunk      */ {12000.0f, 69.0f, 0.55f }, // zeta 0.55, e ~ 0.13
    /* OBJECT generic    */ {15000.0f, 70.0f, 0.50f },
  };

  inline const ContactMaterial& contactMaterial(uint8_t surface) {
    return CONTACT_MATERIALS[surface < 4 ? surface : 3];
  }

  // tangential viscous coefficient capping Coulomb friction near rest
  inline constexpr float CONTACT_KT = 400.0f;   // N*s/m
  // per-contact normal force clamp (robustness at extreme penetrations)
  inline constexpr float CONTACT_FMAX = 250.0f; // N
  // penetration depth clamp
  inline constexpr float CONTACT_DMAX = 0.06f;  // m

}

#endif // CONTACT_MATERIALS_H

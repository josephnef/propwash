# Licensing

| Component | License | Why |
|-----------|---------|-----|
| `extern/betaflight` (submodule) | GPL-3.0 | Upstream Betaflight |
| `extern/betaflightext/` | GPL-3.0 | Derived from Betaflight sources |
| `core/`, `tools/` | GPL-3.0 | Statically link Betaflight; `core/sim/physics.cpp` and `core/sim/util/` are ported from SimITL (GPL-3.0, © AJ92) |
| `protocol/` | MIT | The wire protocol header. **Must never include a Betaflight header** — it is the license boundary. |
| `client-godot/`, `python/` | MIT | Speak the UDP protocol only; no GPL code crosses the socket. |

The process boundary (propwash-core executable ↔ UDP clients) is the same
licensing arrangement KwadSim uses: the GPL work is distributed as a
standalone program; clients interoperate through a documented protocol.

If you redistribute binaries of propwash-core you must provide its source
(including the Betaflight submodule at the pinned revision).

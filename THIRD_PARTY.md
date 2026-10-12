# Third-party dependencies

Ironwood is GPL-3.0-or-later. Every dependency below is license-compatible as linked.

| Dependency | Version (dev) | License | Link mode | Used by | Status |
|---|---|---|---|---|---|
| Open CASCADE Technology (OCCT) | 7.9.x | LGPL-2.1 with OCCT exception | dynamic (system shared libs) | `iw_kernel` | required |
| Qt 6 (Core, Gui, Widgets) | 6.12 | LGPL-3.0 | dynamic (system shared libs) | `iw_render`, `iw_ui`, `ironwood` | required for GUI |
| oneTBB | 2023 | Apache-2.0 | dynamic | `iw_core` (private; job system) | required |
| GoogleTest | 1.18 | BSD-3-Clause | static, tests only (not shipped) | `tests/` | required for tests |
| pybind11 | 3.1 | BSD-3-Clause | header-only (from P1a) | future `api/` | not yet used |
| Tracy | tag in `cmake/IronwoodTracy.cmake` | BSD-3-Clause | static, profiling builds only | `iw_core` zone macros | optional (`IRONWOOD_TRACY`, default OFF) |
| nlohmann/json | 3.12.0 | MIT | header-only, vendored at `third_party/nlohmann/` (private to `iw_io`; not in public headers) | `iw_io` native document manifest (`document_io.cpp`) | required |

Planned (see docs/ARCHITECTURE.md section 2): Manifold (Apache-2.0), planegcs (LGPL-2.1+), Netgen (LGPL-2.1),
Clipper2 (BSL-1.0), QuickJS (MIT). LibreDWG (GPL-3) only as a separate converter process.
Build tools (CMake, Ninja, ccache) are not linked into the product.

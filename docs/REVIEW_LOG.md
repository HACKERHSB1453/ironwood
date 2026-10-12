# Review log

## Round 1 — 2026-10-12 — v0.1 → v0.2
Reviewers: Sonnet 5.5 (S), Haiku 5.5 (H). Both read FEATURE_INVENTORY.md + ARCHITECTURE.md v0.1. Triage by Opus.

### Accepted
| # | Finding | From | Change |
|---|---|---|---|
| 1 | LibreDWG loaded in-process makes the app GPL | S, H | Out-of-process converter only |
| 2 | Qt 6 is LGPL-3 → combined work is LGPL-3 at least | S, H | License decision moved before P0 (GPL-3 recommended) |
| 3 | SolveSpace solver is GPL-3; cuDSS & OptiX proprietary | S | SolveSpace rejected; NVIDIA libs user-installed only |
| 4 | Timeline is order-dependent; "independent" features can conflict via body state | S | Implicit state deps in evaluator (§4) |
| 5 | DAG parallelism small for linear histories; real win is prefix reuse | H | Reframed perf plan around prefix-reuse cache |
| 6 | OCCT shape hashes are pointer-based | S | Version stamps first; canonical hash later (§4) |
| 7 | Parallel OCCT may be non-deterministic → breaks cache/naming/tests | S, H | Determinism CI gate |
| 8 | "Pure function" unproven for OCCT | S | P0 thread-safety matrix |
| 9 | Fillets/shells are serial; table overstated MT | S | Table corrected |
| 10 | Naming: 50 cases too small; history coverage uneven; split/merge; sketch naming; silent rebinding | S, H | Naming = P0 data model + research track + mutation fuzzer |
| 11 | P0 overloaded; P1 is multi-year | S, H | P0 trimmed; P1 → P1a/P1b; reorder → P2; var/G2 fillets & modelled threads → P2 |
| 12 | Hand-rolled Vulkan renderer in P0 is a time sink; mesh shaders premature | H, S | Qt RHI (Vulkan backend) in P0; raw Vulkan only on profiler evidence |
| 13 | Ratio targets without measured Fusion baseline | S, H | Measurement protocol (§10), absolute targets, budgets for latency/memory |
| 14 | GPU FEA unjustified below ~1M DOF | S | CPU sparse direct first |
| 15 | CAM P5 too big; additive irrelevant | S | P5a/P5b; additive out of scope |
| 16 | P6 bundles unrelated items; P6/P8 no exit criteria; weak sim criterion | S | P6a/b/c; criteria added; NAFEMS + analytic |
| 17 | Parasolid/SAT readers = reversing proprietary formats | H | Only from published specs, else out |
| 18 | REA on Fusion binaries contradicts clean-room | H | Clarified: Fusion executables never disassembled; file-level analysis of user's own .f3d only + legal review. (v0.1 never proposed binary RE, but the wording was ambiguous — fixed.) |
| 19 | Missing: direct modelling, snapping/inference, edit-in-place, expressions, named views, i18n, lightweight assemblies, thickness analysis, version compare, templates | S, H | Inventory §12 |
| 20 | Out-of-scope items should be explicit | S | Inventory §13 |
| 21 | Profiling infra from P0; VAAPI/software video first-class; Clipper2 license spelled out | S | Done |

### Rejected / corrected
| Finding | From | Why |
|---|---|---|
| 2026 release items (Jan–Sep) may be fabricated | S, H | They came from a live web search of Autodesk's monthly posts this session; now cited with links. Sept items are flagged as from a search summary only. |
| "<5 ms pick" realistic (H) vs not (S) | — | Resolved as "~1 frame, async readback"; real number set by measurement |
| IKernel "future swap" | H | Agreed — downgraded to thin facade (accepted, not rejected; listed here because both views differ in emphasis) |

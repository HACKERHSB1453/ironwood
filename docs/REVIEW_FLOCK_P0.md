# Ironwood review triage (355 findings -> ~40 distinct issues, checked against current code)

## REAL
| id | sev | file:line | problem | fix | owner |
|---|---|---|---|---|---|
| R1 | med | src/render/viewport.cpp:57,153-157,305-312 | `releaseResources()` destroys the renderer (and its readback slots) but never resets `d_->inFlight`. A pick in flight at release leaves `inFlight>0` forever, so `render()` re-arms `QTimer(8ms)->update()` endlessly (125 Hz busy loop). The in-flight `done` and a still-`pending` `done` are dropped silently, contradicting the header ("done is still called with empty PickResult"). `~Viewport` drops them too. | In `releaseResources()`/dtor: `inFlight=0`; flush `pending->done` and (store them) in-flight dones with `PickResult{}` via queued call. | render |
| R2 | low-med | src/render/viewport.hpp:29-30, scene_renderer.cpp:471-505 | PickResult.bodyIndex indexes the vector given to setScene, but the readback completes 1-2 frames later. If `setScene` ran in between, the index refers to the old scene (may be out of range for callers). | Keep a scene generation counter; capture at submit; on completion if changed return empty PickResult (or document + bounds-check in ui). | render/ui |
| R3 | low | src/kernel/exchange.cpp:192-203, 348 | importStep/exportStep use XCAF app singleton + global `Interface_Static` units; header/THREAD_SAFETY say "one thread at a time" but nothing enforces it. Today only the CLI calls it (apps/ironwood-cli/main.cpp:111), but UI import will run it from JobSystem workers. | File-static `std::mutex` held across both functions (cheap, removes the whole class of reports). | kernel |
| R4 | low | src/naming/resolve.hpp:153 vs resolve.cpp:49 | Header promises a geometric fallback ("allowed ONLY when single candidate"); implementation says "out of scope for v0" and returns NotFound. Doc/impl drift; ResolveQuality::GeometricFallback never produced. | Reword header to "not implemented in v0 (P1a)". | naming |
| R5 | low | src/core/job.hpp:126-131 | If `enqueue` falls back to inline run (TBB bad_alloc, job.cpp:68-72), the job executes under `LatestWins::mutex_`; a job calling `latest()/submit()` would self-deadlock. Very rare. | Document, or submit outside the lock then swap under it. | core |
| R6 | low | src/CMakeLists (root CMakeLists.txt:24) | `find_package(Qt6 6.5 ...)` but `QRhiWidget` (Widgets) only exists from Qt 6.7. | Require 6.7. | build |
| N1 | nit | src/recompute/timeline.hpp:1-12 | Uses std::optional/std::min without including `<optional>`/`<algorithm>` (transitive). | add includes | recompute |
| N2 | nit | src/core/undo.hpp:118 | `undo()` documented "must not fail" but not `noexcept`; `clean_` is `long` vs `size_t` index (casts everywhere). | `undo() noexcept`; use signed sentinel only | core |
| N3 | nit | src/io/document_io.cpp:~135 | save path renames target->backup then staging->target: crash between leaves only `.old-*` (known F3: no fsync). | Already in F3; no new action. | io |

## Counts (distinct issues; finding counts approximate)
- REAL: 6 (+3 nits) - maps to ~25 findings (viewport pick ~10, STEP thread ~12 shared with DESIGN, resolve ~3)
- FIXED: 2 distinct (~24 findings): Transform::apply includes translation (math.hpp:90-91); Timeline::insert advances marker when inserting before it (timeline.cpp:10-13)
- DESIGN: ~14 distinct (~150 findings): see below
- FALSE: ~16 distinct (~150 findings)

## Notable FALSE
- "C++23 vs C++20", std::expected / move_only_function need C++23 (~20): ARCHITECTURE.md:22 deliberate.
- relaxed memory order on CancelToken (~14): pure monotonic flag, no data published through it.
- ElementName vector<ElementName> "infinite size/recursive" (~24): legal; parse depth-limited (element_name.cpp kMaxDepth); by-value set() is moved into storage (naming_map.cpp:25).
- std::hash<StrongId> in global namespace (ids.hpp:43) is the correct C++20 form, header-only template, no ODR issue.
- ShapeHistory map/set need operator<: defaulted <=> provides all relational operators.
- Document::nextId single counter (io): intentional unique id space; loader validates it (document_io.cpp).
- FeatureRegistry::create "double wrapping": factory/create types equal, no wrapping.
- Evaluator "copies timeline internally": it reads it synchronously under its mutex; Feature::compute const = pure by contract.
- ShapeStamp not crypto: cache key, documented. Transform::apply "ignores translation": stale, fixed.
- Error::message std::string, NotImplemented in enum, parseLengthUnit case-sensitive (file-format token), massProperties lacks CancelToken (fast op).
- Module order / IronwoodDefaults missing: files exist (cmake/), order matches deps.
## Notable DESIGN
- JobHandle::wait self-deadlock and JobSystem dtor blocking: documented contract (job.hpp:56,76,108); wait() does not hold the mutex while blocked, so the "job holds mutex" reasoning is wrong.
- Kernel thread-safety UNVERIFIED: P0 deliverable is the matrix (THREAD_SAFETY.md, ARCHITECTURE s4); 4-thread determinism test exists (test_determinism.cpp:171); shared-shape concurrent reads not stress-tested (known, matrix).
- Shape::impl()/occt_access (F7), withStamp, isSame excludes stamp (documented).
- GuiPrivate/QRhiWidget: ARCHITECTURE s2 chooses Qt RHI; QT_NO_PRIVATE_MODULE_WARNING set, Qt 6.12 installed.
- tessellate blocking/no async variant: documented "invoke from a job".
- Pick callback async on GUI thread, NavPreset [VERIFY]: documented/accepted. Evaluator cache copies State per feature (perf, F9).

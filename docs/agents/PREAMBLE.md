# Agent preamble (read before any Ironwood task)

You are implementing one task of Ironwood (GPL-3.0-or-later clean-room Fusion-like CAD, C++23, OCCT 7.9.3,
Qt 6.12, oneTBB, GTest, Eigen 5 + Boost installed). Repo: /home/hakanhsb/AI_Assistants/ironwood-cad (git).
READ FIRST: docs/P1A_TASKS.md ("Cross-module contracts" + your task row), docs/ARCHITECTURE.md §4–§5,
docs/P0_FOLLOWUPS.md (rows referenced by your task), and the existing headers you build on.

## Hard rules
- Edit ONLY the files your task owns. Other agents edit other files IN PARALLEL in the same tree. If something
  outside your ownership must change, STOP and report it.
- Shared build files: do NOT edit CMakeLists.txt or tests/CMakeLists.txt. Modules src/sketch, src/features,
  api/ and fuzz/ are auto-registered once their own CMakeLists.txt exists. Register tests by creating
  `tests/<your_test>.cmake` with your `iw_add_module_test(<name> LIBS ...)` line — create the .cmake only
  AFTER the test .cpp exists (otherwise everyone's CMake regeneration breaks).
- Use YOUR OWN build dir: `cmake --preset debug -B build/<TASK>`; `cmake --build build/<TASK> --target <yours>`;
  `ctest --test-dir build/<TASK> -R <your tests> --output-on-failure`. Before finishing also build+test with
  clang: `CXX=clang++ cmake --preset debug-clang -B build/<TASK>-clang`. If the full build fails only because of
  another task's in-progress files, build just your targets and say so.
- **CPU courtesy (user request):** always build with `-j 6` (e.g. `cmake --build build/<TASK> -j 6 --target ...`) and run ctest with `-j 4` at most — several agents build in parallel on the user's desktop machine.
- **Never** copy anything from ~/AI_Assistants/aviary, any `.env` file, or any API key/token into this repo.
- Zero warnings (-Werror). SPDX `// SPDX-License-Identifier: GPL-3.0-or-later` on new files. No Autodesk
  code/names/icons. No git commits.
- GUI tests: `QT_QPA_PLATFORM=offscreen` ONLY — never open windows on the user's desktop. Never `pkill -f`
  (it can kill your own shell); kill by PID.
- Write real tests for every acceptance criterion; never weaken one to pass — report it instead.
- Keep scope tight and FINISH: implement, test, report.

## Report (≤300 words)
What you built, each acceptance criterion pass/fail, exact test counts, deviations, follow-ups.

# Local patches to planegcs

Base: FreeCAD tag `1.1.4`, commit `4fd3bf320d9566a27e60069fc8387448aaa3a094`.
Every file differs from the base only in the lines listed here.

## 1. `SketcherGlobal.h` replaced by `compat/planegcs_compat.h` (GCS.h, Geo.h, Constraints.h)

`#include "../../SketcherGlobal.h"` becomes `#include "planegcs_compat.h"`.
FreeCAD's header defines `SketcherExport` for Windows DLL export. The compat header defines it as
empty, which is correct for a static Linux build.

## 2. Boost graph header path (GCS.cpp)

`#include <boost_graph_adjacency_list.hpp>` (a FreeCAD-internal wrapper) becomes
`#include <boost/graph/adjacency_list.hpp>`. The header is the same Boost Graph Library header.

## 3. Logging and timing shims (no source change, compat/ headers)

planegcs calls `Base::Console()` (`log`, `warning`) and `Base::TimeElapsed` from FreeCAD's `Base`
library. `compat/Base/Console.h` provides those names:

- `Base::Console().log/warning/message/error` discard their output. The solver library does not
  write to stdout or stderr.
- `Base::TimeElapsed` is a `std::chrono::steady_clock` wrapper with the same static `diffTimeF`.

`compat/FCConfig.h` is an empty header that satisfies `#include <FCConfig.h>`.

## Notes (not modifications)

- The `System` constructor calls `Eigen::setNbThreads(1)`. This is a process-wide Eigen setting,
  left as upstream. Nothing else in Ironwood uses Eigen.
- Built against the system Eigen 5.0.1 and Boost 1.92 headers without source changes.
  `EIGEN_SPARSEQR_COMPATIBLE` is defined by upstream when Eigen's sparse module is available,
  and the sparse QR path is used.

## Adapter note: F20-sym (not a planegcs patch)

`System::addConstraintP2PSymmetric` adds two constraints but returns only the last index. The Ironwood adapter
(`src/sketch/solver/sketch_solver.cpp`, case `Symmetric`) used to register just that index for `rescaleConstraint`,
so the first constraint (perpendicular / point-on-perp-bisector) kept the scale computed at construction time, when
every parameter was still 0. That made `SymmetricAboutLineAndAboutPoint` fail with a bogus conflict. The adapter now
adds both constraints itself and registers both. The vendored planegcs sources are unchanged.

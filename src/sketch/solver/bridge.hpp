// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Bridge between the sketch document model (sketch/sketch.hpp: entities plus constraint and dimension
// records) and the constraint solver (sketch/solver/sketch_solver.hpp). Qt-free and OCCT-free.
//
// Every constraint and dimension kind of sketch.hpp is mapped. Dimension expressions are evaluated through
// a core ParameterTable (nullptr = literals only). A solve either writes the solved coordinates back into the
// Sketch (status Solved) or leaves the Sketch untouched. Diagnostics name the sketch RECORD ids
// (SketchConstraintId / SketchDimensionId), never the solver's internal ids.
//
// Encoding of the solver id space (internal, documented here for debugging): entity E uses solver id 4E for the
// entity itself, and 4E+1..4E+3 for auxiliary points (line start/end, circle or arc centre, arc start/end).

#include <cstdint>
#include <string>
#include <vector>

#include "core/params.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/sketch_solver.hpp"

namespace iw::sketch {

/// Outcome of a solve or diagnosis of a Sketch. Constraint and dimension diagnostics are mapped back to the
/// sketch record ids.
struct SketchSolveReport {
    SolverStatus status = SolverStatus::Failed;
    std::string message;   ///< Detail for Failed / InvalidInput (e.g. unknown parameter, bad dimension value).
    int dof = -1;          ///< Degrees of freedom; -1 when no diagnosis ran (InvalidInput).

    std::vector<SketchConstraintId> conflictingConstraints;
    std::vector<SketchDimensionId> conflictingDimensions;
    std::vector<SketchConstraintId> redundantConstraints;
    std::vector<SketchDimensionId> redundantDimensions;
    std::vector<SketchConstraintId> partiallyRedundantConstraints;
    std::vector<SketchDimensionId> partiallyRedundantDimensions;

    bool ok() const noexcept { return status == SolverStatus::Solved; }
};

/// Solves `sketch`. On Solved the solved geometry is written back into `sketch` (ids and kinds are kept). On any
/// other status the sketch is unchanged. Dimension expressions are evaluated against `params` (nullptr: literals
/// only; any parameter reference then fails with InvalidInput). Diagnostics are always filled when a solve ran.
SketchSolveReport solveSketch(Sketch& sketch, const ParameterTable* params = nullptr);

/// Same analysis as solveSketch (DOF, conflicts, redundancies) but never modifies `sketch`.
SketchSolveReport diagnoseSketch(const Sketch& sketch, const ParameterTable* params = nullptr);

} // namespace iw::sketch

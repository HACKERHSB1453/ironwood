// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Qt-free evaluation of a document snapshot, for the GUI's background jobs. features::Document is not copyable and
// its evaluator is private, so the UI copies the timeline and the parameter table on the GUI thread and evaluates
// the copies on a worker with its own recompute::Evaluator. The pipeline is the one of Document::evaluate (sketch
// pre-pass, then the evaluator); the Evaluator's prefix cache lives in the evaluator the caller passes in, so keeping
// one evaluator per document keeps rollback drags and parameter edits cheap.

#include <map>
#include <memory>

#include "core/cancel.hpp"
#include "core/ids.hpp"
#include "core/params.hpp"
#include "core/result.hpp"
#include "features/document.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/timeline.hpp"
#include "tess/tessellate.hpp"

namespace iw::ui::timeline {

/// A document evaluation with the renderable meshes of its bodies.
struct EvalOutput {
    features::DocumentEvaluation eval;
    std::map<BodyId, std::shared_ptr<const tess::MeshData>> meshes;
    double evalMs = 0;
    double tessellateMs = 0;
};

/// Same pipeline as features::Document::evaluate, on the given timeline and parameter table. Errors: Cancelled, or the
/// evaluator's own errors.
Result<features::DocumentEvaluation> evaluateTimeline(const recompute::Timeline& timeline, const ParameterTable& params,
                                                      recompute::Evaluator& evaluator, CancelToken cancel = {});

/// evaluateTimeline followed by tessellation of every produced body (tessellation failures are reported as errors).
Result<EvalOutput> evaluateAndTessellate(const recompute::Timeline& timeline, const ParameterTable& params,
                                         recompute::Evaluator& evaluator, CancelToken cancel = {});

/// Volume (mm^3) of a body of an evaluation; nullopt when the body is absent or has no volume.
std::optional<double> volumeOfBody(const features::DocumentEvaluation& eval, BodyId body);

} // namespace iw::ui::timeline

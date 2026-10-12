// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <string>
#include <string_view>

#include "core/result.hpp"
#include "sketch/sketch.hpp"

namespace iw::features {

/// Serialises a sketch to compact JSON text (plane, entities with ids and construction flags, constraint and
/// dimension records, and the id counter). Ids are persisted: they anchor sketch-level element names.
/// Errors: Internal only if the sketch's id counter cannot be observed.
Result<std::string> sketchToJson(const sketch::Sketch& sk);

/// Rebuilds a sketch with exactly the serialised ids. The public Sketch API allocates ids sequentially, so records
/// are replayed in id order and gaps are consumed with temporary points that are removed again; the id counter is
/// then advanced to the stored `nextId`, so ids are never reused after a reload either.
/// Errors: ParseError (malformed JSON or inconsistent ids), InvalidArgument (geometry rejected by the sketch).
Result<sketch::Sketch> sketchFromJson(std::string_view text);

} // namespace iw::features

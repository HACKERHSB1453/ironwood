<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Ironwood native document format - v0 (UNSTABLE)

**Status: v1, unstable. No compatibility or migration promise until a stable version.** v1 added `parameters`; v0 manifests still load (empty parameter table). Later versions will be a single
zip container with the same logical layout ("directory-in-zip").

## Layout (a directory, suffix `.ironwood`)

```
mydoc.ironwood/
  manifest.json          required; format version, metadata, timeline
  blobs/<name>.brep      optional; OCCT BREP text/binary blobs for imported, non-parametric shapes
```

## manifest.json

```json
{
  "format": "ironwood-document",
  "version": 1,
  "name": "Part1",
  "displayUnit": "mm",
  "nextId": 12,
  "parameters": [
    { "name": "d1", "expression": "2 in + 5 mm", "unit": "mm", "comment": "wall", "favourite": true }
  ],
  "timeline": {
    "rollbackMarker": null,
    "suppressed": [3],
    "features": [
      { "id": 1, "kind": "extrude", "params": { "...": "feature-defined" } }
    ]
  },
  "blobs": [ { "id": 7, "file": "blobs/7.brep", "sha256": "..." } ]
}
```

Rules:
* All lengths in mm, angles in radians, regardless of `displayUnit`.
* IDs (`FeatureId`, `BodyId`, `ComponentId`) are unsigned 64-bit integers, > 0, never reused. Serialised as JSON numbers
  (< 2^53 in v0).
* Element references inside `params` are stored as `ElementName::toString()` strings, **never indices**.
* `params` is the object returned by `Feature::paramsJson()`; unknown `kind` values make load fail with NotFound.
* Derived data (tessellation, recompute caches) is never stored in v0.
* Readers must reject `version` greater than they support with ParseError.
* `nextId` is in [1, 2^53) and greater than every feature id; violations are ParseError on load.
* `timeline.rollbackMarker` is `null` when no marker is set (equivalent to "at end"), otherwise an integer in
  [0, feature count]. Writers emit `null` whenever the effective marker is at the end.
* `timeline.suppressed` lists the ids of suppressed features; each must name a feature in `features`.
* `features` order is timeline order. Each `params` must be a JSON object; loaders pass its compact text to
  `FeatureRegistry::create`, so key order on disk is not significant.
* `parameters` (v1) lists every user parameter in insertion order with `name`, `expression` (source text), `unit`
  ("" = inferred), `comment` and `favourite`. Loaders add them in that order. A version 0 manifest has no
  `parameters` and loads as an empty table; a version 1 manifest without `parameters` is a ParseError.
* Expressions in feature params are stored as text and evaluated on load-and-recompute against this table.
* v0 writers do not emit `blobs`; readers ignore unknown top-level keys.
* Manifests are UTF-8 JSON, 2-space indented, with a trailing newline. Saving replaces an existing document
  directory via a staging directory and rename; a failed save leaves the previous directory in place.

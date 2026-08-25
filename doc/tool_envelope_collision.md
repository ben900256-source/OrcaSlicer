# Tool-envelope collision kernel

This subsystem provides headless collision checks between a moving printer tool and material that has already been deposited. It is a standalone `libslic3r` API: this patch does not connect it to normal slicing, G-code generation, `ConflictChecker`, or Z Contouring.

## Coordinates and units

Public poses, segment endpoints, widths, heights, Z values, margins, clearances, and Z-raise results are unscaled millimeters. Polygon geometry remains in OrcaSlicer's scaled integer `ExPolygons` representation. Conversion happens only while reading or writing JSON and while translating geometry to a queried pose.

A pose is the nozzle-tip position. Envelope XY geometry is local to the tip and its slice Z interval is added to the pose Z.

Extrusion Z is the nozzle/bead-top Z. A deposited endpoint at `z_mm` with height `h` occupies `[z_mm - h, z_mm]`:

```text
                    toolhead slice
                  +----------------+
                  |                |
                  +-------+--------+  local z_max
                          |
                       nozzle
                          V
 nozzle-tip origin  ------+--------  local z = 0
                     deposited bead  bead top = extrusion Z
                    (______________)
                     bead bottom      extrusion Z - height
```

For a sloped deposited segment, the scene stores the conservative enclosing interval from the lower endpoint bottom to the higher endpoint top. A zero-length segment is valid and has a circular bead footprint.

## Envelope JSON

Schema version 1 requires `origin: "nozzle_tip"`:

```json
{
  "version": 1,
  "origin": "nozzle_tip",
  "name": "example 0.4 mm tool",
  "slices": [
    {
      "name": "heater block",
      "zone": "hard_collision",
      "z_min_mm": 0.2,
      "z_max_mm": 5.0,
      "geometry": [
        {
          "contour": [[-8, -4], [8, -4], [8, 4], [-8, 4]],
          "holes": []
        }
      ]
    }
  ]
}
```

Zone strings are `deposition`, `hard_collision`, `thermal`, and `toolhead`. A bare point ring may replace a polygon object when it has no holes. Serialization always emits the canonical `{ "contour": ..., "holes": ... }` form, so holes round-trip without being flattened.

Parsing rejects unsupported versions, a different origin, missing or empty geometry, non-finite coordinates and Z values, invalid Z ranges, degenerate or self-intersecting contours, and holes that cross or lie outside their contour. Errors are `Slic3r::InvalidArgument` values whose messages identify the failing field.

## Deposited scene and index

`PrintedScene` owns `DepositedSegment` records in insertion order. Each record carries a caller-provided primitive ID and sequence index. The scene stores a capsule AABB using half the bead width and the conservative vertical interval described above.

The XY index is a deterministic uniform grid with 5 mm cells. A capsule is referenced by every cell its AABB overlaps. Queries visit only intersected cells, deduplicate primitive indices, sort them by insertion order, and then apply exact AABB and Z filters. `PrintedSceneQueryStats` exposes visited-cell, deduplicated-candidate, and exact-result counts for reproducible performance tests; no wall-clock behavior is part of the contract.

## Policies and exact pose checks

`ToolContactPolicy` selects each zone independently and supplies an XY margin for each one. By default deposition and thermal slices are ignored, while hard-collision and toolhead slices are checked. All margins default to zero.

An obstacle footprint is built with the existing Clipper open-polyline offset at `width / 2 + zone margin`. It is intersected with a translated copy of the tool slice. This preserves asymmetric tool shapes and polygon holes. OrcaSlicer's `EPSILON` is included conservatively, so boundary contact is a collision within the library tolerance.

The signed vertical clearance is positive when two relevant Z intervals are separated and non-positive when they touch or overlap. Collision diagnostics include hit time, primitive ID, obstacle sequence, slice index/name, zone, signed clearance, and required Z raise. All-hit queries collapse duplicate slice hits into the earliest report for each deposited primitive and sort by hit time, obstacle sequence, primitive ID, then slice order.

## Adaptive sweeps

Sweeps first query each active slice's complete translated XY and Z bounds. Each candidate is then checked recursively over `[0, 1]`:

1. Enclose the interval's swept Z range.
2. Translate the slice to the interval midpoint.
3. Offset it by half the interval's XY travel, which conservatively encloses every translation during that interval.
4. Discard the interval when either Z or XY proves it safe.
5. Otherwise subdivide left-first.

Subdivision stops when XY travel is at most `RESOLUTION` and Z travel is at most `EPSILON`, or at depth 32. A still-possible leaf is reported conservatively as an interval. Consequently, a thin obstacle cannot be skipped between sampled poses. The tradeoff is a possible conservative hit near geometry or tolerance boundaries; this kernel is designed for clearance validation, not contact-time metrology.

Required Z raise is the maximum upward displacement, plus `EPSILON`, needed to put the active slice bottom above every colliding obstacle interval. `required_z_raise_for_sweep` returns the maximum across the collapsed obstacle reports. It does not plan a detour or prove that the raised move fits machine limits.

## Ordered toolpath validation

`ToolpathClearanceValidator` consumes moves in their final execution order. It checks a move against the current scene, records its earliest diagnostic, and only then inserts that move's extrusion. The move ID becomes the deposited primitive ID and input position becomes its sequence index. Travel moves never deposit material.

The future production integration point is `GCode::_extrude`. At that point OrcaSlicer has final `ExtrusionPath` ordering, width, height, role, and active extruder. It also converts contoured path-point Z to absolute nozzle Z by adding `m_nominal_z`. That makes `_extrude` the appropriate place to construct ordered validation moves later. It is intentionally unwired here so ordinary slicing and emitted G-code cannot change in this patch.

## Limitations and extension paths

- Deposited beads are round-ended constant-width capsules; variable width must be split into primitives.
- Sloped beads use an enclosing Z interval rather than a swept 3D solid.
- Tool orientation is fixed; poses translate but do not rotate the envelope.
- Thermal behavior is a selectable geometric zone, not a heat-transfer simulation.
- The grid is incremental and has no primitive removal or mutation API.
- Conservative sweep leaves may report contact slightly before the exact mathematical time.

Natural extensions are per-segment width sampling, rotated or articulated tool poses, printer-limit-aware Z-hop planning, richer role/extruder policy, and the documented ordered `GCode::_extrude` adapter.

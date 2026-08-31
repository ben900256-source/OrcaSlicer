# Organic support contact/extrusion association

This diagnostic layer relates realized round-tip Organic support contacts to the
model extrusion paths that the slicer has actually planned above them. It does
not alter extrusion geometry, roles, ordering, speed, flow, support placement,
or G-code.

## Pipeline timing and lifetime

Associations are rebuilt for every print object after perimeter, infill, and
support path simplification and before conflict checking. At that point both
freshly generated and slicedata-loaded paths have their final pre-emission
geometry. Shared objects copy their finalized layers and contacts first, then
derive their own identical association values.

The values live on `PrintObject` and are available through a const accessor.
They are cleared when model layers, support layers, contacts, shared-object
state, or dependent slicing steps are invalidated. They are deliberately not
written to slicedata, 3MF projects, or profiles: slicedata retains only the
realized contacts, and associations are recomputed from those contacts and the
loaded final paths.

The entire feature is gated by `tree_support_round_tip`. Objects for which the
option is disabled have an empty association list.

## Slice Preview diagnostics

When a completed `Print` is loaded into Slice Preview, `GCodeViewer` snapshots
the association records and the exact finalized path chains by value. The
snapshot is expanded with each `PrintInstance::shift_without_plate_offset()`;
it does not retain `PrintObject` pointers and is discarded whenever preview
data is reset or replaced. Standalone imported G-code and prints with round tips
disabled therefore have no overlay data.

The preview legend shows an **Organic support diagnostics** eye toggle only
when at least one eligible contact snapshot exists. Its application-local
preference is stored as `preview_show_organic_support_associations`, defaults to
off, and is not serialized into projects, profiles, slicedata, or G-code.
Disabling the overlay or rebuilding preview data clears the selected contact.

Visible contacts are filtered only by their supported-layer print Z and the
current layer-slider range. Move-slider position and extrusion-role visibility
are intentionally ignored because the records describe finalized pre-emission
geometry rather than emitted traversal order. Contacts are drawn at
`support_tip_z` as physically sized translucent footprints (teal when associated
and red when unassociated). A selected contact adds a yellow outline and its
vertical clearance connector. Clicking the nearest projected visible footprint
uses its projected radius with a minimum screen-space hit target; clicking empty
preview space clears the selection.

For the selected contact, every physical association interval is clipped back
onto its exact tessellated chain and drawn in white. Forward and backward
planned-material spans are orange and blue. Anchor intervals are green, with
circles for pin anchors and diamonds for lower-track anchors. Tolerance-only
associations draw only their closest-point marker and connector: they never
invent a physical overlap or unsupported span. Batched `GLModel` geometry is
rendered after normal toolpaths and before ImGui with depth writes and depth
testing disabled, making the diagnostic an x-ray overlay without modifying
LibVGCode data.

The selected-contact legend section is scrollable and reports object/instance
IDs, contact geometry and Z values, clearance and tolerance, status, printable
extrusion IDs, localized and stable roles, width, distance, overlap, path
interval, tangent state, directional spans, and anchor IDs.

## Geometry representation

Only `LayerRegion::perimeters` and `LayerRegion::fills` are inspected. Recursive
collections are flattened while loops and multipaths remain continuous chains.
Each leaf keeps its own role and width. Force-no-extrusion leaves, degenerate
segments, support extrusions, skirts, brims, and other non-model collections are
excluded. Accepted roles cover perimeter and overhang walls, gap fill, ironing,
sparse and solid fills, surfaces, and bridge/internal-bridge fills.

Ordinary paths use their final simplified points. When arc fitting is enabled
and a path has fitted-arc metadata, each emitted arc is tessellated for the
diagnostic index. For radius `r` and allowed chord error `e`, the maximum angular
step is

```
2 acos(1 - e / r)
```

and the fitted sweep is divided into enough equal steps not to exceed it. Exact
arc endpoints are retained. Here `e` is the association tolerance, defined as
`max(EPSILON, print resolution)` and reported in the JSON.

## Contact association

Every finalized contact receives its index in the already sorted contact vector
as its deterministic contact ID. A leaf extrusion of width `w` is associated
with a pin of radius `rp` when the minimum center-to-centerline distance `d`
satisfies

```
d <= rp + w / 2 + tolerance.
```

The reported physical overlap does not include the tolerance. On each segment
the quadratic interval for

```
|segment(t) - pin_center| <= rp + w / 2,  0 <= t <= 1
```

is converted to continuous-chain arclength and adjacent intervals are unioned.
Disjoint departures and re-entries remain separate records, including a path
that doubles back below one pin. A tolerance-only association has zero nominal
overlap, no physical path interval, and no unsupported-span result.

The local tangent follows stored chain direction. In the interior it is the
segment unit vector. At a vertex it is the normalized sum of the incoming and
outgoing directions. A cusp or exact double-back is marked ambiguous and uses a
deterministic adjacent-segment fallback. "Approximately centered" means
`d <= tolerance`.

## Planned-material unsupported spans

The only anchors are:

- nominal pin footprints for every contact targeting the current model layer;
- swept footprints of finalized model extrusion tracks on the immediately
  preceding model layer.

For current width `wc` and lower-track width `wl`, a lower track anchors the
current centerline where it lies inside the lower segment capsule expanded by

```
(wc + wl) / 2.
```

Candidate segment pairs come from the existing AABB line index. Exact filtering
splits the current segment at the lower segment's projection boundaries and
solves the corresponding endpoint-circle or interior-strip quadratic on each
piece. Thus parallel tracks may anchor through width overlap even when their
centerlines never intersect.

Forward and backward spans begin at the corresponding edge of the current
pin's unioned physical interval and end at the first subsequent anchor boundary.
All anchors tied at that boundary are retained. An overlapping anchor returns
zero. An open chain with no later anchor returns a null span and the unanchored
distance to its endpoint. A closed chain is traversed no more than once; after a
full wrap, the opposite edge of the same pin may be the terminating anchor.

These values describe planned material only. They neither read nor reinterpret
slice polygons, overhang classes, bridge detection, or speed-classification
data.

## Diagnostic IDs and coordinate space

An extrusion ID contains model layer ID, region index, source
(`perimeter`/`fill`), every recursive collection index, and the leaf path index
within a path, loop, or multipath. The printable form (for example
`L12:R0:fill:E3.1:P0`) is derived solely from those values and never from a
pointer.

Support-contact JSON schema 2 retains the schema-1 fields and adds the structured
and printable IDs, stable role/source names, width, closest point, distance,
overlap and arclength interval, tangent/ambiguity, centered state, and explicit
`planned_material_unsupported_span` directions. Positions are converted from
object-local to plate-local coordinates per instance; distances, widths,
tangents, and arclengths are translation invariant.

## Scope for future reuse

The module exposes a pure geometry entry point, a public finalized-path extractor,
and `clip_path_chain_interval()` for resolving chain-arclength intervals across
leaf boundaries and closed-loop seams. Future diagnostics may reuse its stable
final-path IDs and capsule interval machinery. A slice-polygon comparison, if
added, must be a separately named metric. Seam gaps, seam splitting, final
traversal reversal, per-instance G-code ordering, and other emission-time changes
remain outside this pre-emission association model.

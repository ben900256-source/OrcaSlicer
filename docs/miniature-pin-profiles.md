# Miniature Pin profiles for the Prusa XL

The built-in miniature family is an easy starting point for a Prusa XL 5-tool machine with a 0.25 mm nozzle installed on physical tool 2 and 0.4 mm nozzles on tools 1, 3, 4, and 5. It intentionally prints every model and Organic Pin support role through tool 2. It does not mix nozzle sizes within a print.

These profiles are starting points, not universal quality guarantees. Calibrate temperature, flow ratio, pressure advance, and maximum volumetric speed for the specific PLA before printing a miniature. Surface quality and support-removal force still require comparison prints on the target machine.

## Source provenance

The reusable miniature settings were adapted from the community's [Dungeons and Derps High Quality Settings 2.0](https://www.reddit.com/r/FDMminiatures/comments/1rbnet7/high_quality_profile_version_20_is_here/). In a later discussion, HOHansen [recommended the newer ObscuraNox profile](https://www.reddit.com/r/FDMminiatures/comments/1tuxp59/latest_hohansen_settings/) as the easier current baseline instead of his older 0.2 mm settings.

- Retrieved: 2026-08-24
- Source archive: `Dungeons and Derps 2.0 Settings.zip`
- Source archive SHA-256: `0E53D0F1CA113BFA70BF62FC666E4B1840668A9C4A8F33EB81B7252429DBCF91`

The source profile was made for different hardware. This fork reuses its geometry and conservative print-speed ideas but deliberately does not copy Bambu firmware G-code, pressure advance, retraction, Z-hop, or motion limits.

## Adapted settings

| Area | Community value retained | XL/Pin adaptation |
| --- | --- | --- |
| Walls and infill | Classic walls, inner/outer/inner order, 3 walls, 20% gyroid | Ironing remains off |
| Line widths | Default 100%, first layer 125%, inner 120%, outer 115%, infill/solid 110%, top 105% | Percentages resolve against tool 2's 0.25 mm nozzle |
| Speeds | Outer/top 35, inner 55, solid 45, sparse 65, gap 30, support 45, travel 325 mm/s | XL process acceleration values are retained instead of the source machine's limits |
| Adhesion and precision | 2-loop skirt, 6 mm outer brim, reduced wall crossing, curled-perimeter slowdown, 0.001 mm slicing resolution | The brim uses zero object gap so it remains coupled to the miniature and support bases; applied through a hidden cross-printer base |
| Supports | Automatic Organic supports | Supports are allowed everywhere, including model-supported islands |
| Pin contact | Community interface geometry replaced | No interface layers; direct-contact round Pin with a continuous curved taper, initially a 1.6 mm Organic approach and up to 0.8 mm straight tip; 0.4 mm support width/tip, 2 mm branches, 5 degree diameter angle, 3 mm branch distance, 13.33% density |
| Layer variants | 0.06 mm source baseline | Experimental zero-clearance contact for both Balanced and Ultra Detail; the Pin axis follows moderate underside normals and is reduced or made vertical when needed for printable support |
| Extrusion roles | Not retained | Walls, infill, solid/top/bottom surfaces, bridges through their owning roles, and support base/interface are fixed to tool 2 |
| Filament | Generic PLA starting point | 3 mm³/s cap, full cooling, and tool-index-aware XL pressure-advance G-code |

## Machine preparation

Prusa documents printing with different XL nozzle diameters as [experimental](https://help.prusa3d.com/article/experimental-printing-with-different-nozzle-diameters_821176). After installing the 0.25 mm nozzle on tool 2:

1. Run **Control > Calibrations & Tests > Tool Offset Calibration** on the printer.
2. Select the matching minimum and maximum layer-height ranges for every tool.
3. Keep the inherited per-tool `M862.1` nozzle checks enabled.
4. Confirm the slicer shows `T1/T3/T4/T5 = 0.4 mm` and `T2 = 0.25 mm` before exporting.

The firmware and G-code use zero-based tool commands, so physical tool 2 is emitted as `T1` in G-code.

Do not call the profiles print-proven until a supervised physical coupon has confirmed island survival, underside quality, removal force, and surface marking on the prepared machine.

## Experimental zero-clearance contact

The built-in values use zero Top Z distance. Sparse Organic branches end in small round Pins that directly meet the underside. Surface normals are sampled around the final rendered contact on the transformed model, with neighboring-surface and height checks.

Within the configured Organic branch angle, the slicer tries the full surface-normal tilt first. It extends the curved approach downward toward the branch junction or root before reducing the angle. Short branches may also shorten the straight tip segment to leave room for the curve. The lower attachment, incoming tangent, contact location, and tip diameter stay fixed.

Each candidate is checked after collision clipping, bed clipping, and fragment removal. Every component must remain attached through the transition, movement must stay within the configured layer limit, and the Pin runup must retain at least 98% direct layer overlap through the final tip. Vertical candidates, including unresolved-surface fallbacks, receive the same checks. If no Pin fits, the original Organic terminal is retained only when its complete connection passes the Organic attachment and movement checks. Otherwise slicing stops with the object and contact location. Adjust the contact or support settings before reslicing.

A footprint too narrow for the configured inset perimeter is retried with a narrower extrusion line so the round tip does not disappear from the toolpaths. This remains an experimental FFF contact mode, not a claim that same-PLA contacts behave like cured resin: direct contact can fuse to the miniature or leave nubs, pits, whitening, and other marks.

Before using either profile for production miniatures, print a supervised physical coupon with at least six identical supported features using the same dried PLA, tool 2, cooling, and calibration planned for the model. Confirm that every island survives, inspect underside sag, stringing, and sharpness under consistent lighting, record the force and tools needed for removal, and check every contact for nubs, pits, whitening, fused supports, or torn material. If contacts fuse or mark the model, switch to a nonzero Top Z distance; those configured-gap Pins remain vertical. Repeat the coupon after any material, temperature, cooling, or calibration change.

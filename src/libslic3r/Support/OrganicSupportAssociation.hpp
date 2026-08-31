#pragma once

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Line.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

class PrintObject;

namespace OrganicSupport {

enum class ExtrusionSource : unsigned char {
    Perimeter,
    Fill
};

struct ExtrusionId
{
    size_t              layer_id { 0 };
    size_t              region_id { 0 };
    ExtrusionSource     source { ExtrusionSource::Perimeter };
    std::vector<size_t> entity_indices;
    size_t              leaf_path_index { 0 };

    std::string to_string() const;

    bool operator==(const ExtrusionId &rhs) const;
    bool operator!=(const ExtrusionId &rhs) const { return !(*this == rhs); }
    bool operator<(const ExtrusionId &rhs) const;
};

struct PathInterval
{
    double begin { 0. };
    double end { 0. };

    double length() const { return std::max(0., end - begin); }
};

enum class AnchorType : unsigned char {
    Pin,
    LowerTrack
};

struct PlannedMaterialAnchor
{
    AnchorType                 type { AnchorType::Pin };
    size_t                     contact_id { 0 };
    std::optional<ExtrusionId> extrusion_id;
    PathInterval               interval;
};

struct DirectionalUnsupportedSpan
{
    std::optional<double>              span;
    std::optional<double>              unanchored_distance;
    std::vector<PlannedMaterialAnchor> anchors;
};

struct PlannedMaterialUnsupportedSpan
{
    DirectionalUnsupportedSpan forward;
    DirectionalUnsupportedSpan backward;
};

struct ExtrusionAssociation
{
    ExtrusionId                                  extrusion_id;
    ExtrusionRole                                role { erNone };
    double                                       width { 0. };
    Vec2d                                        closest_point { Vec2d::Zero() };
    double                                       centerline_distance { 0. };
    double                                       nominal_overlap_length { 0. };
    std::optional<PathInterval>                  path_arclength_interval;
    Vec2d                                        tangent { Vec2d::Zero() };
    bool                                         tangent_ambiguous { false };
    bool                                         approximately_centered { false };
    std::optional<PlannedMaterialUnsupportedSpan> planned_material_unsupported_span;
};

struct Contact
{
    size_t                               contact_id { 0 };
    size_t                               supported_layer_id { 0 };
    double                               supported_layer_print_z { 0. };
    double                               model_contact_z { 0. };
    double                               support_tip_z { 0. };
    Vec2d                                center { Vec2d::Zero() };
    double                               radius { 0. };
    double                               nominal_clearance { 0. };
    double                               association_tolerance { 0. };
    std::vector<ExtrusionAssociation>    associations;
};

// Public geometry inputs keep the association calculations independently
// testable. Coordinates, widths, radii and arclengths are in millimetres.
struct Pin
{
    size_t contact_id { 0 };
    size_t supported_layer_id { 0 };
    double supported_layer_print_z { 0. };
    double model_contact_z { 0. };
    double support_tip_z { 0. };
    Vec2d  center { Vec2d::Zero() };
    double radius { 0. };
};

struct PathLeaf
{
    ExtrusionId       id;
    ExtrusionRole     role { erNone };
    double            width { 0. };
    std::vector<Vec2d> points;
};

struct PathChain
{
    size_t                layer_id { 0 };
    bool                  closed { false };
    std::vector<PathLeaf> leaves;
};

using PathPolyline = std::vector<Vec2d>;

std::vector<Contact> associate(const std::vector<Pin>       &pins,
                               const std::vector<PathChain> &chains,
                               double                        tolerance);

// Reproduce the geometry OrcaSlicer will emit for one finalized leaf path.
// Fitted arcs are tessellated with chord error no greater than tolerance.
std::vector<Vec2d> tessellate_path(const ExtrusionPath &path,
                                   bool                 use_fitted_arcs,
                                   double               tolerance);

// Inspect finalized model extrusion paths using the same simplified and
// arc-tessellated representation consumed by associate().
std::vector<PathChain> extract_finalized_path_chains(const PrintObject &object);

// Clip a chain-arclength interval to its exact tessellated geometry. Closed
// chains accept intervals outside [0, chain length] and wrap across the seam.
// Separate polylines are returned only if the stored chain is discontinuous.
std::vector<PathPolyline> clip_path_chain_interval(const PathChain    &chain,
                                                   const PathInterval &interval);

// Inspect finalized model extrusion paths without changing them.
std::vector<Contact> build(const PrintObject &object);

const char *extrusion_source_name(ExtrusionSource source);
const char *extrusion_role_name(ExtrusionRole role);
const char *anchor_type_name(AnchorType type);

} // namespace OrganicSupport
} // namespace Slic3r

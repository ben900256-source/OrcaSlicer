#include <catch2/catch_all.hpp>

#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MinAreaBoundingBox.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"
#include "nlohmann/json.hpp"

#include "test_helpers.hpp" // get access to init_print, etc
#include "test_utils.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

using namespace Slic3r::Test;
using namespace Slic3r;

namespace {

DynamicPrintConfig organic_contact_config(double spacing = 2., double density = 20.)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    // init_print arranges these fixtures around the origin. Keep the complete
    // model and its Organic roots inside the finite bed used for clipping.
    config.set_key_value("printable_area", new ConfigOptionPoints{
        Vec2d(-200., -200.), Vec2d(200., -200.), Vec2d(200., 200.), Vec2d(-200., 200.) });
    config.set_deserialize_strict({
        { "enable_support",                        1 },
        { "support_type",                         "tree(auto)" },
        { "support_style",                        "organic" },
        { "support_interface_top_layers",         0 },
        { "support_remove_small_overhang",         0 },
        { "bridge_no_support",                     0 },
        { "support_line_width",                    0.4 },
        { "tree_support_tip_diameter",             0.4 },
        { "tree_support_branch_diameter_organic",  2.0 },
        { "tree_support_branch_diameter_angle",    5.0 },
        { "tree_support_branch_distance_organic",  spacing },
        { "tree_support_top_rate",                 density },
    });
    return config;
}

using ContactSignature = std::tuple<coord_t, coord_t, coordf_t, coordf_t, coord_t, size_t>;
using AssociationSignature = std::tuple<size_t, std::string, int, double, double, std::optional<double>, std::optional<double>>;

std::vector<ContactSignature> contact_signatures(const PrintObject &object)
{
    std::vector<ContactSignature> out;
    out.reserve(object.support_contacts().size());
    for (const SupportContact &contact : object.support_contacts())
        out.emplace_back(contact.position.x(), contact.position.y(), contact.support_tip_z,
                         contact.model_contact_z, contact.nominal_radius, contact.object_layer_id);
    return out;
}

std::vector<AssociationSignature> association_signatures(const std::vector<OrganicSupport::Contact> &contacts)
{
    std::vector<AssociationSignature> out;
    for (const OrganicSupport::Contact &contact : contacts)
        for (const OrganicSupport::ExtrusionAssociation &association : contact.associations)
            out.emplace_back(contact.contact_id, association.extrusion_id.to_string(), int(association.role),
                             association.width, association.centerline_distance,
                             association.path_arclength_interval ?
                                 std::optional<double>(association.path_arclength_interval->begin) : std::nullopt,
                             association.path_arclength_interval ?
                                 std::optional<double>(association.path_arclength_interval->end) : std::nullopt);
    return out;
}

std::vector<AssociationSignature> association_signatures(const PrintObject &object)
{
    return association_signatures(object.support_associations());
}

void append_path_snapshot(const ExtrusionEntity &entity, nlohmann::json &paths)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            append_path_snapshot(*child, paths);
        return;
    }
    auto append = [&paths](const ExtrusionPath &path) {
        nlohmann::json points = nlohmann::json::array();
        for (const Point3 &point : path.polyline.points)
            points.push_back({ point.x(), point.y(), point.z() });
        nlohmann::json fittings = nlohmann::json::array();
        for (const PathFittingData &fitting : path.polyline.fitting_result)
            fittings.push_back({
                { "start", fitting.start_point_index },
                { "end", fitting.end_point_index },
                { "type", int(fitting.path_type) },
                { "center", { fitting.arc_data.center.x(), fitting.arc_data.center.y() } },
                { "radius", fitting.arc_data.radius },
                { "angle", fitting.arc_data.angle_radians }
            });
        paths.push_back({
            { "role", int(path.role()) }, { "width", path.width }, { "height", path.height },
            { "mm3_per_mm", path.mm3_per_mm }, { "no_extrusion", path.is_force_no_extrusion() },
            { "points", std::move(points) }, { "fittings", std::move(fittings) }
        });
    };
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        append(*path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const ExtrusionPath &path : loop->paths) append(path);
    else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const ExtrusionPath &path : multipath->paths) append(path);
}

std::string final_model_path_snapshot(const PrintObject &object)
{
    nlohmann::json paths = nlohmann::json::array();
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions()) {
            append_path_snapshot(region->perimeters, paths);
            append_path_snapshot(region->fills, paths);
        }
    return paths.dump();
}

const ExtrusionPath *resolve_association_path(const PrintObject &object,
                                              const OrganicSupport::ExtrusionId &id)
{
    const Layer *layer = nullptr;
    for (const Layer *candidate : object.layers())
        if (candidate->id() == id.layer_id) {
            layer = candidate;
            break;
        }
    if (layer == nullptr || id.region_id >= layer->region_count())
        return nullptr;
    const LayerRegion *region = layer->get_region(int(id.region_id));
    const ExtrusionEntity *entity = id.source == OrganicSupport::ExtrusionSource::Perimeter ?
        static_cast<const ExtrusionEntity *>(&region->perimeters) :
        static_cast<const ExtrusionEntity *>(&region->fills);
    for (size_t entity_idx : id.entity_indices) {
        const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity);
        if (collection == nullptr || entity_idx >= collection->entities.size())
            return nullptr;
        entity = collection->entities[entity_idx];
    }
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
        return id.leaf_path_index == 0 ? path : nullptr;
    if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
        return id.leaf_path_index < loop->paths.size() ? &loop->paths[id.leaf_path_index] : nullptr;
    if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(entity))
        return id.leaf_path_index < multipath->paths.size() ? &multipath->paths[id.leaf_path_index] : nullptr;
    return nullptr;
}

void check_gui_association_intervals_resolve(const PrintObject &object)
{
    const std::vector<OrganicSupport::PathChain> chains =
        OrganicSupport::extract_finalized_path_chains(object);
    auto find_chain = [&chains](const OrganicSupport::ExtrusionId &id) {
        return std::find_if(chains.begin(), chains.end(), [&id](const OrganicSupport::PathChain &chain) {
            return std::any_of(chain.leaves.begin(), chain.leaves.end(), [&id](const OrganicSupport::PathLeaf &leaf) {
                return leaf.id == id;
            });
        });
    };

    for (const OrganicSupport::Contact &contact : object.support_associations())
        for (const OrganicSupport::ExtrusionAssociation &association : contact.associations) {
            const auto chain_it = find_chain(association.extrusion_id);
            REQUIRE(chain_it != chains.end());
            if (!association.path_arclength_interval) {
                CHECK_FALSE(association.planned_material_unsupported_span.has_value());
                continue;
            }

            CHECK_FALSE(OrganicSupport::clip_path_chain_interval(
                *chain_it, *association.path_arclength_interval).empty());
            if (!association.planned_material_unsupported_span)
                continue;

            const OrganicSupport::PathInterval &nominal = *association.path_arclength_interval;
            const auto check_direction = [&](const OrganicSupport::DirectionalUnsupportedSpan &direction,
                                             bool forward) {
                const std::optional<double> distance = direction.span ? direction.span : direction.unanchored_distance;
                REQUIRE(distance.has_value());
                const OrganicSupport::PathInterval span_interval = forward ?
                    OrganicSupport::PathInterval{ nominal.end, nominal.end + *distance } :
                    OrganicSupport::PathInterval{ nominal.begin - *distance, nominal.begin };
                CHECK_FALSE(OrganicSupport::clip_path_chain_interval(*chain_it, span_interval).empty());
                for (const OrganicSupport::PlannedMaterialAnchor &anchor : direction.anchors)
                    CHECK_FALSE(OrganicSupport::clip_path_chain_interval(*chain_it, anchor.interval).empty());
            };
            check_direction(association.planned_material_unsupported_span->forward, true);
            check_direction(association.planned_material_unsupported_span->backward, false);
        }
}

double median_nearest_neighbour_distance(const std::vector<SupportContact> &contacts)
{
    std::vector<double> nearest;
    nearest.reserve(contacts.size());
    for (size_t i = 0; i < contacts.size(); ++i) {
        double best = std::numeric_limits<double>::max();
        for (size_t j = 0; j < contacts.size(); ++j) {
            if (i == j || contacts[i].object_layer_id != contacts[j].object_layer_id)
                continue;
            const double dx = unscale<double>(contacts[i].position.x() - contacts[j].position.x());
            const double dy = unscale<double>(contacts[i].position.y() - contacts[j].position.y());
            best = std::min(best, std::hypot(dx, dy));
        }
        if (best < std::numeric_limits<double>::max())
            nearest.push_back(best);
    }
    REQUIRE_FALSE(nearest.empty());
    std::sort(nearest.begin(), nearest.end());
    const size_t middle = nearest.size() / 2;
    return nearest.size() % 2 == 0 ? 0.5 * (nearest[middle - 1] + nearest[middle]) : nearest[middle];
}

struct TerminalFootprintStats {
    size_t count;
    double median_aspect_ratio;
    double max_aspect_ratio;
};

TerminalFootprintStats unobstructed_terminal_footprint_stats(const PrintObject &object)
{
    std::vector<double> ratios;
    for (const SupportContact &contact : object.support_contacts()) {
        const SupportLayer *contact_layer = nullptr;
        for (const SupportLayer *layer : object.support_layers())
            if (std::abs(layer->print_z - contact.support_tip_z) < EPSILON) {
                contact_layer = layer;
                break;
            }
        if (contact_layer == nullptr)
            continue;

        for (const ExPolygon &island : contact_layer->support_islands) {
            if (! island.contains(contact.position))
                continue;

            // Collision-clipped or branch-merged islands are not unobstructed
            // terminal footprints. A discretized circle is slightly under its
            // analytic area, so keep a symmetric tolerance around the nominal tip.
            const double nominal_area = PI * sqr(double(contact.nominal_radius));
            const double area_ratio   = std::abs(island.area()) / nominal_area;
            if (area_ratio < 0.9 || area_ratio > 1.1)
                break;

            // A terminal island shared by multiple tips has already merged and is
            // not an unobstructed footprint of one planned contact.
            const size_t contacts_in_island = std::count_if(
                object.support_contacts().begin(), object.support_contacts().end(),
                [&](const SupportContact &candidate) {
                    return std::abs(candidate.support_tip_z - contact.support_tip_z) < EPSILON &&
                           island.contains(candidate.position);
                });
            if (contacts_in_island != 1)
                break;

            const MinAreaBoundigBox box(island);
            const double short_side = double(std::min(box.width(), box.height()));
            const double long_side  = double(std::max(box.width(), box.height()));
            if (short_side > 0.)
                ratios.push_back(long_side / short_side);
            break;
        }
    }

    if (ratios.empty())
        return { 0, 0., 0. };
    std::sort(ratios.begin(), ratios.end());
    const size_t middle = ratios.size() / 2;
    const double median = ratios.size() % 2 == 0 ? 0.5 * (ratios[middle - 1] + ratios[middle]) : ratios[middle];
    return { ratios.size(), median, ratios.back() };
}

double support_volume(const PrintObject &object)
{
    double volume = 0.;
    for (const SupportLayer *layer : object.support_layers())
        volume += layer->support_fills.total_volume();
    return volume;
}

double support_path_length(const PrintObject &object)
{
    double length = 0.;
    for (const SupportLayer *layer : object.support_layers())
        for (const ExtrusionEntity *entity : layer->support_fills.flatten().entities)
            if (! entity->is_collection())
                length += entity->length();
    return length;
}

bool round_terminal_layers_form_resin_style_runup(const PrintObject &object)
{
    static constexpr double minimum_runup_height_mm = 0.8;
    const ConstSupportLayerPtrsAdaptor layers = object.support_layers();
    size_t checked = 0;
    size_t tapered_checked = 0;
    size_t supported_bases_checked = 0;
    for (const SupportContact &contact : object.support_contacts()) {
        size_t layer_idx = 0;
        while (layer_idx < layers.size() && std::abs(layers[layer_idx]->print_z - contact.support_tip_z) >= EPSILON)
            ++ layer_idx;
        if (layer_idx == layers.size())
            return false;

        size_t runup_base_depth = 1;
        while (runup_base_depth < layer_idx &&
               contact.support_tip_z - layers[layer_idx - runup_base_depth]->print_z <
                   minimum_runup_height_mm - EPSILON)
            ++ runup_base_depth;
        if (runup_base_depth >= layer_idx)
            continue;

        const ExPolygon *upper = nullptr;
        for (const ExPolygon &island : layers[layer_idx]->support_islands)
            if (island.contains(contact.position)) {
                upper = &island;
                break;
            }
        if (upper == nullptr)
            return false;

        const double nominal_area = PI * sqr(double(contact.nominal_radius));
        const double area_ratio   = std::abs(upper->area()) / nominal_area;
        if (area_ratio < 0.9 || area_ratio > 1.1)
            continue;
        const double pin_center_offset = unscale<double>(
            (upper->contour.centroid() - contact.position).cast<double>().norm());
        if (pin_center_offset > 0.01)
            return false;
        ++ checked;

        bool upper_runup_is_centered = true;
        size_t isolated_taper_layers = 0;
        double previous_area = std::abs(upper->area());
        double maximum_isolated_radius = contact.nominal_radius;
        bool runup_base_is_supported = false;
        // A long sequence of concentric, progressively larger elliptical slices
        // leads from the centered pin into the curved Organic approach below.
        for (size_t depth = 1; depth <= runup_base_depth + 1; ++ depth) {
            const ExPolygon *overlapping = nullptr;
            for (const ExPolygon &island : layers[layer_idx - depth]->support_islands)
                if (! intersection_ex(ExPolygons{ *upper }, ExPolygons{ island }).empty()) {
                    overlapping = &island;
                    break;
                }
            if (overlapping == nullptr)
                return false;
            // The start of the runup must carry the next terminal layer.
            if (depth == runup_base_depth) {
                const double runup_base_area = std::abs(upper->area());
                double supported_area = 0.;
                for (const ExPolygon &overlap :
                     intersection_ex(ExPolygons{ *upper }, ExPolygons{ *overlapping }))
                    supported_area += std::abs(overlap.area());
                const double support_ratio = runup_base_area > 0. ? supported_area / runup_base_area : 0.;
                runup_base_is_supported = support_ratio >= 0.98;
            }
            if (depth <= runup_base_depth) {
                const size_t contacts_in_island = std::count_if(
                    object.support_contacts().begin(), object.support_contacts().end(),
                    [&](const SupportContact &candidate) {
                        return std::abs(candidate.support_tip_z - contact.support_tip_z) < EPSILON &&
                               overlapping->contains(candidate.position);
                    });
                const MinAreaBoundigBox box(*overlapping);
                const double short_side = double(std::min(box.width(), box.height()));
                if (contacts_in_island == 1 && short_side > 0.) {
                    const double center_offset = unscale<double>(
                        (overlapping->contour.centroid() - contact.position).cast<double>().norm());
                    if (depth <= 4 && center_offset > 0.03)
                        upper_runup_is_centered = false;
                    if (center_offset <= 0.03) {
                        const double slice_area = std::abs(overlapping->area());
                        if (slice_area < 0.95 * previous_area)
                            return false;
                        ++ isolated_taper_layers;
                        previous_area = slice_area;
                        maximum_isolated_radius = std::max(maximum_isolated_radius,
                            std::sqrt(slice_area / PI));
                    }
                }
            }
            upper = overlapping;
        }
        const double runup_height = contact.support_tip_z -
            layers[layer_idx - runup_base_depth]->print_z;
        if (upper_runup_is_centered && runup_height >= minimum_runup_height_mm - EPSILON &&
            runup_base_depth >= 10 && isolated_taper_layers >= 4 &&
            maximum_isolated_radius >= 1.5 * contact.nominal_radius)
            ++ tapered_checked;
        if (runup_base_is_supported)
            ++ supported_bases_checked;
    }
    return checked >= 6 && tapered_checked >= 6 && supported_bases_checked >= 6;
}

struct TerminalRunupStats {
    size_t count{ 0 };
    double median_angle_degrees{ 0. };
    double max_angle_degrees{ 0. };
    double median_x_slope{ 0. };
    double minimum_support_ratio{ 1. };
};

struct TerminalRunupSample {
    Vec2d  contact_center;
    Vec2d  slope;
    double angle_degrees;
    double minimum_support_ratio;
};

std::vector<TerminalRunupSample> terminal_runup_samples(
    const PrintObject &object, double sample_depth_mm = 0.15,
    double *overall_minimum_support_ratio = nullptr)
{
    std::vector<TerminalRunupSample> samples;
    double overall_minimum = 1.;
    const ConstSupportLayerPtrsAdaptor layers = object.support_layers();
    for (const SupportContact &contact : object.support_contacts()) {
        size_t terminal_layer_idx = 0;
        while (terminal_layer_idx < layers.size() &&
               std::abs(layers[terminal_layer_idx]->print_z - contact.support_tip_z) >= EPSILON)
            ++ terminal_layer_idx;
        REQUIRE(terminal_layer_idx < layers.size());

        const ExPolygon *upper = nullptr;
        bool contact_found = false;
        for (const ExPolygon &island : layers[terminal_layer_idx]->support_islands)
            if (island.contains(contact.position)) {
                contact_found = true;
                const double nominal_area = PI * sqr(double(contact.nominal_radius));
                const double area_ratio = std::abs(island.area()) / nominal_area;
                if (area_ratio >= 0.9 && area_ratio <= 1.1)
                    upper = &island;
                break;
            }
        REQUIRE(contact_found);
        if (upper == nullptr)
            continue;

        const Point terminal_center = upper->contour.centroid();
        const ExPolygon *sample = upper;
        bool isolated_runup = true;
        double sampled_depth = 0.;
        double minimum_support_ratio = 1.;
        for (size_t depth = 1; depth <= terminal_layer_idx; ++ depth) {
            const SupportLayer *lower_layer = layers[terminal_layer_idx - depth];
            const ExPolygon *lower = nullptr;
            double largest_overlap_area = 0.;
            for (const ExPolygon &island : lower_layer->support_islands) {
                double overlap_area = 0.;
                for (const ExPolygon &overlap : intersection_ex(ExPolygons{ *sample }, ExPolygons{ island }))
                    overlap_area += std::abs(overlap.area());
                if (overlap_area > largest_overlap_area) {
                    largest_overlap_area = overlap_area;
                    lower = &island;
                }
            }
            REQUIRE(lower != nullptr);
            // A nearby junction can merge into the lower footprint. Its
            // centroid does not measure the terminal axis; connectivity is
            // checked separately for every contact, including these joins.
            if (lower->area() > 4. * upper->area())
                isolated_runup = false;
            const double upper_area = std::abs(sample->area());
            if (upper_area > 0.) {
                const double support_ratio = largest_overlap_area / upper_area;
                minimum_support_ratio = std::min(minimum_support_ratio, support_ratio);
                overall_minimum = std::min(overall_minimum, support_ratio);
            }
            sample = lower;
            sampled_depth = contact.support_tip_z - lower_layer->print_z;
            if (sampled_depth + EPSILON >= sample_depth_mm)
                break;
        }
        if (!isolated_runup || sampled_depth + EPSILON < sample_depth_mm)
            continue;

        const Point upward_delta_scaled = terminal_center - sample->contour.centroid();
        const Vec2d upward_delta = unscaled<double>(upward_delta_scaled);
        const Vec2d slope = upward_delta / sampled_depth;
        samples.push_back({ unscaled<double>(contact.position), slope,
                            std::atan(slope.norm()) * 180. / PI,
                            minimum_support_ratio });
    }
    if (overall_minimum_support_ratio != nullptr)
        *overall_minimum_support_ratio = overall_minimum;
    return samples;
}

TerminalRunupStats terminal_runup_stats(const PrintObject &object, double sample_depth_mm = 0.15)
{
    double minimum_support_ratio = 1.;
    const std::vector<TerminalRunupSample> samples =
        terminal_runup_samples(object, sample_depth_mm, &minimum_support_ratio);
    if (samples.empty())
        return {};
    std::vector<double> angles;
    std::vector<double> x_slopes;
    angles.reserve(samples.size());
    x_slopes.reserve(samples.size());
    for (const TerminalRunupSample &sample : samples) {
        angles.push_back(sample.angle_degrees);
        x_slopes.push_back(sample.slope.x());
    }
    std::sort(angles.begin(), angles.end());
    std::sort(x_slopes.begin(), x_slopes.end());
    const auto median = [](const std::vector<double> &values) {
        const size_t middle = values.size() / 2;
        return values.size() % 2 == 0 ? 0.5 * (values[middle - 1] + values[middle]) : values[middle];
    };
    return { angles.size(), median(angles), angles.back(), median(x_slopes), minimum_support_ratio };
}

struct NormalTestPanel {
    Vec2d  center;
    double x_rotation_degrees;
    double y_rotation_degrees;

    Vec2d expected_slope() const
    {
        return Vec2d(std::tan(y_rotation_degrees * PI / 180.),
                     -std::tan(x_rotation_degrees * PI / 180.));
    }
};

const std::array<NormalTestPanel, 5> &multi_normal_test_panels()
{
    static const std::array<NormalTestPanel, 5> panels{
        NormalTestPanel{ Vec2d(  0.,   0.),   0.,   0. },
        NormalTestPanel{ Vec2d( 28.,   0.),   0.,  20. },
        NormalTestPanel{ Vec2d(-28.,   0.),   0., -20. },
        NormalTestPanel{ Vec2d(  0.,  28.),  18.,   0. },
        NormalTestPanel{ Vec2d(  0., -28.), -18.,   0. },
    };
    return panels;
}

static constexpr double multi_normal_panel_size_mm = 18.;

TriangleMesh multi_normal_underside_coupon()
{
    TriangleMesh coupon;
    for (const NormalTestPanel &panel : multi_normal_test_panels()) {
        TriangleMesh stem = make_cube(2., 2., 12.);
        stem.translate(Vec3f(float(panel.center.x() - 1.),
                             float(panel.center.y() - 1.), 0.f));
        coupon.merge(stem);

        TriangleMesh roof = make_cube(
            multi_normal_panel_size_mm, multi_normal_panel_size_mm, 1.);
        const float half_panel_size = float(0.5 * multi_normal_panel_size_mm);
        roof.translate(Vec3f(-half_panel_size, -half_panel_size, -0.5f));
        roof.rotate_x(float(panel.x_rotation_degrees * PI / 180.));
        roof.rotate_y(float(panel.y_rotation_degrees * PI / 180.));
        roof.translate(Vec3f(float(panel.center.x()), float(panel.center.y()), 15.f));
        coupon.merge(roof);
    }
    return coupon;
}

TriangleMesh sloped_underside_coupon(double angle_degrees, bool invert_underside = false)
{
    TriangleMesh stem = make_cube(2., 2., 15.);
    stem.translate(Vec3f(-1.f, -1.f, 0.f));
    TriangleMesh roof = make_cube(12., 12., 1.);
    roof.translate(Vec3f(-6.f, -6.f, -0.5f));
    if (invert_underside) {
        for (Vec3i32 &face : roof.its.indices) {
            const Vec3f &v0 = roof.its.vertices[size_t(face[0])];
            const Vec3f &v1 = roof.its.vertices[size_t(face[1])];
            const Vec3f &v2 = roof.its.vertices[size_t(face[2])];
            if ((v1 - v0).cross(v2 - v0).z() < -0.9f)
                std::swap(face[0], face[1]);
        }
    }
    roof.rotate_y(float(angle_degrees * PI / 180.));
    roof.translate(Vec3f(0.f, 0.f, 15.f));
    stem.merge(roof);
    return stem;
}

// Check emitted material as well as islands: a polygon can survive while its
// inset perimeter vanishes, or a steep taper can land inside a hollow trunk.
void require_connected_contacts(const PrintObject &object)
{
    const auto layers = object.support_layers();
    REQUIRE_FALSE(object.support_contacts().empty());
    std::vector<Polygons> extrusions;
    extrusions.reserve(layers.size());
    for (const SupportLayer *layer : layers)
        extrusions.push_back(union_(layer->support_fills.polygons_covered_by_width()));
    for (const SupportContact &contact : object.support_contacts()) {
        CAPTURE(contact.position, contact.support_tip_z);
        auto terminal = std::find_if(layers.begin(), layers.end(), [&](const SupportLayer *layer) {
            return std::abs(layer->print_z - contact.support_tip_z) < EPSILON;
        });
        REQUIRE(terminal != layers.end());
        ExPolygons connected;
        for (const ExPolygon &island : (*terminal)->support_islands)
            if (island.contains(contact.position))
                connected.push_back(island);
        REQUIRE_FALSE(connected.empty());
        // Follow the entire connection, including extended approaches and merged
        // junctions. No missing layer/contact may silently drop out of the test.
        for (size_t idx = size_t(terminal - layers.begin()); idx > 0; -- idx) {
            const Polygons printed = intersection_clipped(to_polygons(connected), extrusions[idx]);
            REQUIRE_FALSE(printed.empty());
            ExPolygons lower;
            for (const ExPolygon &island : layers[idx - 1]->support_islands)
                if (!intersection_ex(connected, ExPolygons{ island }).empty())
                    lower.push_back(island);
            if (lower.empty()) {
                // Organic roots may terminate on the fixture's central stem.
                const auto model_layer = std::find_if(object.layers().begin(), object.layers().end(),
                    [&](const Layer *layer) {
                        return std::abs(layer->print_z - layers[idx - 1]->print_z) < EPSILON;
                    });
                REQUIRE(model_layer != object.layers().end());
                REQUIRE_FALSE(intersection_ex(connected, (*model_layer)->lslices).empty());
                break;
            }
            REQUIRE_FALSE(intersection_clipped(printed, extrusions[idx - 1]).empty());
            // Original Organic fallbacks retain their wider terminal; the
            // direct 98% footprint rule applies to the round Pin candidates.
            const double tip_area = std::abs(area(to_polygons(connected)));
            const double nominal_area = PI * sqr(double(contact.nominal_radius));
            if (std::abs(contact.support_tip_z - layers[idx]->print_z) < EPSILON &&
                tip_area >= 0.9 * nominal_area && tip_area <= 1.1 * nominal_area) {
                for (const ExPolygon &component : connected) {
                    double overlap = 0.;
                    for (const ExPolygon &part : intersection_ex(ExPolygons{ component }, lower))
                        overlap += part.area();
                    CHECK(overlap / component.area() >= 0.98 - 1e-5);
                }
            }
            connected = std::move(lower);
        }
    }
}

} // namespace

TEST_CASE("Three raft layers are created", "[SupportMaterial]")
{
	Slic3r::Print print;
	Slic3r::Test::init_and_process_print({ cube(20) }, print, {
        { "enable_support", 1 },
        { "raft_layers",    3 }
		});
    REQUIRE(print.objects().front()->support_layers().size() == 3);
}

TEST_CASE("Enforced support layers are generated", "[SupportMaterial]")
{
    // enforce_support_layers forces support on the first N layers even with support off.
    Slic3r::Print baseline;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, baseline, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    REQUIRE(baseline.objects().front()->support_layers().empty());

    Slic3r::Print enforced;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, enforced, {
        { "enable_support",         0 },
        { "enforce_support_layers", 100 }
    });
    REQUIRE(enforced.objects().front()->support_layers().size() > 0);
}

SCENARIO("Support layer Z honors contact distance", "[SupportMaterial]")
{
    // Box h = 20mm, hole bottom at 5mm, hole height 10mm (top edge at 15mm).
    TriangleMesh mesh = Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole);
    mesh.rotate_x(float(M_PI / 2));

	auto check = [](Slic3r::Print &print, bool &first_support_layer_height_ok, bool &layer_height_minimum_ok, bool &layer_height_maximum_ok)
	{
        ConstSupportLayerPtrsAdaptor support_layers = print.objects().front()->support_layers();

		first_support_layer_height_ok = support_layers.front()->print_z == print.config().initial_layer_print_height.value;

		layer_height_minimum_ok = true;
		layer_height_maximum_ok = true;
		double min_layer_height = print.config().min_layer_height.values.front();
		double max_layer_height = print.config().nozzle_diameter.values.front();
		if (print.config().max_layer_height.values.front() > EPSILON)
			max_layer_height = std::min(max_layer_height, print.config().max_layer_height.values.front());
		for (size_t i = 1; i < support_layers.size(); ++ i) {
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z < min_layer_height - EPSILON)
				layer_height_minimum_ok = false;
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z > max_layer_height + EPSILON)
				layer_height_maximum_ok = false;
		}
	};

    GIVEN("A print object having one modelObject") {
        WHEN("Layer height = 0.2 and first layer height = 0.4") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.4 },
                { "dont_support_bridges",       false },
			});
			bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
        WHEN("Layer height = 0.2 and first layer height = 0.3") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.3 },
                { "dont_support_bridges",       false },
            });
            bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
    }
}

// extrude_support once held a `static` lambda capturing `this`, so a second export in the
// same process dereferenced a returned stack frame (ASan: stack-use-after-return).
TEST_CASE("Support G-code emission survives a second slice in the same process", "[SupportMaterial][Regression]")
{
    const std::string first = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(first, "support").empty());

    const std::string second = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(second, "support").empty());
}

TEST_CASE("Organic supports expose deterministic realized contacts", "[SupportMaterial][OrganicContacts]")
{
    const DynamicPrintConfig config = organic_contact_config();

    Print first;
    Model first_model;
    init_print({ TestMesh::overhang }, first, first_model, config);
    first.process();

    const PrintObject &first_object = *first.objects().front();
    const auto first_contacts = contact_signatures(first_object);
    REQUIRE_FALSE(first_contacts.empty());
    CHECK(first_object.support_associations().empty());
    CHECK(std::is_sorted(first_contacts.begin(), first_contacts.end()));
    for (const SupportContact &contact : first_object.support_contacts()) {
        CHECK(contact.nominal_radius == scaled<coord_t>(0.2));
        CHECK(contact.support_tip_z < contact.model_contact_z);
        CHECK(contact.object_layer_id < first_object.layer_count());
    }
    const TreeSupport3D::TreeSupportMeshGroupSettings mesh_settings(first_object);
    const TreeSupport3D::TreeSupportSettings tree_settings(mesh_settings, first_object.slicing_parameters());
    coord_t                            previous_radius = 0;
    TreeSupport3D::SupportElementState radius_state{};
    for (size_t distance_to_top = 0; distance_to_top <= tree_settings.tip_layers; ++distance_to_top) {
        radius_state.distance_to_top          = uint32_t(distance_to_top);
        radius_state.effective_radius_height = uint32_t(distance_to_top);
        const coord_t radius = TreeSupport3D::support_element_radius(tree_settings, radius_state);
        CHECK(radius >= previous_radius);
        previous_radius = radius;
    }
    CHECK(previous_radius == tree_settings.branch_radius);
    CHECK(tree_settings.tip_layers * unscale<double>(tree_settings.layer_height) <= 1.5);
    CHECK(first.validate_support_contact_export().empty());

    const std::string support_gcode = gcode(first);
    CHECK_FALSE(support_gcode.empty());
    CHECK_FALSE(layers_with_role(support_gcode, "support").empty());

    Print second;
    Model second_model;
    init_print({ TestMesh::overhang }, second, second_model, config);
    second.process();
    CHECK(contact_signatures(*second.objects().front()) == first_contacts);
}

TEST_CASE("Organic contact snapshots store resolved branch terminals", "[SupportMaterial][OrganicContacts]")
{
    DynamicPrintConfig config = organic_contact_config();
    config.set_deserialize_strict({ { "tree_support_round_tip", true } });
    Print print;
    Model model;
    init_print({ TestMesh::overhang }, print, model, config);
    print.process();

    PrintObject &object = *print.get_object(0);
    const TreeSupport3D::TreeSupportMeshGroupSettings mesh_settings(object);
    const TreeSupport3D::TreeSupportSettings tree_settings(mesh_settings, object.slicing_parameters());
    const TreeSupport3D::LayerIndex target_height = 5;
    REQUIRE(size_t(target_height) + tree_settings.z_distance_top_layers + 1 <
            object.layer_count() + tree_settings.raft_layers.size());

    TreeSupport3D::SupportElementState endpoint{};
    endpoint.target_height           = target_height;
    endpoint.target_position         = Point(scale_(10.), scale_(10.));
    endpoint.next_position           = endpoint.target_position;
    endpoint.layer_idx               = target_height;
    endpoint.effective_radius_height = 0;
    endpoint.distance_to_top         = 0;
    endpoint.result_on_layer         = endpoint.target_position;
    endpoint.increased_to_model_radius = 0;
    endpoint.elephant_foot_increases  = 0.;
    endpoint.dont_move_until          = 0;

    TreeSupport3D::SupportElements elements;
    elements.emplace_back(endpoint, Polygons{}); // realized and connected
    elements.emplace_back(endpoint, Polygons{}); // disconnected
    elements.emplace_back(endpoint, Polygons{}); // pruned
    elements.back().state.deleted = true;
    elements.emplace_back(endpoint, Polygons{}); // not a terminal node
    elements.back().parents.push_back(0);

    const size_t object_layer_idx = size_t(target_height) + tree_settings.z_distance_top_layers + 1 -
                                    tree_settings.raft_layers.size();
    const Layer *object_layer = object.get_layer(int(object_layer_idx));
    TreeSupport3D::TerminalContactFrames terminal_frames;
    terminal_frames.emplace(&elements[0], TreeSupport3D::TerminalContactFrame{
        endpoint.target_position,
        TreeSupport3D::layer_z(object.slicing_parameters(), tree_settings, size_t(target_height)),
        object_layer->bottom_z(),
        TreeSupport3D::support_element_radius(tree_settings, endpoint),
        object_layer->id(),
        object_layer_idx,
        Vec3d::UnitZ(),
        false,
        false
    });
    TreeSupport snapshotter(object, object.slicing_parameters());
    snapshotter.store_organic_support_contacts(terminal_frames);

    REQUIRE(object.support_contacts().size() == 1);
    CHECK(object.support_contacts().front().position == endpoint.target_position);
    CHECK(object.support_associations().empty());
    const std::vector<OrganicSupport::Contact> rebuilt = OrganicSupport::build(object);
    REQUIRE(rebuilt.size() == 1);
    CHECK(rebuilt.front().contact_id == 0);
}

TEST_CASE("Organic contacts survive invalidation and slicedata cache round trips", "[SupportMaterial][OrganicContacts]")
{
    DynamicPrintConfig config = organic_contact_config();
    config.set_deserialize_strict({ { "tree_support_round_tip", true } });
    Print              print;
    Model              model;
    init_print({ TestMesh::overhang }, print, model, config);
    print.process();

    PrintObject &object = *print.get_object(0);
    const auto expected = contact_signatures(object);
    const auto expected_associations = association_signatures(object);
    REQUIRE_FALSE(expected.empty());
    REQUIRE_FALSE(expected_associations.empty());

    config.set_deserialize_strict({ { "support_threshold_angle", 31 } });
    print.apply(model, config);
    CHECK_FALSE(object.is_step_done(posSupportMaterial));
    CHECK(contact_signatures(object) == expected);
    CHECK(object.support_associations().empty());

    ScopedTemporaryDir temporary("support-contact-cache");
    const boost::filesystem::path cache = temporary.path() / "slicedata";
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);

    const ModelInstance *model_instance = object.instances().front().model_instance;
    const size_t identify_id = model_instance->loaded_id > 0 ? model_instance->loaded_id : model_instance->id().id;
    std::ifstream cache_stream((cache / ("obj_" + std::to_string(identify_id) + ".json")).string());
    const nlohmann::json cached_object = nlohmann::json::parse(cache_stream);
    REQUIRE(cached_object.is_object());
    REQUIRE(cached_object.at("support_contacts").is_array());
    for (const nlohmann::json &cached_contact : cached_object.at("support_contacts"))
        REQUIRE(cached_contact.is_object());

    REQUIRE(print.load_cached_data(cache.string()) == 0);
    CHECK(contact_signatures(object) == expected);
    CHECK(association_signatures(object) == expected_associations);
    check_gui_association_intervals_resolve(object);
}

TEST_CASE("Support contact reports apply instance transforms and summarize nominal area", "[SupportMaterial][OrganicContacts]")
{
    DynamicPrintConfig config = organic_contact_config();
    config.set_deserialize_strict({ { "tree_support_round_tip", true } });
    Print print;
    Model model;
    init_print({ TestMesh::overhang }, print, model, config);

    ModelObject &model_object = *model.objects.front();
    const Vec3d first_offset = model_object.instances.front()->get_offset();
    model_object.add_instance()->set_offset(first_offset + Vec3d(40., 0., 0.));
    print.apply(model, config);
    print.process();

    const size_t contacts_per_instance = print.objects().front()->support_contacts().size();
    REQUIRE(contacts_per_instance > 0);
    check_gui_association_intervals_resolve(*print.objects().front());

    ScopedTemporaryFile report_file(".support-contacts.json");
    print.export_support_contacts(report_file.string(), 3);
    std::ifstream report_stream(report_file.string());
    const nlohmann::json report = nlohmann::json::parse(report_stream);

    CHECK(report.at("schema_version") == 2);
    CHECK(report.at("plate") == 3);
    CHECK(report.at("coordinate_space") == "plate-local");
    CHECK(report.at("units").at("position") == "mm");
    CHECK(report.at("units").at("area") == "mm^2");
    CHECK(report.at("contact_count") == 2 * contacts_per_instance);
    REQUIRE(report.at("instances").size() == 2);
    CHECK(report.at("instances").at(0).at("contact_count") == contacts_per_instance);
    CHECK(report.at("instances").at(1).at("contact_count") == contacts_per_instance);

    const auto &first_contact  = report.at("instances").at(0).at("contacts").at(0);
    const auto &second_contact = report.at("instances").at(1).at("contacts").at(0);
    CHECK(first_contact.contains("support_tip_z"));
    CHECK(first_contact.contains("model_contact_z"));
    CHECK(first_contact.contains("nominal_radius"));
    CHECK(first_contact.contains("object_layer_id"));
    CHECK(first_contact.contains("contact_id"));
    CHECK(first_contact.contains("supported_layer"));
    CHECK(first_contact.contains("nominal_clearance"));
    CHECK(first_contact.contains("association_status"));
    CHECK(first_contact.contains("associations"));
    CHECK_THAT(second_contact.at("x").get<double>() - first_contact.at("x").get<double>(),
               Catch::Matchers::WithinAbs(40., 1e-9));
    CHECK_THAT(second_contact.at("y").get<double>(),
               Catch::Matchers::WithinAbs(first_contact.at("y").get<double>(), 1e-9));

    size_t associated_contact = 0;
    while (associated_contact < contacts_per_instance &&
           report.at("instances").at(0).at("contacts").at(associated_contact).at("associations").empty())
        ++associated_contact;
    REQUIRE(associated_contact < contacts_per_instance);
    const auto &first_association = report.at("instances").at(0).at("contacts").at(associated_contact)
        .at("associations").at(0);
    CHECK(first_association.at("extrusion_id").contains("layer_id"));
    CHECK(first_association.at("extrusion_id").contains("region_id"));
    CHECK(first_association.at("extrusion_id").contains("entity_indices"));
    CHECK(first_association.contains("extrusion_id_string"));
    CHECK(first_association.at("role") != "unsupported");
    CHECK(first_association.contains("width"));
    CHECK(first_association.contains("centerline_distance"));
    CHECK(first_association.contains("nominal_overlap_length"));
    CHECK(first_association.contains("path_arclength_interval"));
    CHECK(first_association.at("tangent").contains("ambiguous"));
    CHECK(first_association.contains("approximately_centered"));
    CHECK(first_association.contains("planned_material_unsupported_span"));

    const auto &first_closest = first_association.at("closest_point");
    const auto &second_closest = report.at("instances").at(1).at("contacts").at(associated_contact)
        .at("associations").at(0).at("closest_point");
    CHECK_THAT(second_closest.at("x").get<double>() - first_closest.at("x").get<double>(),
               Catch::Matchers::WithinAbs(40., 1e-9));
    CHECK_THAT(second_closest.at("y").get<double>(),
               Catch::Matchers::WithinAbs(first_closest.at("y").get<double>(), 1e-9));

    const double expected_area = 2. * contacts_per_instance * PI * 0.2 * 0.2;
    CHECK_THAT(report.at("total_nominal_circular_area").get<double>(),
               Catch::Matchers::WithinAbs(expected_area, 1e-9));
}

TEST_CASE("Round Organic contacts resolve finalized model extrusion paths", "[SupportMaterial][OrganicContacts]")
{
    DynamicPrintConfig config = organic_contact_config(3., 13.33);
    config.set_deserialize_strict({
        { "tree_support_round_tip", true },
        { "layer_height", 0.1 },
        { "initial_layer_print_height", 0.1 }
    });
    TriangleMesh coupon = load_model("support_contact_coupon.obj");
    coupon.scale(Vec3f(1.f, 1.f, 0.25f));
    Print print;
    Model model;
    init_print({ std::move(coupon) }, print, model, config);
    print.process();

    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_associations().size() == object.support_contacts().size());
    size_t association_count = 0;
    for (size_t contact_id = 0; contact_id < object.support_associations().size(); ++contact_id) {
        const OrganicSupport::Contact &contact = object.support_associations()[contact_id];
        CHECK(contact.contact_id == contact_id);
        CHECK(contact.supported_layer_id == object.support_contacts()[contact_id].object_layer_id);
        CHECK_THAT(contact.association_tolerance,
                   Catch::Matchers::WithinAbs(std::max(EPSILON, print.config().resolution.value), 1e-12));
        for (const OrganicSupport::ExtrusionAssociation &association : contact.associations) {
            ++association_count;
            const ExtrusionPath *path = resolve_association_path(object, association.extrusion_id);
            REQUIRE(path != nullptr);
            CHECK(path->role() == association.role);
            CHECK_THAT(path->width, Catch::Matchers::WithinAbs(association.width, 1e-9));
            CHECK(association.extrusion_id.layer_id == contact.supported_layer_id);
        }
    }
    REQUIRE(association_count > 0);
    check_gui_association_intervals_resolve(object);

    const std::string paths_before = final_model_path_snapshot(object);
    const std::vector<OrganicSupport::Contact> rebuilt = OrganicSupport::build(object);
    CHECK(final_model_path_snapshot(object) == paths_before);
    CHECK(association_signatures(rebuilt) == association_signatures(object));
    const std::vector<OrganicSupport::Contact> rebuilt_again = OrganicSupport::build(object);
    CHECK(association_signatures(rebuilt_again) == association_signatures(rebuilt));

    const std::string gcode_before = gcode(print);
    ScopedTemporaryFile report_file(".support-contacts.json");
    print.export_support_contacts(report_file.string(), 0);
    std::ifstream first_report_stream(report_file.string());
    std::ostringstream first_report;
    first_report << first_report_stream.rdbuf();
    ScopedTemporaryFile second_report_file(".support-contacts.json");
    print.export_support_contacts(second_report_file.string(), 0);
    std::ifstream second_report_stream(second_report_file.string());
    std::ostringstream second_report;
    second_report << second_report_stream.rdbuf();
    CHECK(second_report.str() == first_report.str());

    auto without_generated_timestamp = [](std::string value) {
        const size_t begin = value.find("; generated by OrcaSlicer ");
        if (begin != std::string::npos) {
            const size_t end = value.find('\n', begin);
            value.erase(begin, end == std::string::npos ? value.size() - begin : end - begin + 1);
        }
        return value;
    };
    CHECK(without_generated_timestamp(gcode(print)) == without_generated_timestamp(gcode_before));
}

TEST_CASE("Shared Organic objects rebuild identical associations", "[SupportMaterial][OrganicContacts]")
{
    DynamicPrintConfig config = organic_contact_config();
    config.set_deserialize_strict({ { "tree_support_round_tip", true } });
    Print print;
    Model model;
    init_print({ TestMesh::overhang }, print, model, config);
    ModelObject *original = model.objects.front();
    if (!original->config.has("extruder"))
        original->config.set_key_value("extruder", new ConfigOptionInt(1));
    ModelObject *duplicate = model.add_object(*original);
    for (ModelInstance *instance : duplicate->instances)
        instance->set_offset(instance->get_offset() + Vec3d(40., 0., 0.));
    print.apply(model, config);
    print.process();

    REQUIRE(print.objects().size() == 2);
    const PrintObject &first = *print.objects()[0];
    const PrintObject &second = *print.objects()[1];
    REQUIRE_FALSE(first.support_associations().empty());
    CHECK(association_signatures(second) == association_signatures(first));
    CHECK(second.get_shared_object() == &first);
    check_gui_association_intervals_resolve(first);
    check_gui_association_intervals_resolve(second);
}

TEST_CASE("Support contact export rejects non-discrete support configurations", "[SupportMaterial][OrganicContacts]")
{
    SECTION("top interfaces") {
        DynamicPrintConfig config = organic_contact_config();
        config.set_deserialize_strict({ { "support_interface_top_layers", 1 } });
        Print print;
        Model model;
        init_print({ TestMesh::overhang }, print, model, config);
        print.process();
        CHECK(print.validate_support_contact_export().find("top interface layers") != std::string::npos);
        CHECK(print.objects().front()->support_contacts().empty());
        CHECK_FALSE(layers_with_role(gcode(print), "support").empty());
    }

    SECTION("non-Organic tree style") {
        DynamicPrintConfig config = organic_contact_config();
        config.set_deserialize_strict({ { "support_style", "tree_slim" } });
        Print print;
        Model model;
        init_print({ TestMesh::overhang }, print, model, config);
        CHECK(print.validate_support_contact_export().find("only for Organic") != std::string::npos);
    }

    SECTION("support disabled") {
        Print print;
        Model model;
        init_print({ cube(20) }, print, model);
        print.process();
        REQUIRE(print.validate_support_contact_export().empty());

        ScopedTemporaryFile report_file(".support-contacts.json");
        print.export_support_contacts(report_file.string(), 1);
        std::ifstream report_stream(report_file.string());
        const nlohmann::json report = nlohmann::json::parse(report_stream);
        CHECK(report.at("contact_count") == 0);
        CHECK(report.at("instances").at(0).at("contacts").empty());
    }
}

TEST_CASE("Existing Organic controls realize sparse contact spacing", "[SupportMaterial][OrganicContacts]")
{
    struct Result {
        size_t count;
        double median_distance;
    };
    auto slice_coupon = [](double spacing, double density) {
        Print print;
        Model model;
        init_print({ load_model("support_contact_coupon.obj") }, print, model, organic_contact_config(spacing, density));
        print.process();
        const std::vector<SupportContact> contacts = print.objects().front()->support_contacts();
        REQUIRE(contacts.size() > 1);
        return Result{ contacts.size(), median_nearest_neighbour_distance(contacts) };
    };

    const Result spacing_2 = slice_coupon(2., 20.);
    const Result spacing_3 = slice_coupon(3., 13.33);
    const Result spacing_4 = slice_coupon(4., 10.);
    CAPTURE(spacing_2.count, spacing_2.median_distance,
            spacing_3.count, spacing_3.median_distance,
            spacing_4.count, spacing_4.median_distance);

    CHECK(spacing_2.count > spacing_3.count);
    CHECK(spacing_3.count > spacing_4.count);
    CHECK_THAT(spacing_2.median_distance, Catch::Matchers::WithinAbs(2., 0.4));
    CHECK_THAT(spacing_3.median_distance, Catch::Matchers::WithinAbs(3., 0.6));
    CHECK_THAT(spacing_4.median_distance, Catch::Matchers::WithinAbs(4., 0.8));
}

TEST_CASE("Round Organic Pin tips meet terminal footprint limits", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    struct SliceResult {
        TerminalFootprintStats footprints;
        double                 support_volume;
        double                 support_path_length;
        bool                   terminal_layers_form_resin_style_runup;
    };
    CHECK_FALSE(DynamicPrintConfig::full_print_config().opt_bool("tree_support_round_tip"));

    auto slice_coupon = [](double layer_height, double top_gap, bool round_tip) {
        DynamicPrintConfig config = organic_contact_config(3., 13.33);
        config.set_deserialize_strict({
            { "layer_height",                  layer_height },
            { "initial_layer_print_height",    layer_height },
            { "support_top_z_distance",        top_gap },
            { "tree_support_angle_slow",       25. },
            { "tree_support_round_tip",        round_tip },
        });

        TriangleMesh coupon = load_model("support_contact_coupon.obj");
        coupon.scale(Vec3f(1.f, 1.f, 0.25f));
        Print print;
        Model model;
        init_print({ std::move(coupon) }, print, model, config);
        print.process();
        const PrintObject &object = *print.objects().front();
        const TerminalFootprintStats footprints = unobstructed_terminal_footprint_stats(object);
        const double volume = support_volume(object);
        return SliceResult{ footprints, volume, support_path_length(object),
                            round_terminal_layers_form_resin_style_runup(object) };
    };

    for (const auto &[layer_height, top_gap] : std::array<std::pair<double, double>, 3>{
             std::pair{ 0.05, 0.0 }, std::pair{ 0.05, 0.05 }, std::pair{ 0.06, 0.06 } }) {
        DYNAMIC_SECTION("layer height " << layer_height << ", top gap " << top_gap) {
            const SliceResult baseline  = slice_coupon(layer_height, top_gap, false);
            const SliceResult candidate = slice_coupon(layer_height, top_gap, true);
            CAPTURE(baseline.footprints.count, baseline.footprints.median_aspect_ratio,
                    baseline.footprints.max_aspect_ratio, baseline.support_volume, baseline.support_path_length,
                    candidate.footprints.count, candidate.footprints.median_aspect_ratio,
                    candidate.footprints.max_aspect_ratio, candidate.support_volume, candidate.support_path_length,
                    candidate.terminal_layers_form_resin_style_runup);

            REQUIRE(candidate.footprints.count >= 6);
            CHECK(candidate.terminal_layers_form_resin_style_runup);
            CHECK(candidate.footprints.median_aspect_ratio <= 1.05);
            CHECK(candidate.footprints.max_aspect_ratio <= 1.10);
            CHECK(candidate.support_volume <= 1.15 * baseline.support_volume);
            // Support speed is unchanged, so extrusion path length is the direct
            // support-extrusion-time term without estimator startup noise.
            CHECK(candidate.support_path_length <= 1.15 * baseline.support_path_length);
        }
    }
}

TEST_CASE("Organic Pins taper continuously through the branch and tip", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    const double layer_height = GENERATE(0.05, 0.06);
    const double top_gap = GENERATE(0., 0.06);
    DynamicPrintConfig config = organic_contact_config(4., 10.);
    config.set_deserialize_strict({
        { "layer_height",               layer_height },
        { "initial_layer_print_height", layer_height },
        { "support_top_z_distance",     top_gap },
        { "tree_support_round_tip",     true },
    });
    Print print;
    Model model;
    init_print({ sloped_underside_coupon(0.) }, print, model, config);
    print.process();

    const PrintObject &object = *print.objects().front();
    const auto layers = object.support_layers();
    std::vector<std::vector<size_t>> descendants(layers.size());
    for (size_t idx = 0; idx < layers.size(); ++ idx) {
        descendants[idx].assign(layers[idx]->support_islands.size(), 0);
        for (size_t island = 0; island < descendants[idx].size(); ++ island)
            for (const SupportContact &contact : object.support_contacts())
                if (std::abs(contact.support_tip_z - layers[idx]->print_z) < EPSILON &&
                    layers[idx]->support_islands[island].contains(contact.position))
                    ++ descendants[idx][island];
    }
    // Count contacts by their actual connected paths. Testing only whether a
    // lower island contains the original contact XY misses displaced junctions.
    for (size_t idx = layers.size(); idx-- > 1;)
        for (size_t upper = 0; upper < descendants[idx].size(); ++ upper)
            if (descendants[idx][upper] > 0)
                for (size_t lower = 0; lower < descendants[idx - 1].size(); ++ lower)
                    if (!intersection_ex(ExPolygons{ layers[idx]->support_islands[upper] },
                                         ExPolygons{ layers[idx - 1]->support_islands[lower] }).empty())
                        descendants[idx - 1][lower] += descendants[idx][upper];
    size_t checked = 0;
    for (const SupportContact &contact : object.support_contacts()) {
        auto terminal = std::find_if(layers.begin(), layers.end(), [&](const SupportLayer *layer) {
            return std::abs(layer->print_z - contact.support_tip_z) < EPSILON;
        });
        REQUIRE(terminal != layers.end());
        const size_t terminal_idx = size_t(terminal - layers.begin());
        double previous_radius = unscale<double>(contact.nominal_radius);
        double previous_z = contact.support_tip_z;
        size_t approach_layers = 0;
        for (size_t idx = terminal_idx; idx-- > 0;) {
            const SupportLayer &layer = *layers[idx];
            const double depth = contact.support_tip_z - layer.print_z;
            if (depth > 2.4 + layer_height)
                break;
            const auto island = std::find_if(layer.support_islands.begin(), layer.support_islands.end(),
                [&](const ExPolygon &polygon) { return polygon.contains(contact.position); });
            if (island == layer.support_islands.end())
                break;
            // Merged branches do not describe the taper of a single Pin.
            const size_t contacts_in_island = descendants[idx][size_t(island - layer.support_islands.begin())];
            if (contacts_in_island != 1)
                break;
            const double radius = unscale<double>(std::sqrt(std::abs(island->area()) / PI));
            CAPTURE(layer_height, top_gap, depth, previous_radius, radius);
            // A shoulder produces a sudden radial jump, even when every upper
            // layer is fully supported. Bound the surface slope across the join.
            CHECK((radius - previous_radius) / (previous_z - layer.print_z) < 1.5);
            if (depth >= 0.7 && depth <= 1.2)
                ++ approach_layers;
            previous_radius = radius;
            previous_z = layer.print_z;
        }
        if (approach_layers >= 6)
            ++ checked;
    }
    REQUIRE(checked >= 1);
}

TEST_CASE("Organic Pin contacts remain connected through displaced joins and nearby junctions", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    const double layer_height = GENERATE(0.05, 0.06);
    const double spacing = GENERATE(1.5, 4.0);
    const double height_scale = GENERATE(0.2, 1.0);
    DynamicPrintConfig config = organic_contact_config(spacing, 20.);
    config.set_deserialize_strict({
        { "layer_height", layer_height },
        { "initial_layer_print_height", layer_height },
        { "support_top_z_distance", 0.0 },
        { "tree_support_round_tip", true },
    });
    TriangleMesh fixture = sloped_underside_coupon(20.);
    fixture.scale(Vec3f(1.f, 1.f, float(height_scale)));
    Print print;
    Model model;
    init_print({ fixture }, print, model, config);
    print.process();
    CAPTURE(layer_height, spacing, height_scale);
    require_connected_contacts(*print.objects().front());
}

TEST_CASE("Organic Pins report contacts whose bed-clipped connections cannot be printed", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    DynamicPrintConfig config = organic_contact_config(4., 10.);
    config.set_deserialize_strict({
        { "layer_height", 0.05 },
        { "initial_layer_print_height", 0.05 },
        { "support_top_z_distance", 0.0 },
        { "tree_support_round_tip", true },
    });
    config.set_key_value("printable_area", new ConfigOptionPoints{
        Vec2d(0., 0.), Vec2d(10., 0.), Vec2d(10., 10.), Vec2d(0., 10.) });
    Print print;
    Model model;
    init_print({ sloped_underside_coupon(0.) }, print, model, config);
    try {
        print.process();
        FAIL("A clipped-away Pin connection must stop slicing");
    } catch (const SlicingError &error) {
        CHECK_THAT(error.what(), Catch::Matchers::ContainsSubstring("object.stl"));
        CHECK_THAT(error.what(), Catch::Matchers::ContainsSubstring("X="));
        CHECK_THAT(error.what(), Catch::Matchers::ContainsSubstring("Y="));
        CHECK_THAT(error.what(), Catch::Matchers::ContainsSubstring("Z="));
        CHECK_THAT(error.what(), Catch::Matchers::ContainsSubstring("disable round Pin tips"));
    }
}

TEST_CASE("Direct Organic Pin tips follow printable underside normals", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    const double layer_height = GENERATE(0.05, 0.06);
    // The 20-degree underside resolves a normal at both limits, so the
    // constrained case measures an actual tilted Pin.
    const auto [underside_angle, maximum_tip_angle] = GENERATE(table<double, double>({
        { 0., 30. }, { 20., 30. }, { 20., 10. }
    }));
    CAPTURE(underside_angle, maximum_tip_angle);

    DynamicPrintConfig config = organic_contact_config(4., 10.);
    config.set_deserialize_strict({
        { "layer_height",                      layer_height },
        { "initial_layer_print_height",        layer_height },
        { "support_top_z_distance",            0.0 },
        { "tree_support_branch_angle_organic", maximum_tip_angle },
        { "tree_support_round_tip",            true },
    });

    Print print;
    Model model;
    init_print({ sloped_underside_coupon(underside_angle) }, print, model, config);
    print.process();

    const PrintObject &object = *print.objects().front();
    REQUIRE_FALSE(object.support_contacts().empty());
    for (const SupportContact &contact : object.support_contacts())
        CHECK_THAT(contact.model_contact_z, Catch::Matchers::WithinAbs(contact.support_tip_z, 1e-6));
    const TerminalFootprintStats footprints = unobstructed_terminal_footprint_stats(object);
    if (footprints.count > 0)
        CHECK(footprints.max_aspect_ratio <= 1.10);

    const TerminalRunupStats runup = terminal_runup_stats(object);
    CAPTURE(runup.count, runup.median_angle_degrees, runup.max_angle_degrees,
            runup.median_x_slope, runup.minimum_support_ratio);
    REQUIRE(runup.count >= 1);
    CHECK(runup.minimum_support_ratio >= 0.98);
    if (underside_angle < EPSILON) {
        CHECK(runup.median_angle_degrees <= 1.0);
    } else if (underside_angle < maximum_tip_angle) {
        CHECK_THAT(runup.max_angle_degrees,
                   Catch::Matchers::WithinAbs(underside_angle, 2.0));
        CHECK(runup.median_x_slope > 0.);
    } else {
        CHECK_THAT(runup.max_angle_degrees,
                   Catch::Matchers::WithinAbs(maximum_tip_angle, 1.0));
        CHECK(runup.median_x_slope > 0.);
    }
}

TEST_CASE("Organic Pins sample normals at rendered contacts on curved surfaces", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    const double layer_height = GENERATE(0.05, 0.06);
    DynamicPrintConfig config = organic_contact_config(3., 13.33);
    config.set_deserialize_strict({
        { "layer_height", layer_height },
        { "initial_layer_print_height", layer_height },
        { "support_top_z_distance", 0.0 },
        { "tree_support_round_tip", true },
        { "tree_support_branch_angle_organic", 30.0 },
    });
    // A shallow ellipsoid provides curved normals within the permitted angle
    // over a broad region outside the stem's collision clearance.
    TriangleMesh fixture = make_sphere(8., PI / 180.);
    fixture.scale(Vec3f(1.f, 1.f, 0.3f));
    fixture.translate(Vec3f(0.f, 0.f, 13.f));
    TriangleMesh stem = make_cube(1., 1., 13.);
    stem.translate(Vec3f(-7.5f, -0.5f, 0.f));
    fixture.merge(stem);
    Print print;
    Model model;
    init_print({ fixture }, print, model, config);
    print.process();
    const PrintObject &object = *print.objects().front();
    TriangleMesh transformed = object.model_object()->raw_mesh();
    transformed.transform(object.trafo_centered(), true);
    const AABBMesh surface(transformed);
    require_connected_contacts(object);
    const auto samples = terminal_runup_samples(object, layer_height);
    REQUIRE_FALSE(samples.empty());
    size_t aligned = 0;
    size_t reduced = 0;
    for (const auto &sample : samples) {
        const Vec2d stem_center = (object.trafo_centered() * Vec3d(-7., 0., 0.)).head<2>();
        // The central stem obstructs the approach of these contacts. Reduced
        // tilt there is expected; measure normals where the curve has room.
        if ((sample.contact_center - stem_center).norm() < 3.)
            continue;
        const auto contact = std::find_if(object.support_contacts().begin(), object.support_contacts().end(),
            [&](const SupportContact &value) {
                return (unscaled<double>(value.position) - sample.contact_center).norm() < EPSILON;
            });
        REQUIRE(contact != object.support_contacts().end());
        const auto hit = surface.query_ray_hit(Vec3d(sample.contact_center.x(), sample.contact_center.y(),
                                                     contact->support_tip_z - 0.5), Vec3d::UnitZ());
        REQUIRE(hit.is_hit());
        const Vec3d normal = -hit.normal().normalized();
        if (normal.z() < std::cos(28. * PI / 180.))
            continue;
        const Vec3d axis = Vec3d(sample.slope.x(), sample.slope.y(), 1.).normalized();
        const double error = std::acos(std::clamp(axis.dot(normal), -1., 1.)) * 180. / PI;
        CAPTURE(layer_height, sample.contact_center, sample.slope, normal, error);
        // Short branches on this curved fixture sometimes require less tilt.
        // They must still face the surface normal's azimuth, never lean past it.
        CHECK(sample.angle_degrees <= std::acos(normal.z()) * 180. / PI + 2.);
        if (sample.slope.norm() > 0.01) {
            const double azimuth_error = std::acos(std::clamp(
                sample.slope.normalized().dot(normal.head<2>().normalized()), -1., 1.)) * 180. / PI;
            CHECK(azimuth_error <= 2.);
        }
        if (error <= 2.)
            ++ aligned;
        else
            ++ reduced;
    }
    REQUIRE(aligned > 0);
    if (layer_height == 0.05)
        REQUIRE(reduced > 0);
}

TEST_CASE("Direct Organic Pin tips follow underside normals in multiple directions", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    DynamicPrintConfig config = organic_contact_config(3., 13.33);
    config.set_deserialize_strict({
        { "layer_height",                      0.05 },
        { "initial_layer_print_height",        0.05 },
        { "support_top_z_distance",            0.0 },
        { "tree_support_branch_angle_organic", 30.0 },
        { "tree_support_round_tip",            true },
    });

    Print print;
    Model model;
    init_print({ multi_normal_underside_coupon() }, print, model, config);
    // Keep the complete cross-shaped fixture inside the finite default bed.
    ModelInstance *instance = model.objects.front()->instances.front();
    instance->set_offset(instance->get_offset() + Vec3d(100., 100., 0.));
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    print.process();

    const PrintObject &object = *print.objects().front();
    require_connected_contacts(object);
    const std::vector<TerminalRunupSample> samples = terminal_runup_samples(object);
    CAPTURE(samples.size(), object.support_contacts().size());
    REQUIRE_FALSE(samples.empty());
    // Original Organic fallbacks are checked for connectivity above; only
    // isolated round Pins have a measurable runup axis.
    const auto &panels = multi_normal_test_panels();
    const Transform3d object_transform = object.trafo_centered();
    const double maximum_panel_distance =
        std::sqrt(0.5 * sqr(multi_normal_panel_size_mm)) + 0.1;
    std::vector<Vec2d> transformed_panel_centers;
    transformed_panel_centers.reserve(panels.size());
    for (const NormalTestPanel &panel : panels)
        transformed_panel_centers.emplace_back(
            (object_transform * Vec3d(panel.center.x(), panel.center.y(), 15.)).head<2>());
    std::vector<std::vector<TerminalRunupSample>> panel_samples(panels.size());
    for (const TerminalRunupSample &sample : samples) {
        size_t nearest_panel = 0;
        double nearest_distance = std::numeric_limits<double>::max();
        for (size_t panel_idx = 0; panel_idx < panels.size(); ++ panel_idx) {
            const double distance =
                (sample.contact_center - transformed_panel_centers[panel_idx]).norm();
            if (distance < nearest_distance) {
                nearest_panel = panel_idx;
                nearest_distance = distance;
            }
        }
        CAPTURE(sample.contact_center.x(), sample.contact_center.y(), nearest_panel, nearest_distance);
        CHECK(nearest_distance <= maximum_panel_distance);
        panel_samples[nearest_panel].push_back(sample);
    }

    for (size_t panel_idx = 0; panel_idx < panels.size(); ++ panel_idx) {
        const NormalTestPanel &panel = panels[panel_idx];
        const std::vector<TerminalRunupSample> &region = panel_samples[panel_idx];
        CAPTURE(panel_idx, panel.center.x(), panel.center.y(),
                panel.x_rotation_degrees, panel.y_rotation_degrees, region.size());
        REQUIRE_FALSE(region.empty());
        for (const TerminalRunupSample &sample : region)
            CHECK(sample.minimum_support_ratio >= 0.98);

        const Vec2d model_slope = panel.expected_slope();
        const Vec3d transformed_axis = object_transform.linear() *
            Vec3d(model_slope.x(), model_slope.y(), 1.).normalized();
        const Vec2d expected_slope = transformed_axis.head<2>() / transformed_axis.z();
        if (expected_slope.squaredNorm() <= EPSILON) {
            for (const TerminalRunupSample &sample : region) {
                CAPTURE(sample.contact_center.x(), sample.contact_center.y(),
                        sample.slope.x(), sample.slope.y(), sample.angle_degrees);
                CHECK(sample.angle_degrees <= 1.0);
            }
            continue;
        }

        const Vec2d expected_direction = expected_slope.normalized();
        // Contacts close to a rotated roof edge may intentionally resolve to a side-face
        // normal. Select the sample aligned with the broad underside to test this panel.
        const auto most_aligned = std::max_element(
            region.begin(), region.end(),
            [&expected_direction](const TerminalRunupSample &lhs, const TerminalRunupSample &rhs) {
                return lhs.slope.dot(expected_direction) < rhs.slope.dot(expected_direction);
            });
        const double expected_angle = std::atan(expected_slope.norm()) * 180. / PI;
        const double projected_slope = most_aligned->slope.dot(expected_direction);
        const double projected_angle = std::atan(std::max(0., projected_slope)) * 180. / PI;
        const double direction_cosine = most_aligned->slope.squaredNorm() > EPSILON ?
            most_aligned->slope.normalized().dot(expected_direction) : 0.;
        CAPTURE(most_aligned->contact_center.x(), most_aligned->contact_center.y(),
                expected_slope.x(), expected_slope.y(), most_aligned->slope.x(), most_aligned->slope.y(),
                expected_angle, projected_angle, direction_cosine);
        CHECK(projected_slope > 0.);
        CHECK_THAT(projected_angle, Catch::Matchers::WithinAbs(expected_angle, 2.0));
        CHECK(direction_cosine >= 0.98);
    }
}

TEST_CASE("Direct Organic Pin tips fall back vertically for negative-volume objects", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    DynamicPrintConfig config = organic_contact_config(4., 10.);
    config.set_deserialize_strict({
        { "layer_height",                      0.05 },
        { "initial_layer_print_height",        0.05 },
        { "support_top_z_distance",            0.0 },
        { "tree_support_branch_angle_organic", 10.0 },
        { "tree_support_round_tip",            true },
    });

    auto prepare = [&config](Print &print, Model &model) {
        init_print({ sloped_underside_coupon(55.) }, print, model, config);
        TriangleMesh negative = make_cube(0.5, 0.5, 0.5);
        negative.translate(Vec3f(-0.25f, -0.25f, 2.f));
        model.objects.front()->add_volume(std::move(negative), ModelVolumeType::NEGATIVE_VOLUME);
        print.apply(model, config);
        print.validate();
        print.set_status_silent();
        print.process();
    };

    Print first;
    Model first_model;
    prepare(first, first_model);
    const PrintObject &first_object = *first.objects().front();
    const TerminalRunupStats first_runup = terminal_runup_stats(first_object);
    CAPTURE(first_runup.count, first_runup.median_angle_degrees,
            first_runup.max_angle_degrees, first_runup.minimum_support_ratio);
    REQUIRE(first_runup.count >= 1);
    CHECK(first_runup.median_angle_degrees <= 1.0);
    CHECK(first_runup.minimum_support_ratio >= 0.98);

    Print second;
    Model second_model;
    prepare(second, second_model);
    CHECK(contact_signatures(*second.objects().front()) == contact_signatures(first_object));
    const TerminalRunupStats second_runup = terminal_runup_stats(*second.objects().front());
    CHECK_THAT(second_runup.median_angle_degrees,
               Catch::Matchers::WithinAbs(first_runup.median_angle_degrees, 1e-6));
}

TEST_CASE("Direct Organic Pin tips fall back vertically for invalid surface hits", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    DynamicPrintConfig config = organic_contact_config(4., 10.);
    config.set_deserialize_strict({
        { "layer_height",                      0.05 },
        { "initial_layer_print_height",        0.05 },
        { "support_top_z_distance",            0.0 },
        { "tree_support_branch_angle_organic", 30.0 },
        { "tree_support_round_tip",            true },
    });

    struct Result {
        std::vector<ContactSignature> contacts;
        TerminalRunupStats            runup;
    };
    const auto slice_invalid_surface = [&config]() {
        Print print;
        Model model;
        init_print({ sloped_underside_coupon(20., true) }, print, model, config);
        print.process();
        const PrintObject &object = *print.objects().front();
        return Result{ contact_signatures(object), terminal_runup_stats(object) };
    };

    const Result first = slice_invalid_surface();
    CAPTURE(first.runup.count, first.runup.median_angle_degrees,
            first.runup.max_angle_degrees, first.runup.minimum_support_ratio);
    REQUIRE(first.runup.count >= 1);
    CHECK(first.runup.median_angle_degrees <= 1.0);
    CHECK(first.runup.minimum_support_ratio >= 0.98);

    const Result second = slice_invalid_surface();
    CHECK(second.contacts == first.contacts);
    CHECK_THAT(second.runup.median_angle_degrees,
               Catch::Matchers::WithinAbs(first.runup.median_angle_degrees, 1e-6));
}

TEST_CASE("Organic Pin contact Z matches one two and three layer gaps", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    const double layer_height = GENERATE(0.05, 0.06);
    const int gap_layers = GENERATE(1, 2, 3);
    const double top_gap = gap_layers * layer_height;
    CAPTURE(layer_height, gap_layers, top_gap);

    DynamicPrintConfig config = organic_contact_config(3., 13.33);
    config.set_deserialize_strict({
        { "layer_height",               layer_height },
        { "initial_layer_print_height", layer_height },
        { "support_top_z_distance",     top_gap },
        { "tree_support_round_tip",     true },
    });

    TriangleMesh coupon = load_model("support_contact_coupon.obj");
    coupon.scale(Vec3f(1.f, 1.f, 0.25f));
    Print print;
    Model model;
    init_print({ std::move(coupon) }, print, model, config);
    print.process();

    const std::vector<SupportContact> &contacts = print.objects().front()->support_contacts();
    REQUIRE_FALSE(contacts.empty());
    for (const SupportContact &contact : contacts)
        CHECK_THAT(contact.model_contact_z - contact.support_tip_z,
                   Catch::Matchers::WithinAbs(top_gap, 1e-6));
}

TEST_CASE("Prusa XL miniature profiles realize direct Pin contacts on physical tool 2", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    static constexpr const char *machine_name = "Prusa XL 5T T2 0.25 nozzle (others 0.4)";
    static constexpr const char *filament_name = "Prusa Generic Miniature PLA @XL 5T";
    const std::array<const char *, 2> process_names {
        "0.05mm Miniature Ultra Detail + Pin @Prusa XL 5T T2 0.25",
        "0.06mm Miniature Balanced + Pin @Prusa XL 5T T2 0.25"
    };

    PresetBundle bundle;
    bundle.set_is_validation_mode(true);
    REQUIRE(bundle.load_vendor_configs_from_json(
                PROFILES_DIR, "Prusa", PresetBundle::LoadSystem,
                ForwardCompatibilitySubstitutionRule::EnableSilent).second > 0);

    Preset *machine = bundle.printers.find_preset(machine_name, false, true);
    Preset *filament = bundle.filaments.find_preset(filament_name, false, true);
    REQUIRE(machine != nullptr);
    REQUIRE(filament != nullptr);

    for (const char *process_name : process_names) {
        DYNAMIC_SECTION(process_name) {
            Preset *process = bundle.prints.find_preset(process_name, false, true);
            REQUIRE(process != nullptr);

            DynamicPrintConfig project_config = bundle.project_config;
            project_config.option<ConfigOptionInts>("filament_map", true)->values = { 1, 1, 1, 1, 1 };
            project_config.option<ConfigOptionInts>("filament_nozzle_map", true)->values = { 0, 0, 0, 0, 0 };
            project_config.option<ConfigOptionInts>("filament_volume_map", true)->values = { 0, 0, 0, 0, 0 };
            project_config.option<ConfigOptionStrings>("filament_colour", true)->values = {
                "#26A69A", "#26A69A", "#26A69A", "#26A69A", "#26A69A"
            };
            project_config.option<ConfigOptionFloats>("flush_multiplier", true)->values = { 0.3 };
            project_config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values.assign(25, 0.);
            std::vector<Preset> selected_filaments(5, *filament);
            DynamicPrintConfig config = PresetBundle::construct_full_config(
                *machine, *process, project_config, selected_filaments, true, std::nullopt);
            config.set_key_value("gcode_comments", new ConfigOptionBool(true));
            CHECK(config.opt_bool("tree_support_round_tip"));
            CHECK_THAT(config.opt_float("tree_support_angle_slow"),
                       Catch::Matchers::WithinAbs(25.0, 1e-9));
            CHECK_THAT(config.opt_float("support_top_z_distance"),
                       Catch::Matchers::WithinAbs(0.0, 1e-9));

            TriangleMesh coupon = load_model("support_contact_coupon.obj");
            coupon.scale(Vec3f(1.f, 1.f, 0.25f));
            Print print;
            Model model;
            init_print({ std::move(coupon) }, print, model, config);
            ModelInstance *instance = model.objects.front()->instances.front();
            instance->set_offset(instance->get_offset() + Vec3d(100., 100., 0.));
            print.apply(model, config);
            print.process();

            const PrintObject &object = *print.objects().front();
            REQUIRE(object.support_contacts().size() > 1);
            for (const SupportContact &contact : object.support_contacts())
                CHECK_THAT(contact.model_contact_z - contact.support_tip_z,
                           Catch::Matchers::WithinAbs(0.0, 1e-6));
            const std::vector<ContactSignature> signatures = contact_signatures(object);
            CHECK(std::is_sorted(signatures.begin(), signatures.end()));
            CHECK_THAT(median_nearest_neighbour_distance(object.support_contacts()),
                       Catch::Matchers::WithinAbs(3., 0.6));
            CHECK(print.validate_support_contact_export().empty());

            TriangleMesh repeated_coupon = load_model("support_contact_coupon.obj");
            repeated_coupon.scale(Vec3f(1.f, 1.f, 0.25f));
            Print repeated_print;
            Model repeated_model;
            init_print({ std::move(repeated_coupon) }, repeated_print, repeated_model, config);
            ModelInstance *repeated_instance = repeated_model.objects.front()->instances.front();
            repeated_instance->set_offset(repeated_instance->get_offset() + Vec3d(100., 100., 0.));
            repeated_print.apply(repeated_model, config);
            repeated_print.process();
            CHECK(contact_signatures(*repeated_print.objects().front()) == signatures);

            ScopedTemporaryFile report_file(".support-contacts.json");
            print.export_support_contacts(report_file.string(), 0);
            std::ifstream report_stream(report_file.string());
            const nlohmann::json report = nlohmann::json::parse(report_stream);
            CHECK(report.at("schema_version") == 2);
            CHECK(report.at("contact_count") == object.support_contacts().size());

            ScopedTemporaryFile repeated_report_file(".support-contacts.json");
            repeated_print.export_support_contacts(repeated_report_file.string(), 0);
            std::ifstream repeated_report_stream(repeated_report_file.string());
            nlohmann::json repeated_report = nlohmann::json::parse(repeated_report_stream);
            nlohmann::json normalized_report = report;
            for (nlohmann::json *candidate : { &normalized_report, &repeated_report }) {
                for (nlohmann::json &instance : candidate->at("instances")) {
                    instance.erase("object_id");
                    instance.erase("instance_id");
                }
            }
            CHECK(repeated_report == normalized_report);

            TriangleMesh miniature_fixture = mesh(TestMesh::overhang);
            miniature_fixture.scale(Vec3f(0.2f, 0.2f, 0.25f));
            Print miniature_print;
            Model miniature_model;
            init_print({ std::move(miniature_fixture) }, miniature_print, miniature_model, config);
            ModelInstance *miniature_instance = miniature_model.objects.front()->instances.front();
            miniature_instance->set_offset(miniature_instance->get_offset() + Vec3d(100., 100., 0.));
            miniature_print.apply(miniature_model, config);
            miniature_print.process();
            CHECK_FALSE(miniature_print.objects().front()->support_contacts().empty());
            CHECK(miniature_print.validate_support_contact_export().empty());

            const std::string generated = gcode(print);
            CHECK_FALSE(generated.empty());
            CHECK_FALSE(layers_with_role(generated, "support").empty());
            CHECK(layers_with_role(generated, "support interface").empty());
            CHECK(generated.find("M900 K0.14") != std::string::npos);
            CHECK(generated.find("M572 S0.12") != std::string::npos);

            std::set<int> selected_tools;
            std::istringstream lines(generated);
            for (std::string line; std::getline(lines, line);) {
                if (line.size() < 2 || line.front() != 'T' || !std::isdigit(static_cast<unsigned char>(line[1])))
                    continue;
                size_t consumed = 0;
                const int tool = std::stoi(line.substr(1), &consumed);
                if (consumed > 0 && (consumed + 1 == line.size() || std::isspace(static_cast<unsigned char>(line[consumed + 1]))))
                    selected_tools.insert(tool);
            }
            CHECK(selected_tools == std::set<int>{ 1 });
            REQUIRE(print.get_filament_maps().size() >= 2);
            CHECK(print.get_filament_maps()[1] == 2);
        }
    }
}

#include "OrganicSupportAssociation.hpp"

#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <tuple>
#include <unordered_map>

namespace Slic3r::OrganicSupport {

namespace {

constexpr double geometry_epsilon = 1e-9;

struct Segment
{
    Linef  line;
    size_t chain_idx { 0 };
    size_t chain_segment_idx { 0 };
    size_t leaf_idx { 0 };
    double s_begin { 0. };
    double length { 0. };
    double width { 0. };
};

struct RuntimeChain
{
    const PathChain     *source { nullptr };
    std::vector<Segment> segments;
    double               length { 0. };
};

struct LayerIndex
{
    Linesf                                       lines;
    std::vector<const Segment *>                 segments;
    AABBTreeIndirect::Tree<2, double>            tree;
    double                                       maximum_width { 0. };
};

struct ClosestLocation
{
    double distance { std::numeric_limits<double>::infinity() };
    Vec2d  point { Vec2d::Zero() };
    double arclength { 0. };
    size_t segment_idx { 0 };
};

Vec2d to_unscaled(const Point &point)
{
    return Vec2d(unscale<double>(point.x()), unscale<double>(point.y()));
}

bool points_near(const Vec2d &lhs, const Vec2d &rhs)
{
    return (lhs - rhs).squaredNorm() <= geometry_epsilon * geometry_epsilon;
}

std::vector<PathInterval> merge_intervals(std::vector<PathInterval> intervals)
{
    intervals.erase(std::remove_if(intervals.begin(), intervals.end(), [](const PathInterval &interval) {
        return interval.end + geometry_epsilon < interval.begin;
    }), intervals.end());
    std::sort(intervals.begin(), intervals.end(), [](const PathInterval &lhs, const PathInterval &rhs) {
        return std::tie(lhs.begin, lhs.end) < std::tie(rhs.begin, rhs.end);
    });

    std::vector<PathInterval> out;
    for (const PathInterval &interval : intervals) {
        if (out.empty() || interval.begin > out.back().end + geometry_epsilon)
            out.push_back(interval);
        else
            out.back().end = std::max(out.back().end, interval.end);
    }
    return out;
}

std::optional<PathInterval> quadratic_interval(const Vec2d &origin,
                                               const Vec2d &direction,
                                               double       radius,
                                               double       lower,
                                               double       upper)
{
    const double a = direction.squaredNorm();
    const double b = 2. * origin.dot(direction);
    const double c = origin.squaredNorm() - radius * radius;
    if (a <= geometry_epsilon) {
        if (c <= geometry_epsilon)
            return PathInterval{ lower, upper };
        return std::nullopt;
    }

    double discriminant = b * b - 4. * a * c;
    if (discriminant < -geometry_epsilon)
        return std::nullopt;
    discriminant = std::max(0., discriminant);
    const double root = std::sqrt(discriminant);
    const double t0 = (-b - root) / (2. * a);
    const double t1 = (-b + root) / (2. * a);
    const double begin = std::max(lower, t0);
    const double end   = std::min(upper, t1);
    if (end + geometry_epsilon < begin)
        return std::nullopt;
    return PathInterval{ begin, end };
}

std::optional<PathInterval> circle_interval(const Linef &line, const Vec2d &center, double radius)
{
    return quadratic_interval(line.a - center, line.vector(), radius, 0., 1.);
}

// Return the interval on current for which its centerline lies in the lower
// segment's radius-expanded capsule. Splitting at the projection boundaries
// makes each piece a single quadratic inequality.
std::optional<PathInterval> capsule_interval(const Linef &current, const Linef &lower, double radius)
{
    const Vec2d current_v = current.vector();
    const Vec2d lower_v   = lower.vector();
    const double lower_l2 = lower_v.squaredNorm();
    if (lower_l2 <= geometry_epsilon)
        return circle_interval(current, lower.a, radius);

    const double projection_0 = (current.a - lower.a).dot(lower_v) / lower_l2;
    const double projection_v = current_v.dot(lower_v) / lower_l2;
    std::vector<double> splits { 0., 1. };
    if (std::abs(projection_v) > geometry_epsilon) {
        const double at_start = -projection_0 / projection_v;
        const double at_end   = (1. - projection_0) / projection_v;
        if (at_start > 0. && at_start < 1.) splits.push_back(at_start);
        if (at_end > 0. && at_end < 1.) splits.push_back(at_end);
    }
    std::sort(splits.begin(), splits.end());
    splits.erase(std::unique(splits.begin(), splits.end(), [](double lhs, double rhs) {
        return std::abs(lhs - rhs) <= geometry_epsilon;
    }), splits.end());

    std::vector<PathInterval> pieces;
    for (size_t i = 1; i < splits.size(); ++i) {
        const double lower_t = splits[i - 1];
        const double upper_t = splits[i];
        const double middle  = 0.5 * (lower_t + upper_t);
        const double projection = projection_0 + middle * projection_v;
        std::optional<PathInterval> interval;
        if (projection <= 0.) {
            interval = quadratic_interval(current.a - lower.a, current_v, radius, lower_t, upper_t);
        } else if (projection >= 1.) {
            interval = quadratic_interval(current.a - lower.b, current_v, radius, lower_t, upper_t);
        } else {
            const Vec2d perpendicular_origin = current.a - lower.a - projection_0 * lower_v;
            const Vec2d perpendicular_v      = current_v - projection_v * lower_v;
            interval = quadratic_interval(perpendicular_origin, perpendicular_v, radius, lower_t, upper_t);
        }
        if (interval)
            pieces.push_back(*interval);
    }

    const std::vector<PathInterval> merged = merge_intervals(std::move(pieces));
    if (merged.empty())
        return std::nullopt;
    // A line intersected with a capsule (a convex set) has one interval.
    return PathInterval{ merged.front().begin, merged.back().end };
}

RuntimeChain make_runtime_chain(const PathChain &chain, size_t chain_idx)
{
    RuntimeChain runtime;
    runtime.source = &chain;
    for (size_t leaf_idx = 0; leaf_idx < chain.leaves.size(); ++leaf_idx) {
        const PathLeaf &leaf = chain.leaves[leaf_idx];
        for (size_t point_idx = 1; point_idx < leaf.points.size(); ++point_idx) {
            Linef line(leaf.points[point_idx - 1], leaf.points[point_idx]);
            const double length = line.length();
            if (length <= geometry_epsilon)
                continue;
            runtime.segments.push_back({ line, chain_idx, runtime.segments.size(), leaf_idx,
                                         runtime.length, length, leaf.width });
            runtime.length += length;
        }
    }
    return runtime;
}

std::map<size_t, LayerIndex> make_layer_indices(const std::vector<RuntimeChain> &chains)
{
    std::map<size_t, LayerIndex> out;
    for (const RuntimeChain &chain : chains) {
        LayerIndex &index = out[chain.source->layer_id];
        for (const Segment &segment : chain.segments) {
            index.lines.push_back(segment.line);
            index.segments.push_back(&segment);
            index.maximum_width = std::max(index.maximum_width, segment.width);
        }
    }
    for (auto &[layer_id, index] : out)
        index.tree = AABBTreeLines::build_aabb_tree_over_indexed_lines(index.lines);
    return out;
}

ClosestLocation closest_location(const RuntimeChain &chain,
                                 const Vec2d         &point,
                                 size_t               leaf_idx,
                                 const std::optional<PathInterval> &limit)
{
    ClosestLocation closest;
    for (const Segment &segment : chain.segments) {
        if (segment.leaf_idx != leaf_idx)
            continue;
        double t_min = 0.;
        double t_max = 1.;
        if (limit) {
            t_min = std::max(t_min, (limit->begin - segment.s_begin) / segment.length);
            t_max = std::min(t_max, (limit->end   - segment.s_begin) / segment.length);
            if (t_max + geometry_epsilon < t_min)
                continue;
        }
        const Vec2d vector = segment.line.vector();
        const double projected = vector.squaredNorm() <= geometry_epsilon ? 0. :
            (point - segment.line.a).dot(vector) / vector.squaredNorm();
        const double t = std::clamp(projected, t_min, t_max);
        const Vec2d candidate = segment.line.a + t * vector;
        const double distance = (candidate - point).norm();
        const double arclength = segment.s_begin + t * segment.length;
        if (distance < closest.distance - geometry_epsilon ||
            (std::abs(distance - closest.distance) <= geometry_epsilon && arclength < closest.arclength)) {
            closest = { distance, candidate, arclength, segment.chain_segment_idx };
        }
    }
    return closest;
}

std::pair<Vec2d, bool> tangent_at(const RuntimeChain &chain, const ClosestLocation &location)
{
    if (chain.segments.empty())
        return { Vec2d::Zero(), true };
    const Segment &segment = chain.segments[location.segment_idx];
    const Vec2d fallback = segment.line.unit_vector();
    const double at = location.arclength - segment.s_begin;
    const bool at_start = at <= geometry_epsilon;
    const bool at_end   = segment.length - at <= geometry_epsilon;
    if (!at_start && !at_end)
        return { fallback, false };

    const Segment *previous = nullptr;
    const Segment *next = nullptr;
    if (at_start) {
        if (location.segment_idx > 0)
            previous = &chain.segments[location.segment_idx - 1];
        else if (chain.source->closed)
            previous = &chain.segments.back();
        next = &segment;
    } else {
        previous = &segment;
        if (location.segment_idx + 1 < chain.segments.size())
            next = &chain.segments[location.segment_idx + 1];
        else if (chain.source->closed)
            next = &chain.segments.front();
    }
    if (previous == nullptr || next == nullptr || !points_near(previous->line.b, next->line.a))
        return { fallback, false };

    const Vec2d sum = previous->line.unit_vector() + next->line.unit_vector();
    if (sum.norm() <= geometry_epsilon)
        return { fallback, true };
    return { sum.normalized(), false };
}

bool interval_overlaps(const PathInterval &lhs, const PathInterval &rhs)
{
    return lhs.begin <= rhs.end + geometry_epsilon && rhs.begin <= lhs.end + geometry_epsilon;
}

template<typename Tree>
void query_bbox(const Tree &tree,
                size_t node_idx,
                const typename Tree::BoundingBox &bbox,
                std::vector<size_t> &out)
{
    const auto &node = tree.node(node_idx);
    if (!node.is_valid() || !node.bbox.intersects(bbox))
        return;
    if (node.is_leaf()) {
        out.push_back(node.idx);
        return;
    }
    query_bbox(tree, Tree::left_child_idx(node_idx), bbox, out);
    query_bbox(tree, Tree::right_child_idx(node_idx), bbox, out);
}

DirectionalUnsupportedSpan directional_span(const PathInterval                    &current,
                                             size_t                                 current_contact_id,
                                             const std::vector<PlannedMaterialAnchor> &anchors,
                                             double                                 chain_length,
                                             bool                                   closed,
                                             bool                                   forward)
{
    DirectionalUnsupportedSpan out;
    double nearest = std::numeric_limits<double>::infinity();
    auto consider = [&](double distance, const PlannedMaterialAnchor &anchor) {
        if (distance < nearest - geometry_epsilon) {
            nearest = distance;
            out.anchors.assign(1, anchor);
        } else if (std::abs(distance - nearest) <= geometry_epsilon) {
            out.anchors.push_back(anchor);
        }
    };

    for (const PlannedMaterialAnchor &anchor : anchors) {
        const bool is_current_occurrence = anchor.type == AnchorType::Pin &&
            anchor.contact_id == current_contact_id && interval_overlaps(anchor.interval, current);
        if (is_current_occurrence)
            continue;
        if (interval_overlaps(anchor.interval, current)) {
            consider(0., anchor);
            continue;
        }

        double distance;
        if (forward) {
            distance = anchor.interval.begin - current.end;
            if (closed && distance < -geometry_epsilon)
                distance += chain_length;
        } else {
            distance = current.begin - anchor.interval.end;
            if (closed && distance < -geometry_epsilon)
                distance += chain_length;
        }
        if (distance >= -geometry_epsilon)
            consider(std::max(0., distance), anchor);
    }

    if (closed) {
        // A complete traversal may terminate at the opposite edge of the same
        // pin even when there is no other planned material on the loop.
        PlannedMaterialAnchor self;
        self.type = AnchorType::Pin;
        self.contact_id = current_contact_id;
        self.interval = current;
        consider(std::max(0., chain_length - current.length()), self);
    }

    if (std::isfinite(nearest)) {
        out.span = nearest;
        std::sort(out.anchors.begin(), out.anchors.end(), [](const PlannedMaterialAnchor &lhs,
                                                            const PlannedMaterialAnchor &rhs) {
            if (lhs.type != rhs.type) return lhs.type < rhs.type;
            if (lhs.contact_id != rhs.contact_id) return lhs.contact_id < rhs.contact_id;
            if (lhs.extrusion_id != rhs.extrusion_id) return lhs.extrusion_id < rhs.extrusion_id;
            return std::tie(lhs.interval.begin, lhs.interval.end) < std::tie(rhs.interval.begin, rhs.interval.end);
        });
    } else {
        out.unanchored_distance = forward ? std::max(0., chain_length - current.end) : std::max(0., current.begin);
    }
    return out;
}

bool supported_role(ExtrusionRole role)
{
    return is_perimeter(role) || is_infill(role) || role == erGapFill;
}

std::vector<Vec2d> tessellated_points_impl(const ExtrusionPath &path, bool use_fitted_arcs, double tolerance)
{
    auto raw_points = [&path]() {
        std::vector<Vec2d> out;
        out.reserve(path.polyline.points.size());
        for (const Point3 &point : path.polyline.points)
            out.push_back(to_unscaled(point.to_point()));
        return out;
    };

    const std::vector<PathFittingData> &fittings = path.polyline.fitting_result;
    if (!use_fitted_arcs || fittings.empty() || path.z_contoured ||
        dynamic_cast<const ExtrusionPathSloped *>(&path) != nullptr)
        return raw_points();

    std::vector<Vec2d> out;
    for (const PathFittingData &fitting : fittings) {
        if (fitting.start_point_index >= path.polyline.points.size() ||
            fitting.end_point_index >= path.polyline.points.size() ||
            fitting.end_point_index < fitting.start_point_index)
            return raw_points();
        const Vec2d start = to_unscaled(path.polyline.points[fitting.start_point_index].to_point());
        if (out.empty()) out.push_back(start);
        else if (!points_near(out.back(), start)) out.push_back(start);

        if ((fitting.path_type == EMovePathType::Arc_move_cw ||
             fitting.path_type == EMovePathType::Arc_move_ccw) && fitting.arc_data.is_valid()) {
            const ArcSegment &arc = fitting.arc_data;
            const double radius = unscale<double>(arc.radius);
            const double angle = arc.angle_radians;
            if (radius <= geometry_epsilon || std::abs(angle) <= geometry_epsilon)
                return raw_points();
            const double bounded_tolerance = std::max(tolerance, geometry_epsilon);
            const double ratio = std::clamp(1. - bounded_tolerance / radius, -1., 1.);
            const double maximum_angle = 2. * std::acos(ratio);
            const size_t steps = std::max<size_t>(1, size_t(std::ceil(std::abs(angle) /
                std::max(maximum_angle, geometry_epsilon))));
            const Vec2d center = to_unscaled(arc.center);
            for (size_t step = 1; step < steps; ++step) {
                const double theta = arc.polar_start_theta + angle * double(step) / double(steps);
                out.emplace_back(center.x() + radius * std::cos(theta), center.y() + radius * std::sin(theta));
            }
            out.push_back(to_unscaled(arc.end_point));
        } else {
            for (size_t point_idx = fitting.start_point_index + 1;
                 point_idx <= fitting.end_point_index; ++point_idx)
                out.push_back(to_unscaled(path.polyline.points[point_idx].to_point()));
        }
    }
    return out.size() >= 2 ? out : raw_points();
}

void append_path_group(const std::vector<const ExtrusionPath *> &paths,
                       size_t layer_id,
                       size_t region_id,
                       ExtrusionSource source,
                       const std::vector<size_t> &entity_indices,
                       bool requested_closed,
                       bool use_fitted_arcs,
                       double tolerance,
                       std::vector<PathChain> &out)
{
    PathChain chain;
    chain.layer_id = layer_id;
    Vec2d previous_end = Vec2d::Zero();
    bool have_previous = false;

    auto flush = [&]() {
        if (!chain.leaves.empty()) {
            chain.closed = requested_closed && points_near(chain.leaves.front().points.front(),
                                                            chain.leaves.back().points.back());
            out.push_back(std::move(chain));
            chain = PathChain{};
            chain.layer_id = layer_id;
        }
        have_previous = false;
    };

    for (size_t path_idx = 0; path_idx < paths.size(); ++path_idx) {
        const ExtrusionPath &path = *paths[path_idx];
        if (path.is_force_no_extrusion() || !supported_role(path.role()) || path.width <= 0.) {
            flush();
            continue;
        }
        std::vector<Vec2d> points = tessellated_points_impl(path, use_fitted_arcs, tolerance);
        if (points.size() < 2) {
            flush();
            continue;
        }
        if (have_previous && !points_near(previous_end, points.front()))
            flush();

        PathLeaf leaf;
        leaf.id = { layer_id, region_id, source, entity_indices, path_idx };
        leaf.role = path.role();
        leaf.width = path.width;
        leaf.points = std::move(points);
        previous_end = leaf.points.back();
        have_previous = true;
        chain.leaves.push_back(std::move(leaf));
    }
    flush();
}

void append_entity(const ExtrusionEntity &entity,
                   size_t layer_id,
                   size_t region_id,
                   ExtrusionSource source,
                   std::vector<size_t> &entity_indices,
                   bool use_fitted_arcs,
                   double tolerance,
                   std::vector<PathChain> &out)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (size_t entity_idx = 0; entity_idx < collection->entities.size(); ++entity_idx) {
            entity_indices.push_back(entity_idx);
            append_entity(*collection->entities[entity_idx], layer_id, region_id, source,
                          entity_indices, use_fitted_arcs, tolerance, out);
            entity_indices.pop_back();
        }
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        std::vector<const ExtrusionPath *> paths;
        for (const ExtrusionPath &path : loop->paths) paths.push_back(&path);
        append_path_group(paths, layer_id, region_id, source, entity_indices, true,
                          use_fitted_arcs, tolerance, out);
    } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        std::vector<const ExtrusionPath *> paths;
        for (const ExtrusionPath &path : multipath->paths) paths.push_back(&path);
        append_path_group(paths, layer_id, region_id, source, entity_indices, false,
                          use_fitted_arcs, tolerance, out);
    } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        append_path_group({ path }, layer_id, region_id, source, entity_indices, false,
                          use_fitted_arcs, tolerance, out);
    }
}

std::vector<PathChain> extract_paths(const PrintObject &object, double tolerance)
{
    std::vector<PathChain> out;
    const bool use_fitted_arcs = object.print()->config().enable_arc_fitting.value &&
                                 !object.print()->config().spiral_mode.value;
    for (const Layer *layer : object.layers()) {
        // Retain empty model layers so lower-track lookup means the actual
        // immediately preceding model layer, never the previous non-empty one.
        out.push_back(PathChain{ layer->id(), false, {} });
        for (size_t region_idx = 0; region_idx < layer->region_count(); ++region_idx) {
            const LayerRegion *region = layer->get_region(int(region_idx));
            for (const auto &[collection, source] : {
                     std::pair<const ExtrusionEntityCollection *, ExtrusionSource>{ &region->perimeters, ExtrusionSource::Perimeter },
                     std::pair<const ExtrusionEntityCollection *, ExtrusionSource>{ &region->fills, ExtrusionSource::Fill } }) {
                std::vector<size_t> indices;
                append_entity(*collection, layer->id(), region_idx, source, indices,
                              use_fitted_arcs, tolerance, out);
            }
        }
    }
    return out;
}

} // namespace

std::string ExtrusionId::to_string() const
{
    std::ostringstream out;
    out << 'L' << layer_id << ":R" << region_id << ':' << extrusion_source_name(source) << ":E";
    if (entity_indices.empty())
        out << '-';
    else
        for (size_t i = 0; i < entity_indices.size(); ++i) {
            if (i != 0) out << '.';
            out << entity_indices[i];
        }
    out << ":P" << leaf_path_index;
    return out.str();
}

bool ExtrusionId::operator==(const ExtrusionId &rhs) const
{
    return std::tie(layer_id, region_id, source, entity_indices, leaf_path_index) ==
           std::tie(rhs.layer_id, rhs.region_id, rhs.source, rhs.entity_indices, rhs.leaf_path_index);
}

bool ExtrusionId::operator<(const ExtrusionId &rhs) const
{
    return std::tie(layer_id, region_id, source, entity_indices, leaf_path_index) <
           std::tie(rhs.layer_id, rhs.region_id, rhs.source, rhs.entity_indices, rhs.leaf_path_index);
}

const char *extrusion_source_name(ExtrusionSource source)
{
    return source == ExtrusionSource::Perimeter ? "perimeter" : "fill";
}

const char *anchor_type_name(AnchorType type)
{
    return type == AnchorType::Pin ? "pin" : "lower_track";
}

const char *extrusion_role_name(ExtrusionRole role)
{
    switch (role) {
    case erPerimeter:            return "internal_perimeter";
    case erExternalPerimeter:    return "external_perimeter";
    case erOverhangPerimeter:    return "overhang_perimeter";
    case erInternalInfill:       return "sparse_infill";
    case erSolidInfill:          return "solid_infill";
    case erTopSolidInfill:       return "top_solid_infill";
    case erBottomSurface:        return "bottom_surface";
    case erIroning:              return "ironing";
    case erBridgeInfill:         return "bridge";
    case erInternalBridgeInfill: return "internal_bridge";
    case erGapFill:              return "gap_fill";
    default:                     return "unsupported";
    }
}

std::vector<Vec2d> tessellate_path(const ExtrusionPath &path, bool use_fitted_arcs, double tolerance)
{
    return tessellated_points_impl(path, use_fitted_arcs, tolerance);
}

std::vector<PathChain> extract_finalized_path_chains(const PrintObject &object)
{
    const double tolerance = std::max(EPSILON, object.print()->config().resolution.value);
    return extract_paths(object, tolerance);
}

std::vector<PathPolyline> clip_path_chain_interval(const PathChain &chain, const PathInterval &interval)
{
    const RuntimeChain runtime = make_runtime_chain(chain, 0);
    if (runtime.segments.empty() || interval.end + geometry_epsilon < interval.begin)
        return {};

    const double chain_length = runtime.length;
    std::vector<PathInterval> pieces;
    if (!chain.closed) {
        const double begin = std::clamp(interval.begin, 0., chain_length);
        const double end   = std::clamp(interval.end,   0., chain_length);
        if (end + geometry_epsilon < begin)
            return {};
        pieces.push_back({ begin, end });
    } else {
        const double requested_length = std::min(chain_length, std::max(0., interval.length()));
        double begin = std::fmod(interval.begin, chain_length);
        if (begin < 0.)
            begin += chain_length;
        const double end = begin + requested_length;
        pieces.push_back({ begin, std::min(end, chain_length) });
        if (end > chain_length + geometry_epsilon)
            pieces.push_back({ 0., end - chain_length });
    }

    std::vector<PathPolyline> out;
    auto append_point = [&out](const Vec2d &point) {
        if (out.empty())
            out.emplace_back();
        if (out.back().empty() || !points_near(out.back().back(), point))
            out.back().push_back(point);
    };

    for (const PathInterval &piece : pieces) {
        bool found_segment = false;
        for (const Segment &segment : runtime.segments) {
            const double segment_end = segment.s_begin + segment.length;
            if (piece.end < segment.s_begin - geometry_epsilon ||
                piece.begin > segment_end + geometry_epsilon)
                continue;

            const double begin = std::clamp((piece.begin - segment.s_begin) / segment.length, 0., 1.);
            const double end   = std::clamp((piece.end   - segment.s_begin) / segment.length, 0., 1.);
            if (end + geometry_epsilon < begin)
                continue;

            const Vec2d first = segment.line.a + begin * segment.line.vector();
            const Vec2d last  = segment.line.a + end   * segment.line.vector();
            if (!out.empty() && !out.back().empty() && !points_near(out.back().back(), first))
                out.emplace_back();
            append_point(first);
            append_point(last);
            found_segment = true;
        }
        if (!found_segment && piece.begin >= chain_length - geometry_epsilon)
            append_point(runtime.segments.back().line.b);
    }

    out.erase(std::remove_if(out.begin(), out.end(), [](const PathPolyline &polyline) {
        return polyline.empty();
    }), out.end());
    return out;
}

std::vector<Contact> associate(const std::vector<Pin>       &pins,
                               const std::vector<PathChain> &path_chains,
                               double                        tolerance)
{
    tolerance = std::max(tolerance, EPSILON);
    std::vector<RuntimeChain> chains;
    chains.reserve(path_chains.size());
    for (size_t chain_idx = 0; chain_idx < path_chains.size(); ++chain_idx)
        chains.push_back(make_runtime_chain(path_chains[chain_idx], chain_idx));
    const std::map<size_t, LayerIndex> layer_indices = make_layer_indices(chains);

    std::vector<Contact> out;
    out.reserve(pins.size());
    // Nominal intervals are kept at chain scope so a pin crossing a role/width
    // boundary still has one physical footprint for span measurement.
    std::vector<std::map<size_t, std::vector<PathInterval>>> pin_intervals(pins.size());

    for (size_t pin_idx = 0; pin_idx < pins.size(); ++pin_idx) {
        const Pin &pin = pins[pin_idx];
        Contact result;
        result.contact_id = pin.contact_id;
        result.supported_layer_id = pin.supported_layer_id;
        result.supported_layer_print_z = pin.supported_layer_print_z;
        result.model_contact_z = pin.model_contact_z;
        result.support_tip_z = pin.support_tip_z;
        result.center = pin.center;
        result.radius = pin.radius;
        result.nominal_clearance = pin.model_contact_z - pin.support_tip_z;
        result.association_tolerance = tolerance;

        auto layer_it = layer_indices.find(pin.supported_layer_id);
        if (layer_it == layer_indices.end()) {
            out.push_back(std::move(result));
            continue;
        }
        const LayerIndex &layer_index = layer_it->second;
        const double query_radius = pin.radius + 0.5 * layer_index.maximum_width + tolerance;
        std::vector<size_t> candidate_indices = AABBTreeLines::all_lines_in_radius(
            layer_index.lines, layer_index.tree, pin.center, query_radius * query_radius);
        std::sort(candidate_indices.begin(), candidate_indices.end());
        candidate_indices.erase(std::unique(candidate_indices.begin(), candidate_indices.end()), candidate_indices.end());

        std::map<std::pair<size_t, size_t>, std::vector<const Segment *>> candidates;
        for (size_t candidate_idx : candidate_indices) {
            const Segment &segment = *layer_index.segments[candidate_idx];
            Vec2d closest;
            const double distance = std::sqrt(line_alg::distance_to_squared(segment.line, pin.center, &closest));
            if (distance <= pin.radius + 0.5 * segment.width + tolerance + geometry_epsilon)
                candidates[{ segment.chain_idx, segment.leaf_idx }].push_back(&segment);
            if (auto interval = circle_interval(segment.line, pin.center, pin.radius + 0.5 * segment.width)) {
                pin_intervals[pin_idx][segment.chain_idx].push_back({
                    segment.s_begin + interval->begin * segment.length,
                    segment.s_begin + interval->end   * segment.length
                });
            }
        }

        for (auto &[key, leaf_candidates] : candidates) {
            const size_t chain_idx = key.first;
            const size_t leaf_idx = key.second;
            const RuntimeChain &chain = chains[chain_idx];
            const PathLeaf &leaf = chain.source->leaves[leaf_idx];
            std::vector<PathInterval> intervals;
            for (const Segment *segment : leaf_candidates) {
                if (auto interval = circle_interval(segment->line, pin.center, pin.radius + 0.5 * segment->width)) {
                    intervals.push_back({ segment->s_begin + interval->begin * segment->length,
                                          segment->s_begin + interval->end   * segment->length });
                }
            }
            intervals = merge_intervals(std::move(intervals));
            if (intervals.empty()) {
                const ClosestLocation closest = closest_location(chain, pin.center, leaf_idx, std::nullopt);
                const auto [tangent, ambiguous] = tangent_at(chain, closest);
                result.associations.push_back({ leaf.id, leaf.role, leaf.width, closest.point, closest.distance,
                                                0., std::nullopt, tangent, ambiguous,
                                                closest.distance <= tolerance + geometry_epsilon, std::nullopt });
            } else {
                for (const PathInterval &interval : intervals) {
                    const ClosestLocation closest = closest_location(chain, pin.center, leaf_idx, interval);
                    const auto [tangent, ambiguous] = tangent_at(chain, closest);
                    result.associations.push_back({ leaf.id, leaf.role, leaf.width, closest.point, closest.distance,
                                                    interval.length(), interval, tangent, ambiguous,
                                                    closest.distance <= tolerance + geometry_epsilon, std::nullopt });
                }
            }
        }
        std::sort(result.associations.begin(), result.associations.end(), [](const ExtrusionAssociation &lhs,
                                                                            const ExtrusionAssociation &rhs) {
            if (lhs.extrusion_id != rhs.extrusion_id) return lhs.extrusion_id < rhs.extrusion_id;
            return lhs.path_arclength_interval.value_or(PathInterval{}).begin <
                   rhs.path_arclength_interval.value_or(PathInterval{}).begin;
        });
        out.push_back(std::move(result));
    }

    for (auto &by_chain : pin_intervals)
        for (auto &entry : by_chain)
            entry.second = merge_intervals(std::move(entry.second));

    // Build every allowed planned-material anchor on each current chain.
    std::vector<std::vector<PlannedMaterialAnchor>> chain_anchors(chains.size());
    for (size_t pin_idx = 0; pin_idx < pins.size(); ++pin_idx)
        for (const auto &[chain_idx, intervals] : pin_intervals[pin_idx])
            for (const PathInterval &interval : intervals)
                chain_anchors[chain_idx].push_back({ AnchorType::Pin, pins[pin_idx].contact_id,
                                                     std::nullopt, interval });

    std::vector<size_t> layer_ids;
    for (const auto &[layer_id, index] : layer_indices) layer_ids.push_back(layer_id);
    for (size_t chain_idx = 0; chain_idx < chains.size(); ++chain_idx) {
        const RuntimeChain &current = chains[chain_idx];
        const auto current_layer_it = std::lower_bound(layer_ids.begin(), layer_ids.end(), current.source->layer_id);
        if (current_layer_it == layer_ids.begin() || current_layer_it == layer_ids.end())
            continue;
        const size_t lower_layer_id = *(current_layer_it - 1);
        const LayerIndex &lower_index = layer_indices.at(lower_layer_id);
        if (lower_index.tree.empty())
            continue;

        std::map<ExtrusionId, std::vector<PathInterval>> lower_intervals;
        for (const Segment &segment : current.segments) {
            const double search_radius = 0.5 * (segment.width + lower_index.maximum_width);
            typename AABBTreeIndirect::Tree<2, double>::BoundingBox bbox(segment.line.a, segment.line.a);
            bbox.extend(segment.line.b);
            bbox.min() -= Vec2d::Constant(search_radius);
            bbox.max() += Vec2d::Constant(search_radius);
            std::vector<size_t> candidates;
            query_bbox(lower_index.tree, size_t(0), bbox, candidates);
            std::sort(candidates.begin(), candidates.end());
            candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
            for (size_t candidate_idx : candidates) {
                const Segment &lower = *lower_index.segments[candidate_idx];
                const double radius = 0.5 * (segment.width + lower.width);
                if (auto interval = capsule_interval(segment.line, lower.line, radius)) {
                    const PathLeaf &lower_leaf = chains[lower.chain_idx].source->leaves[lower.leaf_idx];
                    lower_intervals[lower_leaf.id].push_back({
                        segment.s_begin + interval->begin * segment.length,
                        segment.s_begin + interval->end   * segment.length
                    });
                }
            }
        }
        for (auto &[lower_id, intervals] : lower_intervals)
            for (const PathInterval &interval : merge_intervals(std::move(intervals)))
                chain_anchors[chain_idx].push_back({ AnchorType::LowerTrack, 0, lower_id, interval });
    }

    std::map<ExtrusionId, size_t> leaf_to_chain;
    for (size_t chain_idx = 0; chain_idx < chains.size(); ++chain_idx)
        for (const PathLeaf &leaf : chains[chain_idx].source->leaves)
            leaf_to_chain.emplace(leaf.id, chain_idx);
    std::unordered_map<size_t, size_t> pin_index_by_id;
    for (size_t pin_idx = 0; pin_idx < pins.size(); ++pin_idx)
        pin_index_by_id.emplace(pins[pin_idx].contact_id, pin_idx);

    for (Contact &contact : out) {
        const size_t pin_idx = pin_index_by_id.at(contact.contact_id);
        for (ExtrusionAssociation &association : contact.associations) {
            if (!association.path_arclength_interval)
                continue;
            const size_t chain_idx = leaf_to_chain.at(association.extrusion_id);
            const RuntimeChain &chain = chains[chain_idx];
            const auto chain_intervals_it = pin_intervals[pin_idx].find(chain_idx);
            if (chain_intervals_it == pin_intervals[pin_idx].end())
                continue;
            const PathInterval *physical_pin_interval = nullptr;
            for (const PathInterval &interval : chain_intervals_it->second)
                if (interval_overlaps(interval, *association.path_arclength_interval)) {
                    physical_pin_interval = &interval;
                    break;
                }
            if (physical_pin_interval == nullptr)
                continue;

            PlannedMaterialUnsupportedSpan span;
            span.forward = directional_span(*physical_pin_interval, contact.contact_id,
                                            chain_anchors[chain_idx], chain.length,
                                            chain.source->closed, true);
            span.backward = directional_span(*physical_pin_interval, contact.contact_id,
                                             chain_anchors[chain_idx], chain.length,
                                             chain.source->closed, false);
            association.planned_material_unsupported_span = std::move(span);
        }
    }

    return out;
}

std::vector<Contact> build(const PrintObject &object)
{
    if (!object.config().tree_support_round_tip.value)
        return {};

    const double tolerance = std::max(EPSILON, object.print()->config().resolution.value);
    std::vector<Pin> pins;
    pins.reserve(object.support_contacts().size());
    for (size_t contact_id = 0; contact_id < object.support_contacts().size(); ++contact_id) {
        const SupportContact &contact = object.support_contacts()[contact_id];
        const Layer *supported_layer = nullptr;
        for (const Layer *layer : object.layers())
            if (layer->id() == contact.object_layer_id) {
                supported_layer = layer;
                break;
            }
        pins.push_back({ contact_id,
                         contact.object_layer_id,
                         supported_layer == nullptr ? 0. : supported_layer->print_z,
                         contact.model_contact_z,
                         contact.support_tip_z,
                         to_unscaled(contact.position),
                         unscale<double>(contact.nominal_radius) });
    }
    return associate(pins, extract_finalized_path_chains(object), tolerance);
}

} // namespace Slic3r::OrganicSupport

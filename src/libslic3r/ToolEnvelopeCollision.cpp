#include "ToolEnvelopeCollision.hpp"

#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "Polyline.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace Slic3r {
namespace {

[[noreturn]] void invalid(const std::string &field, const std::string &reason)
{
    throw InvalidArgument("ToolEnvelopeCollision." + field + ": " + reason);
}

void validate_pose(const Vec3d &pose, const std::string &field)
{
    if (!std::isfinite(pose.x()))
        invalid(field + ".x", "must be finite");
    if (!std::isfinite(pose.y()))
        invalid(field + ".y", "must be finite");
    if (!std::isfinite(pose.z()))
        invalid(field + ".z", "must be finite");

    const double limit = static_cast<double>(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR;
    if (pose.x() <= -limit || pose.x() >= limit)
        invalid(field + ".x", "is outside the scaled-coordinate range");
    if (pose.y() <= -limit || pose.y() >= limit)
        invalid(field + ".y", "is outside the scaled-coordinate range");
}

void validate_scaled_coordinate(double value, const std::string &field)
{
    const double limit = static_cast<double>(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR;
    if (!std::isfinite(value) || value <= -limit || value >= limit)
        invalid(field, "is outside the scaled-coordinate range");
}

void add_stats(PrintedSceneQueryStats &target, const PrintedSceneQueryStats &source)
{
    target.visited_cells += source.visited_cells;
    target.candidate_count += source.candidate_count;
    target.exact_count += source.exact_count;
}

bool vertical_intersects(double first_min, double first_max, double second_min, double second_max)
{
    return first_min <= second_max + EPSILON && first_max + EPSILON >= second_min;
}

double signed_vertical_clearance(double first_min,
                                 double first_max,
                                 double second_min,
                                 double second_max)
{
    if (first_min > second_max)
        return first_min - second_max;
    if (second_min > first_max)
        return second_min - first_max;
    return -std::max(0.0, std::min(first_max, second_max) - std::max(first_min, second_min));
}

ExPolygons translated_geometry(const ToolEnvelopeSlice &slice, double x_mm, double y_mm)
{
    ExPolygons result = slice.geometry();
    const Point translation = Point::new_scale(x_mm, y_mm);
    for (ExPolygon &polygon : result)
        polygon.translate(translation);
    return result;
}

Polygons obstacle_capsule(const DepositedSegment &segment, double margin_mm)
{
    const double radius_mm = segment.width_mm * 0.5 + margin_mm + EPSILON;
    validate_scaled_coordinate(std::min(segment.start_mm.x(), segment.end_mm.x()) - radius_mm,
                               "obstacle.bounds.min_x_mm");
    validate_scaled_coordinate(std::min(segment.start_mm.y(), segment.end_mm.y()) - radius_mm,
                               "obstacle.bounds.min_y_mm");
    validate_scaled_coordinate(std::max(segment.start_mm.x(), segment.end_mm.x()) + radius_mm,
                               "obstacle.bounds.max_x_mm");
    validate_scaled_coordinate(std::max(segment.start_mm.y(), segment.end_mm.y()) + radius_mm,
                               "obstacle.bounds.max_y_mm");

    Point start = Point::new_scale(segment.start_mm.x(), segment.start_mm.y());
    Point end   = Point::new_scale(segment.end_mm.x(), segment.end_mm.y());
    if (start == end) {
        // Clipper ignores a fully degenerate open path. One scaled coordinate is
        // well below the collision tolerance and lets its round caps represent
        // the required circular footprint conservatively.
        end.x() += 1;
    }
    return offset(Polyline(start, end),
                  static_cast<float>(scale_(radius_mm)),
                  ClipperLib::jtRound,
                  2.0,
                  ClipperLib::etOpenRound);
}

bool xy_intersects(const ExPolygons &tool_geometry, const Polygons &capsule)
{
    return !capsule.empty() && !intersection_ex(tool_geometry, capsule).empty();
}

std::string diagnostic_for(const ToolClearanceResult &result)
{
    std::ostringstream message;
    message << "tool envelope slice " << result.slice_index;
    if (!result.slice_name.empty())
        message << " ('" << result.slice_name << "')";
    message << " in zone " << tool_envelope_zone_name(result.zone)
            << " collides with deposited primitive " << result.primitive_id
            << " (sequence " << result.obstacle_sequence << ") at t="
            << result.first_hit_t;
    return message.str();
}

bool hit_less(const ToolClearanceResult &left, const ToolClearanceResult &right)
{
    if (left.first_hit_t != right.first_hit_t)
        return left.first_hit_t < right.first_hit_t;
    if (left.obstacle_sequence != right.obstacle_sequence)
        return left.obstacle_sequence < right.obstacle_sequence;
    if (left.primitive_id != right.primitive_id)
        return left.primitive_id < right.primitive_id;
    if (left.slice_index != right.slice_index)
        return left.slice_index < right.slice_index;
    return left.obstacle_index < right.obstacle_index;
}

void merge_obstacle_hit(std::unordered_map<std::size_t, ToolClearanceResult> &hits,
                        ToolClearanceResult hit)
{
    const auto found = hits.find(hit.obstacle_index);
    if (found == hits.end()) {
        hits.emplace(hit.obstacle_index, std::move(hit));
        return;
    }

    ToolClearanceResult &current = found->second;
    const double required_raise = std::max(current.required_z_raise_mm, hit.required_z_raise_mm);
    const double last_hit = std::max(current.last_hit_t, hit.last_hit_t);
    if (hit_less(hit, current))
        current = std::move(hit);
    current.required_z_raise_mm = required_raise;
    current.last_hit_t = last_hit;
}

std::vector<ToolClearanceResult> collect_sorted_hits(
    std::unordered_map<std::size_t, ToolClearanceResult> hits,
    const PrintedSceneQueryStats &stats)
{
    std::vector<ToolClearanceResult> result;
    result.reserve(hits.size());
    for (auto &item : hits) {
        item.second.query_stats = stats;
        item.second.diagnostic = diagnostic_for(item.second);
        result.emplace_back(std::move(item.second));
    }
    std::sort(result.begin(), result.end(), hit_less);
    return result;
}

struct SweepHitAccumulator {
    bool   hit { false };
    double first_t { 1.0 };
    double last_t { 0.0 };
    double signed_clearance { std::numeric_limits<double>::infinity() };
    double required_raise { 0.0 };
};

bool interval_xy_possible(const ToolEnvelopeSlice &slice,
                          const Polygons &capsule,
                          const Vec3d &start_pose,
                          const Vec3d &end_pose,
                          double t0,
                          double t1)
{
    const double midpoint_t = (t0 + t1) * 0.5;
    const Vec3d midpoint = start_pose + (end_pose - start_pose) * midpoint_t;
    ExPolygons enclosure = translated_geometry(slice, midpoint.x(), midpoint.y());

    const Vec2d interval_travel =
        ((end_pose - start_pose) * (t1 - t0)).head<2>();
    const double enclosure_radius_mm = interval_travel.norm() * 0.5;
    if (enclosure_radius_mm > 0.0)
        enclosure = offset_ex(enclosure,
                              static_cast<float>(scale_(enclosure_radius_mm)),
                              ClipperLib::jtRound,
                              2.0);
    return xy_intersects(enclosure, capsule);
}

SweepHitAccumulator sweep_slice_against_obstacle(const ToolEnvelopeSlice &slice,
                                                 const DepositedSegment &obstacle,
                                                 const PrintedScene::PrimitiveBounds &obstacle_bounds,
                                                 const Vec3d &start_pose,
                                                 const Vec3d &end_pose,
                                                 double margin_mm)
{
    const Polygons capsule = obstacle_capsule(obstacle, margin_mm);
    SweepHitAccumulator accumulator;
    const Vec3d delta = end_pose - start_pose;
    const double xy_distance = delta.head<2>().norm();
    const double z_distance = std::abs(delta.z());

    std::function<void(double, double, unsigned)> visit;
    visit = [&](double t0, double t1, unsigned depth) {
        const double z0 = start_pose.z() + delta.z() * t0;
        const double z1 = start_pose.z() + delta.z() * t1;
        const double slice_z_min = std::min(z0, z1) + slice.z_min_mm();
        const double slice_z_max = std::max(z0, z1) + slice.z_max_mm();
        if (!vertical_intersects(slice_z_min, slice_z_max,
                                 obstacle_bounds.z_min_mm, obstacle_bounds.z_max_mm))
            return;
        if (!interval_xy_possible(slice, capsule, start_pose, end_pose, t0, t1))
            return;

        const double interval = t1 - t0;
        const bool resolved = xy_distance * interval <= RESOLUTION &&
                              z_distance * interval <= EPSILON;
        if (resolved || depth >= 32) {
            const double clearance = signed_vertical_clearance(
                slice_z_min, slice_z_max, obstacle_bounds.z_min_mm, obstacle_bounds.z_max_mm);
            const double required_raise = std::max(
                0.0, obstacle_bounds.z_max_mm - slice_z_min + EPSILON);
            if (!accumulator.hit) {
                accumulator.hit = true;
                accumulator.first_t = t0;
                accumulator.signed_clearance = clearance;
            }
            accumulator.last_t = std::max(accumulator.last_t, t1);
            accumulator.required_raise = std::max(accumulator.required_raise, required_raise);
            return;
        }

        const double midpoint = (t0 + t1) * 0.5;
        visit(t0, midpoint, depth + 1);
        visit(midpoint, t1, depth + 1);
    };

    visit(0.0, 1.0, 0);
    return accumulator;
}

} // namespace

bool ToolContactPolicy::checks(ToolEnvelopeZone zone) const noexcept
{
    switch (zone) {
    case ToolEnvelopeZone::Deposition:     return check_deposition;
    case ToolEnvelopeZone::HardCollision: return check_hard_collision;
    case ToolEnvelopeZone::Thermal:        return check_thermal;
    case ToolEnvelopeZone::Toolhead:       return check_toolhead;
    }
    return false;
}

double ToolContactPolicy::margin_mm(ToolEnvelopeZone zone) const noexcept
{
    switch (zone) {
    case ToolEnvelopeZone::Deposition:     return deposition_margin_mm;
    case ToolEnvelopeZone::HardCollision: return hard_collision_margin_mm;
    case ToolEnvelopeZone::Thermal:        return thermal_margin_mm;
    case ToolEnvelopeZone::Toolhead:       return toolhead_margin_mm;
    }
    return 0.0;
}

void ToolContactPolicy::validate() const
{
    const std::pair<const char *, double> margins[] = {
        {"policy.deposition_margin_mm", deposition_margin_mm},
        {"policy.hard_collision_margin_mm", hard_collision_margin_mm},
        {"policy.thermal_margin_mm", thermal_margin_mm},
        {"policy.toolhead_margin_mm", toolhead_margin_mm}
    };
    for (const auto &margin : margins) {
        if (!std::isfinite(margin.second))
            invalid(margin.first, "must be finite");
        if (margin.second < 0.0)
            invalid(margin.first, "must not be negative");
    }
}

std::vector<ToolClearanceResult> check_pose_all(const ToolEnvelope &envelope,
                                                const PrintedScene &scene,
                                                const Vec3d &pose_mm,
                                                const ToolContactPolicy &policy,
                                                PrintedSceneQueryStats *stats)
{
    validate_pose(pose_mm, "pose_mm");
    policy.validate();
    PrintedSceneQueryStats aggregate_stats;
    std::unordered_map<std::size_t, ToolClearanceResult> hits;

    for (std::size_t slice_index = 0; slice_index < envelope.slices().size(); ++slice_index) {
        const ToolEnvelopeSlice &slice = envelope.slices()[slice_index];
        if (!policy.checks(slice.zone()))
            continue;
        const double margin = policy.margin_mm(slice.zone()) + EPSILON;
        const BoundingBoxf &bounds = slice.local_bounds_mm();
        PrintedSceneQueryStats slice_stats;
        const std::vector<std::size_t> candidates = scene.query_indices(
            bounds.min.x() + pose_mm.x() - margin,
            bounds.min.y() + pose_mm.y() - margin,
            bounds.max.x() + pose_mm.x() + margin,
            bounds.max.y() + pose_mm.y() + margin,
            pose_mm.z() + slice.z_min_mm() - EPSILON,
            pose_mm.z() + slice.z_max_mm() + EPSILON,
            &slice_stats);
        add_stats(aggregate_stats, slice_stats);

        const ExPolygons geometry = translated_geometry(slice, pose_mm.x(), pose_mm.y());
        for (const std::size_t obstacle_index : candidates) {
            const DepositedSegment &obstacle = scene.segment(obstacle_index);
            const PrintedScene::PrimitiveBounds &obstacle_bounds = scene.bounds(obstacle_index);
            const double slice_z_min = pose_mm.z() + slice.z_min_mm();
            const double slice_z_max = pose_mm.z() + slice.z_max_mm();
            if (!vertical_intersects(slice_z_min, slice_z_max,
                                     obstacle_bounds.z_min_mm, obstacle_bounds.z_max_mm))
                continue;
            if (!xy_intersects(geometry, obstacle_capsule(obstacle, policy.margin_mm(slice.zone()))))
                continue;

            ToolClearanceResult hit;
            hit.collision = true;
            hit.first_hit_t = 0.0;
            hit.last_hit_t = 0.0;
            hit.primitive_id = obstacle.primitive_id;
            hit.obstacle_sequence = obstacle.sequence_index;
            hit.obstacle_index = obstacle_index;
            hit.slice_index = slice_index;
            hit.zone = slice.zone();
            hit.slice_name = slice.name();
            hit.signed_vertical_clearance_mm = signed_vertical_clearance(
                slice_z_min, slice_z_max, obstacle_bounds.z_min_mm, obstacle_bounds.z_max_mm);
            hit.required_z_raise_mm = std::max(
                0.0, obstacle_bounds.z_max_mm - slice_z_min + EPSILON);
            hit.pose_mm = pose_mm;
            merge_obstacle_hit(hits, std::move(hit));
        }
    }

    if (stats != nullptr)
        *stats = aggregate_stats;
    return collect_sorted_hits(std::move(hits), aggregate_stats);
}

ToolClearanceResult check_pose(const ToolEnvelope &envelope,
                               const PrintedScene &scene,
                               const Vec3d &pose_mm,
                               const ToolContactPolicy &policy)
{
    PrintedSceneQueryStats stats;
    std::vector<ToolClearanceResult> hits = check_pose_all(envelope, scene, pose_mm, policy, &stats);
    if (!hits.empty())
        return std::move(hits.front());
    ToolClearanceResult clear;
    clear.pose_mm = pose_mm;
    clear.query_stats = stats;
    return clear;
}

std::vector<ToolClearanceResult> check_sweep_all(const ToolEnvelope &envelope,
                                                 const PrintedScene &scene,
                                                 const Vec3d &start_pose_mm,
                                                 const Vec3d &end_pose_mm,
                                                 const ToolContactPolicy &policy,
                                                 PrintedSceneQueryStats *stats)
{
    validate_pose(start_pose_mm, "start_pose_mm");
    validate_pose(end_pose_mm, "end_pose_mm");
    policy.validate();
    PrintedSceneQueryStats aggregate_stats;
    std::unordered_map<std::size_t, ToolClearanceResult> hits;

    for (std::size_t slice_index = 0; slice_index < envelope.slices().size(); ++slice_index) {
        const ToolEnvelopeSlice &slice = envelope.slices()[slice_index];
        if (!policy.checks(slice.zone()))
            continue;
        const double margin = policy.margin_mm(slice.zone()) + EPSILON;
        const BoundingBoxf &bounds = slice.local_bounds_mm();
        PrintedSceneQueryStats slice_stats;
        const std::vector<std::size_t> candidates = scene.query_indices(
            bounds.min.x() + std::min(start_pose_mm.x(), end_pose_mm.x()) - margin,
            bounds.min.y() + std::min(start_pose_mm.y(), end_pose_mm.y()) - margin,
            bounds.max.x() + std::max(start_pose_mm.x(), end_pose_mm.x()) + margin,
            bounds.max.y() + std::max(start_pose_mm.y(), end_pose_mm.y()) + margin,
            slice.z_min_mm() + std::min(start_pose_mm.z(), end_pose_mm.z()) - EPSILON,
            slice.z_max_mm() + std::max(start_pose_mm.z(), end_pose_mm.z()) + EPSILON,
            &slice_stats);
        add_stats(aggregate_stats, slice_stats);

        for (const std::size_t obstacle_index : candidates) {
            const DepositedSegment &obstacle = scene.segment(obstacle_index);
            const PrintedScene::PrimitiveBounds &obstacle_bounds = scene.bounds(obstacle_index);
            const SweepHitAccumulator accumulator = sweep_slice_against_obstacle(
                slice, obstacle, obstacle_bounds, start_pose_mm, end_pose_mm,
                policy.margin_mm(slice.zone()));
            if (!accumulator.hit)
                continue;

            ToolClearanceResult hit;
            hit.collision = true;
            hit.first_hit_t = accumulator.first_t;
            hit.last_hit_t = accumulator.last_t;
            hit.primitive_id = obstacle.primitive_id;
            hit.obstacle_sequence = obstacle.sequence_index;
            hit.obstacle_index = obstacle_index;
            hit.slice_index = slice_index;
            hit.zone = slice.zone();
            hit.slice_name = slice.name();
            hit.signed_vertical_clearance_mm = accumulator.signed_clearance;
            hit.required_z_raise_mm = accumulator.required_raise;
            hit.pose_mm = start_pose_mm + (end_pose_mm - start_pose_mm) * accumulator.first_t;
            merge_obstacle_hit(hits, std::move(hit));
        }
    }

    if (stats != nullptr)
        *stats = aggregate_stats;
    return collect_sorted_hits(std::move(hits), aggregate_stats);
}

ToolClearanceResult check_sweep(const ToolEnvelope &envelope,
                                const PrintedScene &scene,
                                const Vec3d &start_pose_mm,
                                const Vec3d &end_pose_mm,
                                const ToolContactPolicy &policy)
{
    PrintedSceneQueryStats stats;
    std::vector<ToolClearanceResult> hits = check_sweep_all(
        envelope, scene, start_pose_mm, end_pose_mm, policy, &stats);
    if (!hits.empty())
        return std::move(hits.front());
    ToolClearanceResult clear;
    clear.pose_mm = end_pose_mm;
    clear.query_stats = stats;
    return clear;
}

double required_z_raise_for_sweep(const ToolEnvelope &envelope,
                                  const PrintedScene &scene,
                                  const Vec3d &start_pose_mm,
                                  const Vec3d &end_pose_mm,
                                  const ToolContactPolicy &policy)
{
    const std::vector<ToolClearanceResult> hits = check_sweep_all(
        envelope, scene, start_pose_mm, end_pose_mm, policy);
    double required_raise = 0.0;
    for (const ToolClearanceResult &hit : hits)
        required_raise = std::max(required_raise, hit.required_z_raise_mm);
    return required_raise;
}

} // namespace Slic3r

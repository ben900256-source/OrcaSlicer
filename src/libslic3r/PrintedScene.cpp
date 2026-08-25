#include "PrintedScene.hpp"

#include "Exception.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace Slic3r {
namespace {

[[noreturn]] void invalid(const std::string &field, const std::string &reason)
{
    throw InvalidArgument("PrintedScene." + field + ": " + reason);
}

void require_finite(double value, const std::string &field)
{
    if (!std::isfinite(value))
        invalid(field, "must be finite");
}

void require_grid_coordinate(double value, const std::string &field)
{
    require_finite(value, field);
    const double limit = static_cast<double>(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR;
    if (value <= -limit || value >= limit)
        invalid(field, "is outside the scaled-coordinate range");
}

void validate_point(const Vec3d &point, const std::string &field)
{
    require_finite(point.x(), field + ".x");
    require_finite(point.y(), field + ".y");
    require_finite(point.z(), field + ".z");
}

bool overlaps(double first_min, double first_max, double second_min, double second_max)
{
    return first_min <= second_max && first_max >= second_min;
}

void validate_query(double min_x_mm,
                    double min_y_mm,
                    double max_x_mm,
                    double max_y_mm,
                    double z_min_mm,
                    double z_max_mm)
{
    require_finite(min_x_mm, "query.min_x_mm");
    require_finite(min_y_mm, "query.min_y_mm");
    require_finite(max_x_mm, "query.max_x_mm");
    require_finite(max_y_mm, "query.max_y_mm");
    require_finite(z_min_mm, "query.z_min_mm");
    require_finite(z_max_mm, "query.z_max_mm");
    if (min_x_mm > max_x_mm)
        invalid("query.x_range", "minimum must not exceed maximum");
    if (min_y_mm > max_y_mm)
        invalid("query.y_range", "minimum must not exceed maximum");
    if (z_min_mm > z_max_mm)
        invalid("query.z_range", "minimum must not exceed maximum");
    require_grid_coordinate(min_x_mm, "query.min_x_mm");
    require_grid_coordinate(min_y_mm, "query.min_y_mm");
    require_grid_coordinate(max_x_mm, "query.max_x_mm");
    require_grid_coordinate(max_y_mm, "query.max_y_mm");
}

} // namespace

std::size_t PrintedScene::CellHash::operator()(const Cell &cell) const noexcept
{
    const std::uint64_t x = static_cast<std::uint64_t>(cell.x);
    const std::uint64_t y = static_cast<std::uint64_t>(cell.y);
    std::uint64_t hash = x + 0x9e3779b97f4a7c15ULL;
    hash ^= y + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    return static_cast<std::size_t>(hash);
}

std::int64_t PrintedScene::cell_coordinate(double value_mm)
{
    return static_cast<std::int64_t>(std::floor(value_mm / GRID_CELL_SIZE_MM));
}

PrintedScene::PrimitiveBounds PrintedScene::calculate_bounds(const DepositedSegment &segment)
{
    validate_point(segment.start_mm, "segment.start_mm");
    validate_point(segment.end_mm, "segment.end_mm");
    require_finite(segment.width_mm, "segment.width_mm");
    require_finite(segment.height_mm, "segment.height_mm");
    if (segment.width_mm <= 0.0)
        invalid("segment.width_mm", "must be positive");
    if (segment.height_mm <= 0.0)
        invalid("segment.height_mm", "must be positive");

    const double radius = segment.width_mm * 0.5;
    const PrimitiveBounds bounds {
        std::min(segment.start_mm.x(), segment.end_mm.x()) - radius,
        std::min(segment.start_mm.y(), segment.end_mm.y()) - radius,
        std::max(segment.start_mm.x(), segment.end_mm.x()) + radius,
        std::max(segment.start_mm.y(), segment.end_mm.y()) + radius,
        std::min(segment.start_mm.z() - segment.height_mm,
                 segment.end_mm.z() - segment.height_mm),
        std::max(segment.start_mm.z(), segment.end_mm.z())
    };
    require_grid_coordinate(bounds.min_x_mm, "segment.bounds.min_x_mm");
    require_grid_coordinate(bounds.min_y_mm, "segment.bounds.min_y_mm");
    require_grid_coordinate(bounds.max_x_mm, "segment.bounds.max_x_mm");
    require_grid_coordinate(bounds.max_y_mm, "segment.bounds.max_y_mm");
    require_finite(bounds.z_min_mm, "segment.bounds.z_min_mm");
    require_finite(bounds.z_max_mm, "segment.bounds.z_max_mm");
    return bounds;
}

std::size_t PrintedScene::insert(const DepositedSegment &segment)
{
    const PrimitiveBounds primitive_bounds = calculate_bounds(segment);
    const std::size_t index = m_segments.size();
    m_segments.push_back(segment);
    m_bounds.push_back(primitive_bounds);

    const std::int64_t min_cell_x = cell_coordinate(primitive_bounds.min_x_mm);
    const std::int64_t max_cell_x = cell_coordinate(primitive_bounds.max_x_mm);
    const std::int64_t min_cell_y = cell_coordinate(primitive_bounds.min_y_mm);
    const std::int64_t max_cell_y = cell_coordinate(primitive_bounds.max_y_mm);
    for (std::int64_t y = min_cell_y;; ++y) {
        for (std::int64_t x = min_cell_x;; ++x) {
            m_grid[{x, y}].push_back(index);
            if (x == max_cell_x)
                break;
        }
        if (y == max_cell_y)
            break;
    }
    return index;
}

void PrintedScene::clear()
{
    m_segments.clear();
    m_bounds.clear();
    m_grid.clear();
}

const DepositedSegment &PrintedScene::segment(std::size_t index) const
{
    if (index >= m_segments.size())
        throw OutOfRange("PrintedScene.segment: index is out of range");
    return m_segments[index];
}

const PrintedScene::PrimitiveBounds &PrintedScene::bounds(std::size_t index) const
{
    if (index >= m_bounds.size())
        throw OutOfRange("PrintedScene.bounds: index is out of range");
    return m_bounds[index];
}

std::vector<std::size_t> PrintedScene::query_indices(double min_x_mm,
                                                     double min_y_mm,
                                                     double max_x_mm,
                                                     double max_y_mm,
                                                     double z_min_mm,
                                                     double z_max_mm,
                                                     PrintedSceneQueryStats *stats) const
{
    validate_query(min_x_mm, min_y_mm, max_x_mm, max_y_mm, z_min_mm, z_max_mm);
    PrintedSceneQueryStats local_stats;

    const std::int64_t min_cell_x = cell_coordinate(min_x_mm);
    const std::int64_t max_cell_x = cell_coordinate(max_x_mm);
    const std::int64_t min_cell_y = cell_coordinate(min_y_mm);
    const std::int64_t max_cell_y = cell_coordinate(max_y_mm);
    std::vector<unsigned char> seen(m_segments.size(), 0);
    std::vector<std::size_t> candidates;

    for (std::int64_t y = min_cell_y;; ++y) {
        for (std::int64_t x = min_cell_x;; ++x) {
            ++local_stats.visited_cells;
            const auto found = m_grid.find({x, y});
            if (found != m_grid.end()) {
                for (const std::size_t index : found->second) {
                    if (!seen[index]) {
                        seen[index] = 1;
                        candidates.push_back(index);
                    }
                }
            }
            if (x == max_cell_x)
                break;
        }
        if (y == max_cell_y)
            break;
    }

    std::sort(candidates.begin(), candidates.end());
    local_stats.candidate_count = candidates.size();
    std::vector<std::size_t> result;
    result.reserve(candidates.size());
    for (const std::size_t index : candidates) {
        const PrimitiveBounds &primitive = m_bounds[index];
        if (overlaps(min_x_mm, max_x_mm, primitive.min_x_mm, primitive.max_x_mm) &&
            overlaps(min_y_mm, max_y_mm, primitive.min_y_mm, primitive.max_y_mm) &&
            overlaps(z_min_mm, z_max_mm, primitive.z_min_mm, primitive.z_max_mm))
            result.push_back(index);
    }
    local_stats.exact_count = result.size();
    if (stats != nullptr)
        *stats = local_stats;
    return result;
}

std::vector<std::size_t> PrintedScene::query_indices(const BoundingBoxf &xy_bounds_mm,
                                                     double z_min_mm,
                                                     double z_max_mm,
                                                     PrintedSceneQueryStats *stats) const
{
    return query_indices(xy_bounds_mm.min.x(), xy_bounds_mm.min.y(),
                         xy_bounds_mm.max.x(), xy_bounds_mm.max.y(),
                         z_min_mm, z_max_mm, stats);
}

std::vector<std::size_t> PrintedScene::brute_force_query_indices(double min_x_mm,
                                                                 double min_y_mm,
                                                                 double max_x_mm,
                                                                 double max_y_mm,
                                                                 double z_min_mm,
                                                                 double z_max_mm) const
{
    validate_query(min_x_mm, min_y_mm, max_x_mm, max_y_mm, z_min_mm, z_max_mm);
    std::vector<std::size_t> result;
    for (std::size_t index = 0; index < m_bounds.size(); ++index) {
        const PrimitiveBounds &primitive = m_bounds[index];
        if (overlaps(min_x_mm, max_x_mm, primitive.min_x_mm, primitive.max_x_mm) &&
            overlaps(min_y_mm, max_y_mm, primitive.min_y_mm, primitive.max_y_mm) &&
            overlaps(z_min_mm, z_max_mm, primitive.z_min_mm, primitive.z_max_mm))
            result.push_back(index);
    }
    return result;
}

} // namespace Slic3r

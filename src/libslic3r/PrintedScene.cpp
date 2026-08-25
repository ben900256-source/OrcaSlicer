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
    const std::uint64_t z = static_cast<std::uint64_t>(cell.z);
    std::uint64_t hash = x + 0x9e3779b97f4a7c15ULL;
    hash ^= y + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    hash ^= z + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    return static_cast<std::size_t>(hash);
}

std::int64_t PrintedScene::xy_cell_coordinate(double value_mm)
{
    return static_cast<std::int64_t>(std::floor(value_mm / GRID_CELL_SIZE_MM));
}

std::int64_t PrintedScene::z_cell_coordinate(double value_mm)
{
    return static_cast<std::int64_t>(std::floor(value_mm / GRID_Z_SLAB_SIZE_MM));
}

std::vector<PrintedScene::Cell> PrintedScene::centerline_cells(const Vec3d &start, const Vec3d &end)
{
    Cell cell { xy_cell_coordinate(start.x()), xy_cell_coordinate(start.y()), z_cell_coordinate(start.z()) };
    const Cell last { xy_cell_coordinate(end.x()), xy_cell_coordinate(end.y()), z_cell_coordinate(end.z()) };
    std::vector<Cell> cells { cell };
    if (cell == last)
        return cells;

    const Vec3d delta = end - start;
    const int step[3] = { (delta.x() > 0.0) - (delta.x() < 0.0),
                          (delta.y() > 0.0) - (delta.y() < 0.0),
                          (delta.z() > 0.0) - (delta.z() < 0.0) };
    const double sizes[3] = { GRID_CELL_SIZE_MM, GRID_CELL_SIZE_MM, GRID_Z_SLAB_SIZE_MM };
    double t_max[3];
    double t_delta[3];
    for (int axis = 0; axis < 3; ++axis) {
        const double d = delta[axis];
        if (step[axis] == 0) {
            t_max[axis] = std::numeric_limits<double>::infinity();
            t_delta[axis] = std::numeric_limits<double>::infinity();
            continue;
        }
        const std::int64_t current = axis == 0 ? cell.x : axis == 1 ? cell.y : cell.z;
        const double boundary = static_cast<double>(current + (step[axis] > 0 ? 1 : 0)) * sizes[axis];
        t_max[axis] = (boundary - start[axis]) / d;
        t_delta[axis] = sizes[axis] / std::abs(d);
    }

    while (!(cell == last)) {
        const double next_t = std::min({t_max[0], t_max[1], t_max[2]});
        const double tie_tolerance = std::numeric_limits<double>::epsilon() * 8.0;
        if (t_max[0] <= next_t + tie_tolerance) {
            cell.x += step[0];
            t_max[0] += t_delta[0];
        }
        if (t_max[1] <= next_t + tie_tolerance) {
            cell.y += step[1];
            t_max[1] += t_delta[1];
        }
        if (t_max[2] <= next_t + tie_tolerance) {
            cell.z += step[2];
            t_max[2] += t_delta[2];
        }
        cells.push_back(cell);
    }
    return cells;
}

void PrintedScene::append_padded_cells(std::vector<Cell> &target,
                                       const std::vector<Cell> &centerline,
                                       std::int64_t min_dx,
                                       std::int64_t max_dx,
                                       std::int64_t min_dy,
                                       std::int64_t max_dy,
                                       std::int64_t min_dz,
                                       std::int64_t max_dz)
{
    for (const Cell &cell : centerline)
        for (std::int64_t dz = min_dz; dz <= max_dz; ++dz)
            for (std::int64_t dy = min_dy; dy <= max_dy; ++dy)
                for (std::int64_t dx = min_dx; dx <= max_dx; ++dx)
                    target.push_back({cell.x + dx, cell.y + dy, cell.z + dz});
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

    const std::int64_t xy_pad = static_cast<std::int64_t>(
        std::ceil(segment.width_mm * 0.5 / GRID_CELL_SIZE_MM)) + 1;
    const std::int64_t z_below = static_cast<std::int64_t>(
        std::ceil(segment.height_mm / GRID_Z_SLAB_SIZE_MM)) + 1;
    std::vector<Cell> cells;
    append_padded_cells(cells, centerline_cells(segment.start_mm, segment.end_mm),
                        -xy_pad, xy_pad, -xy_pad, xy_pad, -z_below, 1);
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    for (const Cell &cell : cells)
        m_grid[cell].push_back(index);
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
    std::vector<Cell> cells;
    const std::int64_t min_cell_x = xy_cell_coordinate(min_x_mm);
    const std::int64_t max_cell_x = xy_cell_coordinate(max_x_mm);
    const std::int64_t min_cell_y = xy_cell_coordinate(min_y_mm);
    const std::int64_t max_cell_y = xy_cell_coordinate(max_y_mm);
    const std::int64_t min_cell_z = z_cell_coordinate(z_min_mm);
    const std::int64_t max_cell_z = z_cell_coordinate(z_max_mm);
    for (std::int64_t z = min_cell_z;; ++z) {
        for (std::int64_t y = min_cell_y;; ++y) {
            for (std::int64_t x = min_cell_x;; ++x) {
                cells.push_back({x, y, z});
                if (x == max_cell_x) break;
            }
            if (y == max_cell_y) break;
        }
        if (z == max_cell_z) break;
    }
    return query_cells(std::move(cells), min_x_mm, min_y_mm, max_x_mm, max_y_mm,
                       z_min_mm, z_max_mm, stats);
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

std::vector<std::size_t> PrintedScene::query_sweep_indices(
    const Vec3d &start_pose_mm,
    const Vec3d &end_pose_mm,
    double local_min_x_mm,
    double local_min_y_mm,
    double local_max_x_mm,
    double local_max_y_mm,
    double local_z_min_mm,
    double local_z_max_mm,
    PrintedSceneQueryStats *stats) const
{
    validate_point(start_pose_mm, "query.start_pose_mm");
    validate_point(end_pose_mm, "query.end_pose_mm");
    validate_query(local_min_x_mm, local_min_y_mm, local_max_x_mm, local_max_y_mm,
                   local_z_min_mm, local_z_max_mm);

    const std::int64_t min_dx = static_cast<std::int64_t>(std::floor(local_min_x_mm / GRID_CELL_SIZE_MM)) - 1;
    const std::int64_t max_dx = static_cast<std::int64_t>(std::ceil(local_max_x_mm / GRID_CELL_SIZE_MM)) + 1;
    const std::int64_t min_dy = static_cast<std::int64_t>(std::floor(local_min_y_mm / GRID_CELL_SIZE_MM)) - 1;
    const std::int64_t max_dy = static_cast<std::int64_t>(std::ceil(local_max_y_mm / GRID_CELL_SIZE_MM)) + 1;
    const std::int64_t min_dz = static_cast<std::int64_t>(std::floor(local_z_min_mm / GRID_Z_SLAB_SIZE_MM)) - 1;
    const std::int64_t max_dz = static_cast<std::int64_t>(std::ceil(local_z_max_mm / GRID_Z_SLAB_SIZE_MM)) + 1;
    std::vector<Cell> cells;
    append_padded_cells(cells, centerline_cells(start_pose_mm, end_pose_mm),
                        min_dx, max_dx, min_dy, max_dy, min_dz, max_dz);

    const double min_x = std::min(start_pose_mm.x(), end_pose_mm.x()) + local_min_x_mm;
    const double min_y = std::min(start_pose_mm.y(), end_pose_mm.y()) + local_min_y_mm;
    const double max_x = std::max(start_pose_mm.x(), end_pose_mm.x()) + local_max_x_mm;
    const double max_y = std::max(start_pose_mm.y(), end_pose_mm.y()) + local_max_y_mm;
    const double min_z = std::min(start_pose_mm.z(), end_pose_mm.z()) + local_z_min_mm;
    const double max_z = std::max(start_pose_mm.z(), end_pose_mm.z()) + local_z_max_mm;
    return query_cells(std::move(cells), min_x, min_y, max_x, max_y, min_z, max_z, stats);
}

std::vector<std::size_t> PrintedScene::query_cells(std::vector<Cell> cells,
                                                   double min_x_mm,
                                                   double min_y_mm,
                                                   double max_x_mm,
                                                   double max_y_mm,
                                                   double z_min_mm,
                                                   double z_max_mm,
                                                   PrintedSceneQueryStats *stats) const
{
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    PrintedSceneQueryStats local_stats;
    local_stats.visited_cells = cells.size();
    std::vector<std::size_t> references;
    for (const Cell &cell : cells) {
        const auto found = m_grid.find(cell);
        if (found == m_grid.end())
            continue;
        local_stats.references_examined += found->second.size();
        references.insert(references.end(), found->second.begin(), found->second.end());
    }
    std::sort(references.begin(), references.end());
    references.erase(std::unique(references.begin(), references.end()), references.end());
    local_stats.candidate_count = references.size();

    std::vector<std::size_t> result;
    result.reserve(references.size());
    for (const std::size_t index : references) {
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

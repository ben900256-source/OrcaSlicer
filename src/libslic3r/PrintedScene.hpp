#pragma once

#include "BoundingBox.hpp"
#include "Point.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace Slic3r {

struct DepositedSegment {
    Vec3d         start_mm { Vec3d::Zero() };
    Vec3d         end_mm { Vec3d::Zero() };
    double        width_mm { 0.0 };
    double        height_mm { 0.0 };
    std::uint64_t primitive_id { 0 };
    std::size_t   sequence_index { 0 };

    DepositedSegment() = default;
    DepositedSegment(const Vec3d &start,
                     const Vec3d &end,
                     double width,
                     double height,
                     std::uint64_t id = 0,
                     std::size_t sequence = 0)
        : start_mm(start)
        , end_mm(end)
        , width_mm(width)
        , height_mm(height)
        , primitive_id(id)
        , sequence_index(sequence)
    {}
};

struct PrintedSceneQueryStats {
    std::size_t visited_cells { 0 };
    std::size_t references_examined { 0 };
    std::size_t candidate_count { 0 };
    std::size_t exact_count { 0 };
    std::size_t sweep_subdivisions { 0 };
};

using PrintedSceneQueryStatistics = PrintedSceneQueryStats;

class PrintedScene
{
public:
    static constexpr double GRID_CELL_SIZE_MM = 5.0;
    static constexpr double GRID_Z_SLAB_SIZE_MM = 1.0;

    struct PrimitiveBounds {
        double min_x_mm { 0.0 };
        double min_y_mm { 0.0 };
        double max_x_mm { 0.0 };
        double max_y_mm { 0.0 };
        double z_min_mm { 0.0 };
        double z_max_mm { 0.0 };
    };

    std::size_t insert(const DepositedSegment &segment);
    std::size_t insert_segment(const DepositedSegment &segment) { return insert(segment); }
    std::size_t add_segment(const DepositedSegment &segment) { return insert(segment); }

    std::size_t size() const noexcept { return m_segments.size(); }
    bool        empty() const noexcept { return m_segments.empty(); }
    void        clear();

    const DepositedSegment              &segment(std::size_t index) const;
    const std::vector<DepositedSegment> &segments() const noexcept { return m_segments; }
    const PrimitiveBounds               &bounds(std::size_t index) const;

    std::vector<std::size_t> query_indices(double min_x_mm,
                                           double min_y_mm,
                                           double max_x_mm,
                                           double max_y_mm,
                                           double z_min_mm,
                                           double z_max_mm,
                                           PrintedSceneQueryStats *stats = nullptr) const;

    std::vector<std::size_t> query_indices(const BoundingBoxf &xy_bounds_mm,
                                           double z_min_mm,
                                           double z_max_mm,
                                           PrintedSceneQueryStats *stats = nullptr) const;

    // Broad-phase query for a translated volume. Bounds are local to the
    // supplied poses. A sparse corridor keeps long diagonal sweeps linear.
    std::vector<std::size_t> query_sweep_indices(const Vec3d &start_pose_mm,
                                                 const Vec3d &end_pose_mm,
                                                 double local_min_x_mm,
                                                 double local_min_y_mm,
                                                 double local_max_x_mm,
                                                 double local_max_y_mm,
                                                 double local_z_min_mm,
                                                 double local_z_max_mm,
                                                 PrintedSceneQueryStats *stats = nullptr) const;

    std::vector<std::size_t> brute_force_query_indices(double min_x_mm,
                                                       double min_y_mm,
                                                       double max_x_mm,
                                                       double max_y_mm,
                                                       double z_min_mm,
                                                       double z_max_mm) const;

private:
    struct Cell {
        std::int64_t x;
        std::int64_t y;
        std::int64_t z;

        bool operator==(const Cell &other) const noexcept { return x == other.x && y == other.y && z == other.z; }
        bool operator<(const Cell &other) const noexcept
        {
            if (z != other.z) return z < other.z;
            if (y != other.y) return y < other.y;
            return x < other.x;
        }
    };

    struct CellHash {
        std::size_t operator()(const Cell &cell) const noexcept;
    };

    static PrimitiveBounds calculate_bounds(const DepositedSegment &segment);
    static std::int64_t     xy_cell_coordinate(double value_mm);
    static std::int64_t     z_cell_coordinate(double value_mm);
    static std::vector<Cell> centerline_cells(const Vec3d &start, const Vec3d &end);
    static void append_padded_cells(std::vector<Cell> &target,
                                    const std::vector<Cell> &centerline,
                                    std::int64_t min_dx,
                                    std::int64_t max_dx,
                                    std::int64_t min_dy,
                                    std::int64_t max_dy,
                                    std::int64_t min_dz,
                                    std::int64_t max_dz);

    std::vector<std::size_t> query_cells(std::vector<Cell> cells,
                                         double min_x_mm,
                                         double min_y_mm,
                                         double max_x_mm,
                                         double max_y_mm,
                                         double z_min_mm,
                                         double z_max_mm,
                                         PrintedSceneQueryStats *stats) const;

    std::vector<DepositedSegment> m_segments;
    std::vector<PrimitiveBounds>  m_bounds;
    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> m_grid;
};

} // namespace Slic3r

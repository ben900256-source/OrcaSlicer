#pragma once

#include "PrintedScene.hpp"
#include "ToolEnvelope.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace Slic3r {

struct ToolContactPolicy {
    bool   check_deposition { false };
    bool   check_hard_collision { true };
    bool   check_thermal { false };
    bool   check_toolhead { true };
    double deposition_margin_mm { 0.0 };
    double hard_collision_margin_mm { 0.0 };
    double thermal_margin_mm { 0.0 };
    double toolhead_margin_mm { 0.0 };

    bool   checks(ToolEnvelopeZone zone) const noexcept;
    double margin_mm(ToolEnvelopeZone zone) const noexcept;
    void   validate() const;
};

struct ToolClearanceResult {
    bool             collision { false };
    double           first_hit_t { std::numeric_limits<double>::infinity() };
    double           last_hit_t { std::numeric_limits<double>::infinity() };
    std::uint64_t    primitive_id { 0 };
    std::size_t      obstacle_sequence { 0 };
    std::size_t      obstacle_index { 0 };
    std::size_t      slice_index { 0 };
    ToolEnvelopeZone zone { ToolEnvelopeZone::HardCollision };
    std::string      slice_name;
    double           signed_vertical_clearance_mm { std::numeric_limits<double>::infinity() };
    double           required_z_raise_mm { 0.0 };
    Vec3d            pose_mm { Vec3d::Zero() };
    PrintedSceneQueryStats query_stats;
    std::string      diagnostic;

    explicit operator bool() const noexcept { return collision; }
};

ToolClearanceResult check_pose(const ToolEnvelope &envelope,
                               const PrintedScene &scene,
                               const Vec3d &pose_mm,
                               const ToolContactPolicy &policy = {});

std::vector<ToolClearanceResult> check_pose_all(const ToolEnvelope &envelope,
                                                const PrintedScene &scene,
                                                const Vec3d &pose_mm,
                                                const ToolContactPolicy &policy = {},
                                                PrintedSceneQueryStats *stats = nullptr);

ToolClearanceResult check_sweep(const ToolEnvelope &envelope,
                                const PrintedScene &scene,
                                const Vec3d &start_pose_mm,
                                const Vec3d &end_pose_mm,
                                const ToolContactPolicy &policy = {});

std::vector<ToolClearanceResult> check_sweep_all(const ToolEnvelope &envelope,
                                                 const PrintedScene &scene,
                                                 const Vec3d &start_pose_mm,
                                                 const Vec3d &end_pose_mm,
                                                 const ToolContactPolicy &policy = {},
                                                 PrintedSceneQueryStats *stats = nullptr);

double required_z_raise_for_sweep(const ToolEnvelope &envelope,
                                  const PrintedScene &scene,
                                  const Vec3d &start_pose_mm,
                                  const Vec3d &end_pose_mm,
                                  const ToolContactPolicy &policy = {});

class ToolEnvelopeCollision
{
public:
    static ToolClearanceResult check_pose(const ToolEnvelope &envelope,
                                          const PrintedScene &scene,
                                          const Vec3d &pose_mm,
                                          const ToolContactPolicy &policy = {})
    { return Slic3r::check_pose(envelope, scene, pose_mm, policy); }

    static std::vector<ToolClearanceResult> check_pose_all(const ToolEnvelope &envelope,
                                                           const PrintedScene &scene,
                                                           const Vec3d &pose_mm,
                                                           const ToolContactPolicy &policy = {},
                                                           PrintedSceneQueryStats *stats = nullptr)
    { return Slic3r::check_pose_all(envelope, scene, pose_mm, policy, stats); }

    static ToolClearanceResult check_sweep(const ToolEnvelope &envelope,
                                           const PrintedScene &scene,
                                           const Vec3d &start_pose_mm,
                                           const Vec3d &end_pose_mm,
                                           const ToolContactPolicy &policy = {})
    { return Slic3r::check_sweep(envelope, scene, start_pose_mm, end_pose_mm, policy); }

    static std::vector<ToolClearanceResult> check_sweep_all(const ToolEnvelope &envelope,
                                                            const PrintedScene &scene,
                                                            const Vec3d &start_pose_mm,
                                                            const Vec3d &end_pose_mm,
                                                            const ToolContactPolicy &policy = {},
                                                            PrintedSceneQueryStats *stats = nullptr)
    { return Slic3r::check_sweep_all(envelope, scene, start_pose_mm, end_pose_mm, policy, stats); }

    static double required_z_raise_for_sweep(const ToolEnvelope &envelope,
                                             const PrintedScene &scene,
                                             const Vec3d &start_pose_mm,
                                             const Vec3d &end_pose_mm,
                                             const ToolContactPolicy &policy = {})
    { return Slic3r::required_z_raise_for_sweep(envelope, scene, start_pose_mm, end_pose_mm, policy); }
};

} // namespace Slic3r

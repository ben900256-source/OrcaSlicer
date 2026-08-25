#pragma once

#include "ToolEnvelopeCollision.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace Slic3r {

struct ToolpathValidationMove {
    std::uint64_t move_id { 0 };
    Vec3d         start_mm { Vec3d::Zero() };
    Vec3d         end_mm { Vec3d::Zero() };
    bool          deposits_material { false };
    double        width_mm { 0.0 };
    double        height_mm { 0.0 };
    std::string   role;
    std::size_t   extruder_id { 0 };
};

using ToolpathClearanceMove = ToolpathValidationMove;

struct ToolpathMoveValidation {
    std::uint64_t      move_id { 0 };
    std::size_t        sequence_index { 0 };
    bool               collision { false };
    ToolClearanceResult clearance;
};

struct ToolpathValidationReport {
    bool                                collision_free { true };
    std::vector<ToolpathMoveValidation> moves;
    std::vector<ToolpathMoveValidation> collisions;
    PrintedScene                        final_scene;

    const ToolpathMoveValidation *first_collision() const noexcept
    { return collisions.empty() ? nullptr : &collisions.front(); }
};

using ToolpathClearanceReport = ToolpathValidationReport;

class ToolpathClearanceValidator
{
public:
    ToolpathClearanceValidator(ToolEnvelope envelope, ToolContactPolicy policy = {});

    ToolpathValidationReport validate(const std::vector<ToolpathValidationMove> &moves) const;

private:
    ToolEnvelope      m_envelope;
    ToolContactPolicy m_policy;
};

ToolpathValidationReport validate_ordered_toolpath(const ToolEnvelope &envelope,
                                                   const std::vector<ToolpathValidationMove> &moves,
                                                   const ToolContactPolicy &policy = {});

} // namespace Slic3r

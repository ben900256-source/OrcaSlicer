#include "ToolpathClearanceValidator.hpp"

#include <utility>

namespace Slic3r {

ToolpathClearanceValidator::ToolpathClearanceValidator(ToolEnvelope envelope,
                                                       ToolContactPolicy policy)
    : m_envelope(std::move(envelope))
    , m_policy(policy)
{
    m_policy.validate();
}

ToolpathValidationReport ToolpathClearanceValidator::validate(
    const std::vector<ToolpathValidationMove> &moves) const
{
    ToolpathValidationReport report;
    report.moves.reserve(moves.size());

    for (std::size_t sequence_index = 0; sequence_index < moves.size(); ++sequence_index) {
        const ToolpathValidationMove &move = moves[sequence_index];

        // Temporal ordering is the core contract: a move sees everything already
        // printed, but never sees its own bead before the move has completed.
        ToolpathMoveValidation validation;
        validation.move_id = move.move_id;
        validation.sequence_index = sequence_index;
        validation.clearance = check_sweep(
            m_envelope, report.final_scene, move.start_mm, move.end_mm, m_policy);
        validation.collision = validation.clearance.collision;

        report.moves.push_back(validation);
        if (validation.collision) {
            report.collision_free = false;
            report.collisions.push_back(validation);
        }

        if (move.deposits_material) {
            report.final_scene.insert(DepositedSegment(
                move.start_mm,
                move.end_mm,
                move.width_mm,
                move.height_mm,
                move.move_id,
                sequence_index));
        }
    }
    return report;
}

ToolpathValidationReport validate_ordered_toolpath(
    const ToolEnvelope &envelope,
    const std::vector<ToolpathValidationMove> &moves,
    const ToolContactPolicy &policy)
{
    return ToolpathClearanceValidator(envelope, policy).validate(moves);
}

} // namespace Slic3r

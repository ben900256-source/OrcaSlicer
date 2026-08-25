#include <catch2/catch_all.hpp>

#include "libslic3r/Exception.hpp"
#include "libslic3r/ToolpathClearanceValidator.hpp"

using namespace Slic3r;

namespace {

ToolEnvelope validator_envelope()
{
    return ToolEnvelope({ToolEnvelopeSlice(
        0.0,
        1.0,
        ExPolygons{ExPolygon(Polygon::new_scale({
            {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}
        }))},
        ToolEnvelopeZone::HardCollision,
        "nozzle")});
}

ToolpathValidationMove extrusion(std::uint64_t id,
                                 const Vec3d &start,
                                 const Vec3d &end)
{
    ToolpathValidationMove move;
    move.move_id = id;
    move.start_mm = start;
    move.end_mm = end;
    move.deposits_material = true;
    move.width_mm = 0.4;
    move.height_mm = 0.2;
    move.role = "perimeter";
    return move;
}

ToolpathValidationMove travel(std::uint64_t id,
                              const Vec3d &start,
                              const Vec3d &end)
{
    ToolpathValidationMove move;
    move.move_id = id;
    move.start_mm = start;
    move.end_mm = end;
    return move;
}

} // namespace

TEST_CASE("Ordered validation checks a move before depositing its material", "[ToolpathClearanceValidator]")
{
    const std::vector<ToolpathValidationMove> moves = {
        extrusion(101, {-4., 0., 0.2}, {4., 0., 0.2}),
        extrusion(202, {-4., 0., 0.2}, {4., 0., 0.2})
    };

    const ToolpathValidationReport report = validate_ordered_toolpath(
        validator_envelope(), moves);
    REQUIRE_FALSE(report.collision_free);
    REQUIRE(report.moves.size() == 2);
    REQUIRE_FALSE(report.moves[0].collision);
    REQUIRE(report.moves[1].collision);
    REQUIRE(report.moves[1].clearance.primitive_id == 101);
    REQUIRE(report.moves[1].clearance.obstacle_sequence == 0);
    REQUIRE(report.final_scene.size() == 2);
}

TEST_CASE("Ordered validation maps move IDs and input order onto deposited primitives", "[ToolpathClearanceValidator]")
{
    const std::vector<ToolpathValidationMove> moves = {
        extrusion(9001, {-3., -2., 0.2}, {3., -2., 0.2}),
        travel(77, {-3., 3., 0.2}, {3., 3., 0.2}),
        extrusion(42, {-3., 2., 0.2}, {3., 2., 0.2})
    };
    const ToolpathValidationReport report = ToolpathClearanceValidator(
        validator_envelope()).validate(moves);

    REQUIRE(report.collision_free);
    REQUIRE(report.final_scene.size() == 2);
    REQUIRE(report.final_scene.segment(0).primitive_id == 9001);
    REQUIRE(report.final_scene.segment(0).sequence_index == 0);
    REQUIRE(report.final_scene.segment(1).primitive_id == 42);
    REQUIRE(report.final_scene.segment(1).sequence_index == 2);
}

TEST_CASE("Ordered validation records each move's earliest deterministic diagnostic", "[ToolpathClearanceValidator]")
{
    std::vector<ToolpathValidationMove> moves = {
        extrusion(5, {-3., 0., 0.2}, {3., 0., 0.2}),
        extrusion(3, {-3., 0., 0.2}, {3., 0., 0.2}),
        travel(99, {-3., 0., 0.2}, {3., 0., 0.2})
    };
    const ToolpathValidationReport report = validate_ordered_toolpath(
        validator_envelope(), moves);

    REQUIRE(report.collisions.size() == 2);
    REQUIRE(report.collisions[0].move_id == 3);
    REQUIRE(report.collisions[0].clearance.primitive_id == 5);
    REQUIRE(report.collisions[1].move_id == 99);
    REQUIRE(report.collisions[1].clearance.primitive_id == 5);
    REQUIRE(report.first_collision() == &report.collisions.front());
}

TEST_CASE("Ordered validation honors the supplied contact policy", "[ToolpathClearanceValidator]")
{
    ToolEnvelope deposition_envelope({ToolEnvelopeSlice(
        0.0, 1.0,
        ExPolygons{ExPolygon(Polygon::new_scale({
            {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}
        }))},
        ToolEnvelopeZone::Deposition)});
    const std::vector<ToolpathValidationMove> moves = {
        extrusion(1, {-2., 0., 0.2}, {2., 0., 0.2}),
        travel(2, {-2., 0., 0.2}, {2., 0., 0.2})
    };

    REQUIRE(validate_ordered_toolpath(deposition_envelope, moves).collision_free);
    ToolContactPolicy policy;
    policy.check_deposition = true;
    REQUIRE_FALSE(validate_ordered_toolpath(deposition_envelope, moves, policy).collision_free);
}

TEST_CASE("Ordered validation rejects malformed extrusion dimensions when deposition completes", "[ToolpathClearanceValidator]")
{
    ToolpathValidationMove bad = extrusion(1, {0., 0., 0.2}, {1., 0., 0.2});
    bad.width_mm = 0.0;
    REQUIRE_THROWS_AS(validate_ordered_toolpath(validator_envelope(), {bad}), InvalidArgument);
}

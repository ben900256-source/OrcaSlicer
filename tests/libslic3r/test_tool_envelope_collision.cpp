#include <catch2/catch_all.hpp>

#include "libslic3r/ToolEnvelopeCollision.hpp"

#include <algorithm>
#include <cmath>

using namespace Slic3r;

namespace {

Polygon rectangle(double min_x, double min_y, double max_x, double max_y)
{
    return Polygon::new_scale({
        {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}
    });
}

ToolEnvelope box_envelope(ToolEnvelopeZone zone = ToolEnvelopeZone::HardCollision,
                          double z_min = 0.0,
                          double z_max = 1.0)
{
    return ToolEnvelope({ToolEnvelopeSlice(
        z_min, z_max, ExPolygons{ExPolygon(rectangle(-1., -1., 1., 1.))}, zone, "box")});
}

PrintedScene point_obstacle(double x,
                            double y,
                            double z,
                            double width = 0.4,
                            double height = 0.2,
                            std::uint64_t id = 10,
                            std::size_t sequence = 0)
{
    PrintedScene scene;
    scene.insert(DepositedSegment({x, y, z}, {x, y, z}, width, height, id, sequence));
    return scene;
}

} // namespace

TEST_CASE("Pose checks preserve asymmetric tool geometry and bead-top Z", "[ToolEnvelopeCollision]")
{
    ToolEnvelope envelope({ToolEnvelopeSlice(
        0.0, 1.0, ExPolygons{ExPolygon(rectangle(0., -0.5, 2., 0.5))},
        ToolEnvelopeZone::HardCollision, "offset block")});
    const PrintedScene scene = point_obstacle(1.8, 0.0, 0.5);

    REQUIRE(check_pose(envelope, scene, {0.0, 0.0, 0.5}).collision);
    REQUIRE_FALSE(check_pose(envelope, scene, {-3.0, 0.0, 0.5}).collision);
    REQUIRE(check_pose(envelope, scene, {0.0, 0.0, -0.5}).collision);
    REQUIRE_FALSE(check_pose(envelope, scene, {0.0, 0.0, 0.5002}).collision);
}

TEST_CASE("Pose checks preserve polygon holes", "[ToolEnvelopeCollision]")
{
    Polygon contour = rectangle(-3., -3., 3., 3.);
    Polygon hole = rectangle(-1., -1., 1., 1.);
    hole.reverse();
    ToolEnvelope envelope({ToolEnvelopeSlice(
        0.0, 1.0, ExPolygons{ExPolygon(std::move(contour), std::move(hole))},
        ToolEnvelopeZone::HardCollision, "ring")});

    REQUIRE_FALSE(check_pose(envelope, point_obstacle(0.0, 0.0, 0.5), {0., 0., 0.}).collision);
    REQUIRE(check_pose(envelope, point_obstacle(2.0, 0.0, 0.5), {0., 0., 0.}).collision);
}

TEST_CASE("Contact policies select zones independently and apply XY margins", "[ToolEnvelopeCollision]")
{
    const PrintedScene centered = point_obstacle(0.0, 0.0, 0.5);

    REQUIRE_FALSE(check_pose(box_envelope(ToolEnvelopeZone::Deposition), centered, {0., 0., 0.}).collision);
    REQUIRE(check_pose(box_envelope(ToolEnvelopeZone::HardCollision), centered, {0., 0., 0.}).collision);
    REQUIRE_FALSE(check_pose(box_envelope(ToolEnvelopeZone::Thermal), centered, {0., 0., 0.}).collision);
    REQUIRE(check_pose(box_envelope(ToolEnvelopeZone::Toolhead), centered, {0., 0., 0.}).collision);

    ToolContactPolicy thermal_only;
    thermal_only.check_hard_collision = false;
    thermal_only.check_toolhead = false;
    thermal_only.check_thermal = true;
    REQUIRE(check_pose(box_envelope(ToolEnvelopeZone::Thermal), centered, {0., 0., 0.}, thermal_only).collision);

    const PrintedScene separated = point_obstacle(1.7, 0.0, 0.5, 0.4);
    REQUIRE_FALSE(check_pose(box_envelope(), separated, {0., 0., 0.}).collision);
    ToolContactPolicy margin_policy;
    margin_policy.hard_collision_margin_mm = 0.5;
    REQUIRE(check_pose(box_envelope(), separated, {0., 0., 0.}, margin_policy).collision);
}

TEST_CASE("Boundary contact is a collision within OrcaSlicer tolerance", "[ToolEnvelopeCollision]")
{
    const ToolEnvelope envelope = box_envelope();
    REQUIRE(check_pose(envelope, point_obstacle(1.2, 0.0, 0.5, 0.4), {0., 0., 0.}).collision);
    REQUIRE_FALSE(check_pose(envelope, point_obstacle(1.2003, 0.0, 0.5, 0.4), {0., 0., 0.}).collision);
}

TEST_CASE("Pose checks reject coordinates outside scaled integer range", "[ToolEnvelopeCollision]")
{
    REQUIRE_THROWS_AS(check_pose(
        box_envelope(), point_obstacle(0.0, 0.0, 0.5), {1e20, 0.0, 0.0}), InvalidArgument);
}

TEST_CASE("All-hit pose queries sort deterministically and collapse slice duplicates", "[ToolEnvelopeCollision]")
{
    ToolEnvelope envelope({
        ToolEnvelopeSlice(0.0, 1.0, ExPolygons{ExPolygon(rectangle(-1., -1., 1., 1.))}, ToolEnvelopeZone::HardCollision, "lower"),
        ToolEnvelopeSlice(0.2, 1.2, ExPolygons{ExPolygon(rectangle(-1., -1., 1., 1.))}, ToolEnvelopeZone::HardCollision, "upper")
    });
    PrintedScene scene;
    scene.insert(DepositedSegment({0., 0., 0.5}, {0., 0., 0.5}, 0.4, 0.2, 90, 7));
    scene.insert(DepositedSegment({0., 0., 0.5}, {0., 0., 0.5}, 0.4, 0.2, 20, 2));
    scene.insert(DepositedSegment({0., 0., 0.5}, {0., 0., 0.5}, 0.4, 0.2, 10, 2));

    const std::vector<ToolClearanceResult> hits = check_pose_all(envelope, scene, {0., 0., 0.});
    REQUIRE(hits.size() == 3);
    REQUIRE(hits[0].primitive_id == 10);
    REQUIRE(hits[1].primitive_id == 20);
    REQUIRE(hits[2].primitive_id == 90);
    REQUIRE(std::all_of(hits.begin(), hits.end(), [](const ToolClearanceResult &hit) {
        return hit.slice_index == 0;
    }));
}

TEST_CASE("Pose diagnostics expose signed clearance and conservative Z raise", "[ToolEnvelopeCollision]")
{
    const ToolClearanceResult hit = check_pose(
        box_envelope(), point_obstacle(0., 0., 0.5, 0.4, 0.2, 42, 11), {0., 0., 0.});
    REQUIRE(hit.collision);
    REQUIRE(hit.signed_vertical_clearance_mm <= 0.0);
    REQUIRE_THAT(hit.required_z_raise_mm, Catch::Matchers::WithinAbs(0.5001, 2e-6));
    REQUIRE(hit.diagnostic.find("primitive 42") != std::string::npos);
}

TEST_CASE("Adaptive sweeps detect thin obstacles without tunneling", "[ToolEnvelopeCollision]")
{
    const ToolEnvelope envelope = box_envelope();
    const PrintedScene scene = point_obstacle(0., 0., 0.5, 0.05);
    const ToolClearanceResult hit = check_sweep(
        envelope, scene, {-10., 0., 0.}, {10., 0., 0.});

    REQUIRE(hit.collision);
    REQUIRE(hit.first_hit_t > 0.4);
    REQUIRE(hit.first_hit_t < 0.5);
    REQUIRE(hit.last_hit_t > hit.first_hit_t);
    REQUIRE_FALSE(check_sweep(envelope, scene, {-10., 3., 0.}, {10., 3., 0.}).collision);
}

TEST_CASE("Adaptive sweeps handle simultaneous XY and Z motion conservatively", "[ToolEnvelopeCollision]")
{
    const ToolEnvelope envelope = box_envelope();
    const PrintedScene scene = point_obstacle(0., 0., 1.0, 0.2, 0.2);

    const ToolClearanceResult hit = check_sweep(
        envelope, scene, {-4., 0., -1.0}, {4., 0., 1.0});
    REQUIRE(hit.collision);
    REQUIRE(hit.first_hit_t >= 0.0);
    REQUIRE(hit.first_hit_t <= 1.0);

    const ToolClearanceResult above = check_sweep(
        envelope, scene, {-4., 0., 1.0002}, {4., 0., 1.0002});
    REQUIRE_FALSE(above.collision);
}

TEST_CASE("Sweep all-hit ordering uses hit time then obstacle sequence and ID", "[ToolEnvelopeCollision]")
{
    const ToolEnvelope envelope = box_envelope();
    PrintedScene scene;
    scene.insert(DepositedSegment({3., 0., 0.5}, {3., 0., 0.5}, 0.2, 0.2, 30, 0));
    scene.insert(DepositedSegment({-3., 0., 0.5}, {-3., 0., 0.5}, 0.2, 0.2, 40, 9));
    scene.insert(DepositedSegment({-3., 0., 0.5}, {-3., 0., 0.5}, 0.2, 0.2, 20, 2));

    const std::vector<ToolClearanceResult> hits = check_sweep_all(
        envelope, scene, {-5., 0., 0.}, {5., 0., 0.});
    REQUIRE(hits.size() == 3);
    REQUIRE(hits[0].primitive_id == 20);
    REQUIRE(hits[1].primitive_id == 40);
    REQUIRE(hits[2].primitive_id == 30);
    REQUIRE(hits[0].first_hit_t <= hits[2].first_hit_t);
}

TEST_CASE("Sweep Z raise is the maximum required by reported obstacles", "[ToolEnvelopeCollision]")
{
    const ToolEnvelope envelope = box_envelope();
    PrintedScene scene;
    scene.insert(DepositedSegment({-2., 0., 0.4}, {-2., 0., 0.4}, 0.2, 0.2, 1, 0));
    scene.insert(DepositedSegment({ 2., 0., 0.8}, { 2., 0., 0.8}, 0.2, 0.2, 2, 1));

    const double raise = required_z_raise_for_sweep(
        envelope, scene, {-5., 0., 0.}, {5., 0., 0.});
    REQUIRE_THAT(raise, Catch::Matchers::WithinAbs(0.8001, 2e-6));
}

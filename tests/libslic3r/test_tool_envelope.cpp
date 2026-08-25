#include <catch2/catch_all.hpp>

#include "libslic3r/Exception.hpp"
#include "libslic3r/PrintedScene.hpp"
#include "libslic3r/ToolEnvelope.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <string>

using namespace Slic3r;

namespace {

Polygon ring(std::initializer_list<Vec2d> points)
{
    Polygon result;
    for (const Vec2d &point : points)
        result.points.emplace_back(Point::new_scale(point));
    return result;
}

ExPolygon square_with_hole()
{
    Polygon contour = ring({{-3., -3.}, {3., -3.}, {3., 3.}, {-3., 3.}});
    Polygon hole = ring({{-1., -1.}, {-1., 1.}, {1., 1.}, {1., -1.}});
    return ExPolygon(std::move(contour), std::move(hole));
}

std::string exception_message(const std::function<void()> &function)
{
    try {
        function();
    } catch (const InvalidArgument &error) {
        return error.what();
    }
    return {};
}

} // namespace

TEST_CASE("Tool envelopes round-trip version-one JSON including polygon holes", "[ToolEnvelope]")
{
    ToolEnvelope source({ToolEnvelopeSlice(
        -0.2, 1.5, ExPolygons{square_with_hole()},
        ToolEnvelopeZone::HardCollision, "heater block")}, "0.4 mm nozzle");

    const nlohmann::json json = source.to_json();
    REQUIRE(json.at("version") == 1);
    REQUIRE(json.at("origin") == "nozzle_tip");
    REQUIRE(json.at("slices")[0].at("geometry")[0].is_object());
    REQUIRE(json.at("slices")[0].at("geometry")[0].at("holes").size() == 1);

    const ToolEnvelope restored = ToolEnvelope::from_json(json);
    REQUIRE(restored.name() == source.name());
    REQUIRE(restored.slices().size() == 1);
    REQUIRE(restored.slices()[0].geometry() == source.slices()[0].geometry());
    REQUIRE(restored.slices()[0].zone() == ToolEnvelopeZone::HardCollision);
    REQUIRE_THAT(restored.z_min_mm(), Catch::Matchers::WithinAbs(-0.2, 1e-12));
    REQUIRE_THAT(restored.z_max_mm(), Catch::Matchers::WithinAbs(1.5, 1e-12));
}

TEST_CASE("Tool envelope JSON accepts a bare contour ring and canonicalizes it", "[ToolEnvelope]")
{
    const nlohmann::json json = {
        {"version", 1},
        {"origin", "nozzle_tip"},
        {"slices", nlohmann::json::array({{
            {"zone", "thermal"},
            {"z_min_mm", 0.0},
            {"z_max_mm", 2.0},
            {"geometry", nlohmann::json::array({
                nlohmann::json::array({
                    nlohmann::json::array({-1.0, -1.0}),
                    nlohmann::json::array({ 1.0, -1.0}),
                    nlohmann::json::array({ 1.0,  1.0}),
                    nlohmann::json::array({-1.0,  1.0}),
                    nlohmann::json::array({-1.0, -1.0})
                })
            })}
        }})}
    };

    const ToolEnvelope envelope = ToolEnvelope::from_json(json);
    const nlohmann::json canonical = envelope.to_json();
    REQUIRE(canonical.at("slices")[0].at("zone") == "thermal");
    REQUIRE(canonical.at("slices")[0].at("geometry")[0].is_object());
    REQUIRE(canonical.at("slices")[0].at("geometry")[0].at("contour").size() == 4);
    REQUIRE(canonical.at("slices")[0].at("geometry")[0].at("holes").empty());
}

TEST_CASE("Tool envelope validation reports the malformed field", "[ToolEnvelope]")
{
    nlohmann::json base = {
        {"version", 1}, {"origin", "nozzle_tip"},
        {"slices", nlohmann::json::array({{
            {"zone", "hard_collision"}, {"z_min_mm", 0.0}, {"z_max_mm", 1.0},
            {"geometry", nlohmann::json::array({nlohmann::json::array({
                nlohmann::json::array({0.0, 0.0}), nlohmann::json::array({2.0, 0.0}),
                nlohmann::json::array({2.0, 2.0}), nlohmann::json::array({0.0, 2.0})
            })})}
        }})}
    };

    SECTION("unsupported version") {
        base["version"] = 2;
        REQUIRE(exception_message([&] { (void) ToolEnvelope::from_json(base); }).find("json.version") != std::string::npos);
    }
    SECTION("non-finite coordinate") {
        base["slices"][0]["geometry"][0][1][0] = std::numeric_limits<double>::infinity();
        REQUIRE(exception_message([&] { (void) ToolEnvelope::from_json(base); }).find("geometry[0].contour[1][0]") != std::string::npos);
    }
    SECTION("invalid Z range") {
        base["slices"][0]["z_max_mm"] = 0.0;
        REQUIRE(exception_message([&] { (void) ToolEnvelope::from_json(base); }).find("z_range") != std::string::npos);
    }
    SECTION("self-intersecting contour") {
        base["slices"][0]["geometry"][0] = {
            {0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {3.0, 0.0}
        };
        REQUIRE_FALSE(exception_message([&] { (void) ToolEnvelope::from_json(base); }).empty());
    }
    SECTION("hole outside contour") {
        base["slices"][0]["geometry"][0] = {
            {"contour", {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}},
            {"holes", nlohmann::json::array({{{3.0, 3.0}, {3.0, 4.0}, {4.0, 4.0}, {4.0, 3.0}}})}
        };
        REQUIRE(exception_message([&] { (void) ToolEnvelope::from_json(base); }).find("holes[0]") != std::string::npos);
    }
}

TEST_CASE("Printed scenes use bead-top Z and accept point deposits", "[ToolEnvelope]")
{
    PrintedScene scene;
    scene.insert(DepositedSegment({1.0, 2.0, 0.4}, {1.0, 2.0, 0.4}, 0.6, 0.2, 7, 3));
    scene.insert(DepositedSegment({0.0, 0.0, 1.0}, {5.0, 0.0, 2.0}, 0.4, 0.3, 8, 4));

    REQUIRE(scene.size() == 2);
    REQUIRE_THAT(scene.bounds(0).min_x_mm, Catch::Matchers::WithinAbs(0.7, 1e-12));
    REQUIRE_THAT(scene.bounds(0).z_min_mm, Catch::Matchers::WithinAbs(0.2, 1e-12));
    REQUIRE_THAT(scene.bounds(0).z_max_mm, Catch::Matchers::WithinAbs(0.4, 1e-12));
    REQUIRE_THAT(scene.bounds(1).z_min_mm, Catch::Matchers::WithinAbs(0.7, 1e-12));
    REQUIRE_THAT(scene.bounds(1).z_max_mm, Catch::Matchers::WithinAbs(2.0, 1e-12));
}

TEST_CASE("Printed scene index matches brute force on deterministic random queries", "[ToolEnvelope]")
{
    std::mt19937 generator(0xC0111DEu);
    std::uniform_real_distribution<double> xy(-250.0, 250.0);
    std::uniform_real_distribution<double> z(0.2, 20.0);
    PrintedScene scene;
    for (std::size_t index = 0; index < 2000; ++index) {
        const Vec3d start(xy(generator), xy(generator), z(generator));
        const Vec3d end = start + Vec3d(xy(generator) * 0.02, xy(generator) * 0.02, xy(generator) * 0.002);
        scene.insert(DepositedSegment(start, end, 0.45, 0.2, index, index));
    }

    for (std::size_t query = 0; query < 200; ++query) {
        const double x = xy(generator);
        const double y = xy(generator);
        const double query_z = z(generator);
        const std::vector<std::size_t> indexed = scene.query_indices(
            x - 3.0, y - 3.0, x + 3.0, y + 3.0, query_z - 0.5, query_z + 0.5);
        const std::vector<std::size_t> brute = scene.brute_force_query_indices(
            x - 3.0, y - 3.0, x + 3.0, y + 3.0, query_z - 0.5, query_z + 0.5);
        REQUIRE(indexed == brute);
        REQUIRE(std::is_sorted(indexed.begin(), indexed.end()));
    }
}

TEST_CASE("Printed scene localized queries inspect a small fraction of one hundred thousand deposits", "[ToolEnvelope]")
{
    PrintedScene scene;
    constexpr std::size_t segment_count = 100000;
    for (std::size_t index = 0; index < segment_count; ++index) {
        const double x = static_cast<double>(index % 1000) * 10.0;
        const double y = static_cast<double>(index / 1000) * 10.0;
        scene.insert(DepositedSegment({x, y, 0.2}, {x + 0.5, y, 0.2}, 0.4, 0.2, index, index));
    }

    PrintedSceneQueryStats stats;
    const std::vector<std::size_t> result = scene.query_indices(
        4998.0, 498.0, 5002.0, 502.0, 0.0, 0.3, &stats);
    INFO("visited cells: " << stats.visited_cells);
    INFO("candidates: " << stats.candidate_count);
    INFO("exact candidates: " << stats.exact_count);
    REQUIRE_FALSE(result.empty());
    REQUIRE(stats.visited_cells <= 4);
    REQUIRE(stats.candidate_count < segment_count / 1000);
}

TEST_CASE("Printed scene rejects invalid dimensions and coordinates", "[ToolEnvelope]")
{
    PrintedScene scene;
    REQUIRE_THROWS_AS(scene.insert(DepositedSegment({0., 0., 0.2}, {1., 0., 0.2}, 0.0, 0.2)), InvalidArgument);
    REQUIRE_THROWS_AS(scene.insert(DepositedSegment({0., 0., 0.2}, {1., 0., 0.2}, 0.4, -0.2)), InvalidArgument);
    REQUIRE_THROWS_AS(scene.insert(DepositedSegment(
        {std::numeric_limits<double>::quiet_NaN(), 0., 0.2}, {1., 0., 0.2}, 0.4, 0.2)), InvalidArgument);
    REQUIRE_THROWS_AS(scene.insert(DepositedSegment(
        {1e20, 0., 0.2}, {1e20, 1., 0.2}, 0.4, 0.2)), InvalidArgument);
}

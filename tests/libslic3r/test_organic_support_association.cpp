#include <catch2/catch_all.hpp>

#include "libslic3r/Support/OrganicSupportAssociation.hpp"

#include <array>

using namespace Slic3r;
using namespace Slic3r::OrganicSupport;
using Catch::Matchers::WithinAbs;

namespace {

ExtrusionId path_id(size_t layer_id, size_t entity_id,
                    ExtrusionSource source = ExtrusionSource::Perimeter,
                    size_t leaf_path_id = 0)
{
    return { layer_id, 0, source, { entity_id }, leaf_path_id };
}

PathChain chain(size_t layer_id, std::vector<Vec2d> points, double width = 0.4,
                ExtrusionRole role = erPerimeter, size_t entity_id = 0,
                ExtrusionSource source = ExtrusionSource::Perimeter, bool closed = false)
{
    return { layer_id, closed, { PathLeaf{ path_id(layer_id, entity_id, source), role, width, std::move(points) } } };
}

Pin pin(size_t contact_id, size_t layer_id, Vec2d center, double radius = 0.2)
{
    return { contact_id, layer_id, double(layer_id + 1), double(layer_id), double(layer_id) - 0.05,
             center, radius };
}

const ExtrusionAssociation &only_association(const std::vector<Contact> &contacts)
{
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts.front().associations.size() == 1);
    return contacts.front().associations.front();
}

} // namespace

TEST_CASE("Finalized path intervals clip across leaves and closed seams", "[OrganicSupportAssociation]")
{
    PathChain multipath;
    multipath.layer_id = 1;
    multipath.leaves = {
        { path_id(1, 0, ExtrusionSource::Perimeter, 0), erPerimeter, 0.4,
          { Vec2d(0., 0.), Vec2d(5., 0.) } },
        { path_id(1, 0, ExtrusionSource::Perimeter, 1), erPerimeter, 0.4,
          { Vec2d(5., 0.), Vec2d(10., 0.) } }
    };

    SECTION("an interval within one leaf retains exact endpoints") {
        const auto clipped = clip_path_chain_interval(multipath, { 2., 4. });
        REQUIRE(clipped.size() == 1);
        REQUIRE(clipped.front().size() == 2);
        CHECK_THAT(clipped.front().front().x(), WithinAbs(2., 1e-9));
        CHECK_THAT(clipped.front().back().x(), WithinAbs(4., 1e-9));
    }

    SECTION("an interval crosses a multipath leaf boundary continuously") {
        const auto clipped = clip_path_chain_interval(multipath, { 4., 6. });
        REQUIRE(clipped.size() == 1);
        REQUIRE(clipped.front().size() == 3);
        CHECK_THAT(clipped.front()[0].x(), WithinAbs(4., 1e-9));
        CHECK_THAT(clipped.front()[1].x(), WithinAbs(5., 1e-9));
        CHECK_THAT(clipped.front()[2].x(), WithinAbs(6., 1e-9));
    }

    SECTION("chain endpoints resolve without extrapolation") {
        const auto start = clip_path_chain_interval(multipath, { 0., 0. });
        const auto end   = clip_path_chain_interval(multipath, { 10., 10. });
        REQUIRE(start.size() == 1);
        REQUIRE(end.size() == 1);
        CHECK_THAT(start.front().front().x(), WithinAbs(0., 1e-9));
        CHECK_THAT(end.front().back().x(), WithinAbs(10., 1e-9));
    }

    SECTION("a closed interval wraps through the seam") {
        const PathChain loop = chain(1,
            { Vec2d(0., 0.), Vec2d(10., 0.), Vec2d(10., 10.), Vec2d(0., 10.), Vec2d(0., 0.) },
            0.4, erExternalPerimeter, 0, ExtrusionSource::Perimeter, true);
        const auto clipped = clip_path_chain_interval(loop, { 39., 41. });
        REQUIRE(clipped.size() == 1);
        REQUIRE(clipped.front().size() == 3);
        CHECK_THAT(clipped.front()[0].y(), WithinAbs(1., 1e-9));
        CHECK_THAT(clipped.front()[1].norm(), WithinAbs(0., 1e-9));
        CHECK_THAT(clipped.front()[2].x(), WithinAbs(1., 1e-9));
    }
}

TEST_CASE("Pin associations account for extrusion width and tolerance", "[OrganicSupportAssociation]")
{
    SECTION("centered straight extrusion") {
        const auto result = associate({ pin(0, 1, Vec2d(5., 0.)) },
                                      { chain(1, { Vec2d(0., 0.), Vec2d(10., 0.) }) }, 0.01);
        const ExtrusionAssociation &association = only_association(result);
        CHECK_THAT(association.centerline_distance, WithinAbs(0., 1e-9));
        CHECK_THAT(association.nominal_overlap_length, WithinAbs(0.8, 1e-9));
        CHECK(association.approximately_centered);
        CHECK_THAT(association.tangent.x(), WithinAbs(1., 1e-9));
        CHECK_THAT(association.tangent.y(), WithinAbs(0., 1e-9));
    }

    SECTION("offset extrusion associates through its width") {
        const auto result = associate({ pin(0, 1, Vec2d(5., 0.)) },
                                      { chain(1, { Vec2d(0., 0.3), Vec2d(10., 0.3) }) }, 0.01);
        const ExtrusionAssociation &association = only_association(result);
        CHECK_THAT(association.centerline_distance, WithinAbs(0.3, 1e-9));
        CHECK_THAT(association.nominal_overlap_length, WithinAbs(2. * std::sqrt(0.4 * 0.4 - 0.3 * 0.3), 1e-9));
        CHECK_FALSE(association.approximately_centered);
    }

    SECTION("tolerance-only contact has no physical interval") {
        const auto result = associate({ pin(0, 1, Vec2d(5., 0.)) },
                                      { chain(1, { Vec2d(0., 0.405), Vec2d(10., 0.405) }) }, 0.01);
        const ExtrusionAssociation &association = only_association(result);
        CHECK_FALSE(association.path_arclength_interval.has_value());
        CHECK_THAT(association.nominal_overlap_length, WithinAbs(0., 1e-9));
        CHECK_FALSE(association.planned_material_unsupported_span.has_value());
    }

    SECTION("path outside footprint and tolerance is excluded") {
        const auto result = associate({ pin(0, 1, Vec2d(5., 0.)) },
                                      { chain(1, { Vec2d(0., 0.411), Vec2d(10., 0.411) }) }, 0.01);
        REQUIRE(result.size() == 1);
        CHECK(result.front().associations.empty());
    }

    SECTION("an endpoint retains its clipped nominal interval") {
        const auto result = associate({ pin(0, 1, Vec2d(0., 0.)) },
                                      { chain(1, { Vec2d(0., 0.), Vec2d(10., 0.) }) }, 0.01);
        const ExtrusionAssociation &association = only_association(result);
        REQUIRE(association.path_arclength_interval.has_value());
        CHECK_THAT(association.path_arclength_interval->begin, WithinAbs(0., 1e-9));
        CHECK_THAT(association.path_arclength_interval->end, WithinAbs(0.4, 1e-9));
        CHECK_THAT(association.nominal_overlap_length, WithinAbs(0.4, 1e-9));
    }
}

TEST_CASE("Pin associations retain crossing and vertex geometry", "[OrganicSupportAssociation]")
{
    SECTION("orthogonal and oblique paths remain distinct") {
        const std::vector<PathChain> paths {
            chain(1, { Vec2d(-2., 0.), Vec2d(2., 0.) }, 0.4, erPerimeter, 0),
            chain(1, { Vec2d(0., -2.), Vec2d(0., 2.) }, 0.5, erSolidInfill, 1, ExtrusionSource::Fill),
            chain(1, { Vec2d(-2., -1.), Vec2d(2., 1.) }, 0.3, erBridgeInfill, 2, ExtrusionSource::Fill)
        };
        const auto result = associate({ pin(0, 1, Vec2d::Zero()) }, paths, 0.01);
        REQUIRE(result.front().associations.size() == 3);
        CHECK(result.front().associations[0].role == erPerimeter);
        CHECK(result.front().associations[1].role == erSolidInfill);
        CHECK(result.front().associations[2].role == erBridgeInfill);
    }

    SECTION("a vertex uses the adjacent angle bisector") {
        const auto result = associate({ pin(0, 1, Vec2d::Zero()) },
                                      { chain(1, { Vec2d(-1., 0.), Vec2d(0., 0.), Vec2d(0., 1.) }) }, 0.01);
        const ExtrusionAssociation &association = only_association(result);
        CHECK_FALSE(association.tangent_ambiguous);
        CHECK_THAT(association.tangent.x(), WithinAbs(std::sqrt(0.5), 1e-9));
        CHECK_THAT(association.tangent.y(), WithinAbs(std::sqrt(0.5), 1e-9));
    }

    SECTION("a double-back reports ambiguity and disjoint occurrences") {
        const PathChain doubled = chain(1, { Vec2d(-2., 0.), Vec2d(2., 0.), Vec2d(-2., 0.) });
        const auto result = associate({ pin(0, 1, Vec2d::Zero()) }, { doubled }, 0.01);
        REQUIRE(result.front().associations.size() == 2);
        REQUIRE(result.front().associations[0].path_arclength_interval.has_value());
        REQUIRE(result.front().associations[1].path_arclength_interval.has_value());
        CHECK(result.front().associations[0].path_arclength_interval->end <
              result.front().associations[1].path_arclength_interval->begin);

        const auto cusp_result = associate({ pin(0, 1, Vec2d(2., 0.)) }, { doubled }, 0.01);
        const ExtrusionAssociation &cusp = only_association(cusp_result);
        CHECK(cusp.tangent_ambiguous);
    }

    SECTION("zero-length segments do not create records") {
        const auto result = associate({ pin(0, 1, Vec2d(1., 0.)) },
                                      { chain(1, { Vec2d(0., 0.), Vec2d(0., 0.), Vec2d(2., 0.) }) }, 0.01);
        CHECK_THAT(only_association(result).nominal_overlap_length, WithinAbs(0.8, 1e-9));
    }
}

TEST_CASE("Associations are stable across pins roles and leaf widths", "[OrganicSupportAssociation]")
{
    PathChain variable_width;
    variable_width.layer_id = 1;
    variable_width.leaves = {
        { path_id(1, 0, ExtrusionSource::Perimeter, 0), erExternalPerimeter, 0.4,
          { Vec2d(0., 0.), Vec2d(5., 0.) } },
        { path_id(1, 0, ExtrusionSource::Perimeter, 1), erOverhangPerimeter, 0.8,
          { Vec2d(5., 0.), Vec2d(10., 0.) } }
    };
    const auto result = associate({ pin(0, 1, Vec2d(2., 0.)), pin(1, 1, Vec2d(7., 0.3)) },
                                  { variable_width,
                                    chain(1, { Vec2d(2., -1.), Vec2d(2., 1.) }, 0.35,
                                          erInternalInfill, 1, ExtrusionSource::Fill) }, 0.01);
    REQUIRE(result.size() == 2);
    REQUIRE(result[0].associations.size() == 2);
    REQUIRE(result[1].associations.size() == 1);
    CHECK(result[0].associations[0].extrusion_id.leaf_path_index == 0);
    CHECK(result[0].associations[1].role == erInternalInfill);
    CHECK(result[1].associations[0].extrusion_id.leaf_path_index == 1);
    CHECK_THAT(result[1].associations[0].width, WithinAbs(0.8, 1e-9));
}

TEST_CASE("Fitted arcs are tessellated within association tolerance", "[OrganicSupportAssociation]")
{
    const Point center(0, 0);
    const Point start(scaled<coord_t>(10.), coord_t(0));
    const Point end(coord_t(0), scaled<coord_t>(10.));
    const ArcSegment arc(center, scaled<coord_t>(10.), start, end, ArcDirection::Arc_Dir_CCW);
    ExtrusionPath path(erExternalPerimeter, 0.1, 0.4, 0.2);
    path.polyline.points = { Point3(start, 0), Point3(end, 0) };
    path.polyline.fitting_result.push_back({ 0, 1, EMovePathType::Arc_move_ccw, arc });

    const std::vector<Vec2d> chord = tessellate_path(path, false, 0.01);
    const std::vector<Vec2d> fitted = tessellate_path(path, true, 0.01);
    REQUIRE(chord.size() == 2);
    REQUIRE(fitted.size() > 2);
    CHECK_THAT(fitted.front().x(), WithinAbs(10., 1e-9));
    CHECK_THAT(fitted.back().y(), WithinAbs(10., 1e-9));
    for (size_t idx = 1; idx < fitted.size(); ++idx) {
        const Vec2d midpoint = 0.5 * (fitted[idx - 1] + fitted[idx]);
        CHECK(10. - midpoint.norm() <= 0.01 + 1e-9);
    }

    PathChain fitted_chain = chain(1, fitted, 0.4, erExternalPerimeter);
    const Vec2d pin_center = Vec2d::Constant(10. / std::sqrt(2.));
    CHECK_FALSE(associate({ pin(0, 1, pin_center) }, { fitted_chain }, 0.01)
                    .front().associations.empty());
    CHECK(associate({ pin(0, 1, pin_center) },
                    { chain(1, chord, 0.4, erExternalPerimeter) }, 0.01)
              .front().associations.empty());
}

TEST_CASE("Lower tracks anchor by swept footprint overlap", "[OrganicSupportAssociation]")
{
    const PathChain current = chain(1, { Vec2d(0., 0.), Vec2d(10., 0.) }, 0.4, erSolidInfill,
                                    0, ExtrusionSource::Fill);

    SECTION("width-only overlap creates an exact interval") {
        const PathChain lower = chain(0, { Vec2d(4., 0.35), Vec2d(5., 0.35) }, 0.4,
                                        erInternalInfill, 0, ExtrusionSource::Fill);
        const auto result = associate({ pin(0, 1, Vec2d(2., 0.)) }, { lower, current }, 0.01);
        const auto &span = *only_association(result).planned_material_unsupported_span;
        REQUIRE(span.forward.span.has_value());
        CHECK_THAT(*span.forward.span, WithinAbs((4. - std::sqrt(0.4 * 0.4 - 0.35 * 0.35)) - 2.4, 1e-8));
        REQUIRE(span.forward.anchors.size() == 1);
        CHECK(span.forward.anchors.front().type == AnchorType::LowerTrack);
    }

    SECTION("non-overlapping lower track leaves the path unanchored") {
        const PathChain lower = chain(0, { Vec2d(4., 0.41), Vec2d(5., 0.41) }, 0.4,
                                        erInternalInfill, 0, ExtrusionSource::Fill);
        const auto result = associate({ pin(0, 1, Vec2d(2., 0.)) }, { lower, current }, 0.01);
        const auto &span = *only_association(result).planned_material_unsupported_span;
        CHECK_FALSE(span.forward.span.has_value());
        REQUIRE(span.forward.unanchored_distance.has_value());
        CHECK_THAT(*span.forward.unanchored_distance, WithinAbs(7.6, 1e-9));
    }

    SECTION("an empty preceding model layer does not skip to an older track") {
        const PathChain older = chain(0, { Vec2d(5., -1.), Vec2d(5., 1.) }, 0.4,
                                      erExternalPerimeter, 0);
        const PathChain empty_preceding { 1, false, {} };
        const PathChain upper = chain(2, { Vec2d(0., 0.), Vec2d(10., 0.) });
        const auto result = associate({ pin(0, 2, Vec2d(2., 0.)) },
                                      { older, empty_preceding, upper }, 0.01);
        const auto &span = *only_association(result).planned_material_unsupported_span;
        CHECK_FALSE(span.forward.span.has_value());
        CHECK_THAT(*span.forward.unanchored_distance, WithinAbs(7.6, 1e-9));
    }
}

TEST_CASE("Every model extrusion role may provide a lower anchor", "[OrganicSupportAssociation]")
{
    const std::array<std::pair<ExtrusionRole, ExtrusionSource>, 4> roles {{
        { erExternalPerimeter, ExtrusionSource::Perimeter },
        { erSolidInfill, ExtrusionSource::Fill },
        { erBridgeInfill, ExtrusionSource::Fill },
        { erInternalInfill, ExtrusionSource::Fill }
    }};
    for (size_t role_idx = 0; role_idx < roles.size(); ++role_idx) {
        DYNAMIC_SECTION("role " << int(roles[role_idx].first)) {
            const PathChain lower = chain(0, { Vec2d(5., -1.), Vec2d(5., 1.) }, 0.4,
                                            roles[role_idx].first, role_idx, roles[role_idx].second);
            const PathChain current = chain(1, { Vec2d(0., 0.), Vec2d(10., 0.) });
            const auto result = associate({ pin(0, 1, Vec2d(2., 0.)) }, { lower, current }, 0.01);
            const auto &span = *only_association(result).planned_material_unsupported_span;
            REQUIRE(span.forward.span.has_value());
            REQUIRE(span.forward.anchors.size() == 1);
            CHECK(span.forward.anchors.front().extrusion_id == lower.leaves.front().id);
        }
    }
}

TEST_CASE("Unsupported spans preserve tied and overlapping anchors", "[OrganicSupportAssociation]")
{
    const PathChain current = chain(1, { Vec2d(0., 0.), Vec2d(10., 0.) });

    SECTION("tracks tied at the same boundary are all retained") {
        const PathChain lower_perimeter = chain(0, { Vec2d(5., -1.), Vec2d(5., 1.) }, 0.4,
                                                erExternalPerimeter, 0);
        const PathChain lower_fill = chain(0, { Vec2d(5., -2.), Vec2d(5., 2.) }, 0.4,
                                           erSolidInfill, 1, ExtrusionSource::Fill);
        const auto result = associate({ pin(0, 1, Vec2d(2., 0.)) },
                                      { lower_perimeter, lower_fill, current }, 0.01);
        const auto &span = *only_association(result).planned_material_unsupported_span;
        REQUIRE(span.forward.span.has_value());
        CHECK(span.forward.anchors.size() == 2);
    }

    SECTION("overlap at the current pin returns zero") {
        const PathChain lower = chain(0, { Vec2d(2., -1.), Vec2d(2., 1.) }, 0.4,
                                        erExternalPerimeter, 0);
        const auto result = associate({ pin(0, 1, Vec2d(2., 0.)) }, { lower, current }, 0.01);
        const auto &span = *only_association(result).planned_material_unsupported_span;
        REQUIRE(span.forward.span.has_value());
        REQUIRE(span.backward.span.has_value());
        CHECK_THAT(*span.forward.span, WithinAbs(0., 1e-9));
        CHECK_THAT(*span.backward.span, WithinAbs(0., 1e-9));
    }
}

TEST_CASE("Unsupported spans find neighboring pins in both directions", "[OrganicSupportAssociation]")
{
    SECTION("open paths report anchors and unanchored endpoints") {
        const auto result = associate({ pin(0, 1, Vec2d(2., 0.)), pin(1, 1, Vec2d(5., 0.)) },
                                      { chain(1, { Vec2d(0., 0.), Vec2d(10., 0.) }) }, 0.01);
        const auto &first = *result[0].associations.front().planned_material_unsupported_span;
        REQUIRE(first.forward.span.has_value());
        CHECK_THAT(*first.forward.span, WithinAbs(2.2, 1e-9));
        CHECK(first.forward.anchors.front().contact_id == 1);
        CHECK_FALSE(first.backward.span.has_value());
        CHECK_THAT(*first.backward.unanchored_distance, WithinAbs(1.6, 1e-9));
    }

    SECTION("closed paths traverse at most once and may return to the same pin") {
        const PathChain loop = chain(1,
            { Vec2d(0., 0.), Vec2d(10., 0.), Vec2d(10., 10.), Vec2d(0., 10.), Vec2d(0., 0.) },
            0.4, erExternalPerimeter, 0, ExtrusionSource::Perimeter, true);
        const auto result = associate({ pin(0, 1, Vec2d(5., 0.)) }, { loop }, 0.01);
        const auto &span = *only_association(result).planned_material_unsupported_span;
        REQUIRE(span.forward.span.has_value());
        REQUIRE(span.backward.span.has_value());
        CHECK_THAT(*span.forward.span, WithinAbs(39.2, 1e-9));
        CHECK_THAT(*span.backward.span, WithinAbs(39.2, 1e-9));
        CHECK(span.forward.anchors.front().contact_id == 0);
    }
}

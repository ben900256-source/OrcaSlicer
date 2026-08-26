#include <catch2/catch_all.hpp>

#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MinAreaBoundingBox.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"
#include "nlohmann/json.hpp"

#include "test_helpers.hpp" // get access to init_print, etc
#include "test_utils.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

using namespace Slic3r::Test;
using namespace Slic3r;

namespace {

DynamicPrintConfig organic_contact_config(double spacing = 2., double density = 20.)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",                        1 },
        { "support_type",                         "tree(auto)" },
        { "support_style",                        "organic" },
        { "support_interface_top_layers",         0 },
        { "support_remove_small_overhang",         0 },
        { "bridge_no_support",                     0 },
        { "support_line_width",                    0.4 },
        { "tree_support_tip_diameter",             0.4 },
        { "tree_support_branch_diameter_organic",  2.0 },
        { "tree_support_branch_diameter_angle",    5.0 },
        { "tree_support_branch_distance_organic",  spacing },
        { "tree_support_top_rate",                 density },
    });
    return config;
}

using ContactSignature = std::tuple<coord_t, coord_t, coordf_t, coordf_t, coord_t, size_t>;

std::vector<ContactSignature> contact_signatures(const PrintObject &object)
{
    std::vector<ContactSignature> out;
    out.reserve(object.support_contacts().size());
    for (const SupportContact &contact : object.support_contacts())
        out.emplace_back(contact.position.x(), contact.position.y(), contact.support_tip_z,
                         contact.model_contact_z, contact.nominal_radius, contact.object_layer_id);
    return out;
}

double median_nearest_neighbour_distance(const std::vector<SupportContact> &contacts)
{
    std::vector<double> nearest;
    nearest.reserve(contacts.size());
    for (size_t i = 0; i < contacts.size(); ++i) {
        double best = std::numeric_limits<double>::max();
        for (size_t j = 0; j < contacts.size(); ++j) {
            if (i == j || contacts[i].object_layer_id != contacts[j].object_layer_id)
                continue;
            const double dx = unscale<double>(contacts[i].position.x() - contacts[j].position.x());
            const double dy = unscale<double>(contacts[i].position.y() - contacts[j].position.y());
            best = std::min(best, std::hypot(dx, dy));
        }
        if (best < std::numeric_limits<double>::max())
            nearest.push_back(best);
    }
    REQUIRE_FALSE(nearest.empty());
    std::sort(nearest.begin(), nearest.end());
    const size_t middle = nearest.size() / 2;
    return nearest.size() % 2 == 0 ? 0.5 * (nearest[middle - 1] + nearest[middle]) : nearest[middle];
}

struct TerminalFootprintStats {
    size_t count;
    double median_aspect_ratio;
    double max_aspect_ratio;
};

TerminalFootprintStats unobstructed_terminal_footprint_stats(const PrintObject &object)
{
    std::vector<double> ratios;
    for (const SupportContact &contact : object.support_contacts()) {
        const SupportLayer *contact_layer = nullptr;
        for (const SupportLayer *layer : object.support_layers())
            if (std::abs(layer->print_z - contact.support_tip_z) < EPSILON) {
                contact_layer = layer;
                break;
            }
        if (contact_layer == nullptr)
            continue;

        for (const ExPolygon &island : contact_layer->support_islands) {
            if (! island.contains(contact.position))
                continue;

            // Collision-clipped or branch-merged islands are not unobstructed
            // terminal footprints. A discretized circle is slightly under its
            // analytic area, so keep a symmetric tolerance around the nominal tip.
            const double nominal_area = PI * sqr(double(contact.nominal_radius));
            const double area_ratio   = std::abs(island.area()) / nominal_area;
            if (area_ratio < 0.9 || area_ratio > 1.1)
                break;

            // A terminal island shared by multiple tips has already merged and is
            // not an unobstructed footprint of one planned contact.
            const size_t contacts_in_island = std::count_if(
                object.support_contacts().begin(), object.support_contacts().end(),
                [&](const SupportContact &candidate) {
                    return std::abs(candidate.support_tip_z - contact.support_tip_z) < EPSILON &&
                           island.contains(candidate.position);
                });
            if (contacts_in_island != 1)
                break;

            const MinAreaBoundigBox box(island);
            const double short_side = double(std::min(box.width(), box.height()));
            const double long_side  = double(std::max(box.width(), box.height()));
            if (short_side > 0.)
                ratios.push_back(long_side / short_side);
            break;
        }
    }

    if (ratios.empty())
        return { 0, 0., 0. };
    std::sort(ratios.begin(), ratios.end());
    const size_t middle = ratios.size() / 2;
    const double median = ratios.size() % 2 == 0 ? 0.5 * (ratios[middle - 1] + ratios[middle]) : ratios[middle];
    return { ratios.size(), median, ratios.back() };
}

double support_volume(const PrintObject &object)
{
    double volume = 0.;
    for (const SupportLayer *layer : object.support_layers())
        volume += layer->support_fills.total_volume();
    return volume;
}

double support_path_length(const PrintObject &object)
{
    double length = 0.;
    for (const SupportLayer *layer : object.support_layers())
        for (const ExtrusionEntity *entity : layer->support_fills.flatten().entities)
            if (! entity->is_collection())
                length += entity->length();
    return length;
}

bool round_terminal_layers_are_continuous(const PrintObject &object)
{
    const ConstSupportLayerPtrsAdaptor layers = object.support_layers();
    size_t checked = 0;
    for (const SupportContact &contact : object.support_contacts()) {
        size_t layer_idx = 0;
        while (layer_idx < layers.size() && std::abs(layers[layer_idx]->print_z - contact.support_tip_z) >= EPSILON)
            ++ layer_idx;
        if (layer_idx < 2 || layer_idx == layers.size())
            return false;

        const ExPolygon *upper = nullptr;
        for (const ExPolygon &island : layers[layer_idx]->support_islands)
            if (island.contains(contact.position)) {
                upper = &island;
                break;
            }
        if (upper == nullptr)
            continue;

        const double nominal_area = PI * sqr(double(contact.nominal_radius));
        const double area_ratio   = std::abs(upper->area()) / nominal_area;
        if (area_ratio < 0.9 || area_ratio > 1.1)
            continue;
        ++ checked;

        // The two replaced slices must overlap each other, and the lower one
        // must overlap the untouched tube on the next layer down.
        for (size_t depth = 1; depth <= 2; ++ depth) {
            const ExPolygon *overlapping = nullptr;
            for (const ExPolygon &island : layers[layer_idx - depth]->support_islands)
                if (! intersection_ex(ExPolygons{ *upper }, ExPolygons{ island }).empty()) {
                    overlapping = &island;
                    break;
                }
            if (overlapping == nullptr)
                return false;
            upper = overlapping;
        }
    }
    return checked >= 6;
}

} // namespace

TEST_CASE("Three raft layers are created", "[SupportMaterial]")
{
	Slic3r::Print print;
	Slic3r::Test::init_and_process_print({ cube(20) }, print, {
        { "enable_support", 1 },
        { "raft_layers",    3 }
		});
    REQUIRE(print.objects().front()->support_layers().size() == 3);
}

TEST_CASE("Enforced support layers are generated", "[SupportMaterial]")
{
    // enforce_support_layers forces support on the first N layers even with support off.
    Slic3r::Print baseline;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, baseline, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    REQUIRE(baseline.objects().front()->support_layers().empty());

    Slic3r::Print enforced;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, enforced, {
        { "enable_support",         0 },
        { "enforce_support_layers", 100 }
    });
    REQUIRE(enforced.objects().front()->support_layers().size() > 0);
}

SCENARIO("Support layer Z honors contact distance", "[SupportMaterial]")
{
    // Box h = 20mm, hole bottom at 5mm, hole height 10mm (top edge at 15mm).
    TriangleMesh mesh = Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole);
    mesh.rotate_x(float(M_PI / 2));

	auto check = [](Slic3r::Print &print, bool &first_support_layer_height_ok, bool &layer_height_minimum_ok, bool &layer_height_maximum_ok)
	{
        ConstSupportLayerPtrsAdaptor support_layers = print.objects().front()->support_layers();

		first_support_layer_height_ok = support_layers.front()->print_z == print.config().initial_layer_print_height.value;

		layer_height_minimum_ok = true;
		layer_height_maximum_ok = true;
		double min_layer_height = print.config().min_layer_height.values.front();
		double max_layer_height = print.config().nozzle_diameter.values.front();
		if (print.config().max_layer_height.values.front() > EPSILON)
			max_layer_height = std::min(max_layer_height, print.config().max_layer_height.values.front());
		for (size_t i = 1; i < support_layers.size(); ++ i) {
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z < min_layer_height - EPSILON)
				layer_height_minimum_ok = false;
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z > max_layer_height + EPSILON)
				layer_height_maximum_ok = false;
		}
	};

    GIVEN("A print object having one modelObject") {
        WHEN("Layer height = 0.2 and first layer height = 0.4") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.4 },
                { "dont_support_bridges",       false },
			});
			bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
        WHEN("Layer height = 0.2 and first layer height = 0.3") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.3 },
                { "dont_support_bridges",       false },
            });
            bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
    }
}

// extrude_support once held a `static` lambda capturing `this`, so a second export in the
// same process dereferenced a returned stack frame (ASan: stack-use-after-return).
TEST_CASE("Support G-code emission survives a second slice in the same process", "[SupportMaterial][Regression]")
{
    const std::string first = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(first, "support").empty());

    const std::string second = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(second, "support").empty());
}

TEST_CASE("Organic supports expose deterministic realized contacts", "[SupportMaterial][OrganicContacts]")
{
    const DynamicPrintConfig config = organic_contact_config();

    Print first;
    Model first_model;
    init_print({ TestMesh::overhang }, first, first_model, config);
    first.process();

    const PrintObject &first_object = *first.objects().front();
    const auto first_contacts = contact_signatures(first_object);
    REQUIRE_FALSE(first_contacts.empty());
    CHECK(std::is_sorted(first_contacts.begin(), first_contacts.end()));
    for (const SupportContact &contact : first_object.support_contacts()) {
        CHECK(contact.nominal_radius == scaled<coord_t>(0.2));
        CHECK(contact.support_tip_z < contact.model_contact_z);
        CHECK(contact.object_layer_id < first_object.layer_count());
    }
    const TreeSupport3D::TreeSupportMeshGroupSettings mesh_settings(first_object);
    const TreeSupport3D::TreeSupportSettings tree_settings(mesh_settings, first_object.slicing_parameters());
    coord_t                            previous_radius = 0;
    TreeSupport3D::SupportElementState radius_state{};
    for (size_t distance_to_top = 0; distance_to_top <= tree_settings.tip_layers; ++distance_to_top) {
        radius_state.distance_to_top          = uint32_t(distance_to_top);
        radius_state.effective_radius_height = uint32_t(distance_to_top);
        const coord_t radius = TreeSupport3D::support_element_radius(tree_settings, radius_state);
        CHECK(radius >= previous_radius);
        previous_radius = radius;
    }
    CHECK(previous_radius == tree_settings.branch_radius);
    CHECK(tree_settings.tip_layers * unscale<double>(tree_settings.layer_height) <= 1.5);
    CHECK(first.validate_support_contact_export().empty());

    const std::string support_gcode = gcode(first);
    CHECK_FALSE(support_gcode.empty());
    CHECK_FALSE(layers_with_role(support_gcode, "support").empty());

    Print second;
    Model second_model;
    init_print({ TestMesh::overhang }, second, second_model, config);
    second.process();
    CHECK(contact_signatures(*second.objects().front()) == first_contacts);
}

TEST_CASE("Organic contact snapshots exclude unrealized branch endpoints", "[SupportMaterial][OrganicContacts]")
{
    Print print;
    Model model;
    init_print({ TestMesh::overhang }, print, model, organic_contact_config());
    print.process();

    PrintObject &object = *print.get_object(0);
    const TreeSupport3D::TreeSupportMeshGroupSettings mesh_settings(object);
    const TreeSupport3D::TreeSupportSettings tree_settings(mesh_settings, object.slicing_parameters());
    const TreeSupport3D::LayerIndex target_height = 5;
    REQUIRE(size_t(target_height) + tree_settings.z_distance_top_layers + 1 <
            object.layer_count() + tree_settings.raft_layers.size());

    TreeSupport3D::SupportElementState endpoint{};
    endpoint.target_height           = target_height;
    endpoint.target_position         = Point(scale_(10.), scale_(10.));
    endpoint.next_position           = endpoint.target_position;
    endpoint.layer_idx               = target_height;
    endpoint.effective_radius_height = 0;
    endpoint.distance_to_top         = 0;
    endpoint.result_on_layer         = endpoint.target_position;
    endpoint.increased_to_model_radius = 0;
    endpoint.elephant_foot_increases  = 0.;
    endpoint.dont_move_until          = 0;

    TreeSupport3D::SupportElements elements;
    elements.emplace_back(endpoint, Polygons{}); // realized and connected
    elements.emplace_back(endpoint, Polygons{}); // disconnected
    elements.emplace_back(endpoint, Polygons{}); // pruned
    elements.back().state.deleted = true;
    elements.emplace_back(endpoint, Polygons{}); // not a terminal node
    elements.back().parents.push_back(0);

    std::vector<std::pair<TreeSupport3D::SupportElement*, int>> endpoints = {
        { &elements[0], 0 },
        { &elements[1], -1 },
        { &elements[2], 0 },
        { &elements[3], 0 },
    };
    TreeSupport snapshotter(object, object.slicing_parameters());
    snapshotter.store_organic_support_contacts(endpoints, tree_settings);

    REQUIRE(object.support_contacts().size() == 1);
    CHECK(object.support_contacts().front().position == endpoint.target_position);
}

TEST_CASE("Organic contacts survive invalidation and slicedata cache round trips", "[SupportMaterial][OrganicContacts]")
{
    DynamicPrintConfig config = organic_contact_config();
    Print              print;
    Model              model;
    init_print({ TestMesh::overhang }, print, model, config);
    print.process();

    PrintObject &object = *print.get_object(0);
    const auto expected = contact_signatures(object);
    REQUIRE_FALSE(expected.empty());

    config.set_deserialize_strict({ { "support_threshold_angle", 31 } });
    print.apply(model, config);
    CHECK_FALSE(object.is_step_done(posSupportMaterial));
    CHECK(contact_signatures(object) == expected);

    ScopedTemporaryDir temporary("support-contact-cache");
    const boost::filesystem::path cache = temporary.path() / "slicedata";
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);

    const ModelInstance *model_instance = object.instances().front().model_instance;
    const size_t identify_id = model_instance->loaded_id > 0 ? model_instance->loaded_id : model_instance->id().id;
    std::ifstream cache_stream((cache / ("obj_" + std::to_string(identify_id) + ".json")).string());
    const nlohmann::json cached_object = nlohmann::json::parse(cache_stream);
    REQUIRE(cached_object.is_object());
    REQUIRE(cached_object.at("support_contacts").is_array());
    for (const nlohmann::json &cached_contact : cached_object.at("support_contacts"))
        REQUIRE(cached_contact.is_object());

    REQUIRE(print.load_cached_data(cache.string()) == 0);
    CHECK(contact_signatures(object) == expected);
}

TEST_CASE("Support contact reports apply instance transforms and summarize nominal area", "[SupportMaterial][OrganicContacts]")
{
    const DynamicPrintConfig config = organic_contact_config();
    Print print;
    Model model;
    init_print({ TestMesh::overhang }, print, model, config);

    ModelObject &model_object = *model.objects.front();
    const Vec3d first_offset = model_object.instances.front()->get_offset();
    model_object.add_instance()->set_offset(first_offset + Vec3d(40., 0., 0.));
    print.apply(model, config);
    print.process();

    const size_t contacts_per_instance = print.objects().front()->support_contacts().size();
    REQUIRE(contacts_per_instance > 0);

    ScopedTemporaryFile report_file(".support-contacts.json");
    print.export_support_contacts(report_file.string(), 3);
    std::ifstream report_stream(report_file.string());
    const nlohmann::json report = nlohmann::json::parse(report_stream);

    CHECK(report.at("schema_version") == 1);
    CHECK(report.at("plate") == 3);
    CHECK(report.at("coordinate_space") == "plate-local");
    CHECK(report.at("units").at("position") == "mm");
    CHECK(report.at("units").at("area") == "mm^2");
    CHECK(report.at("contact_count") == 2 * contacts_per_instance);
    REQUIRE(report.at("instances").size() == 2);
    CHECK(report.at("instances").at(0).at("contact_count") == contacts_per_instance);
    CHECK(report.at("instances").at(1).at("contact_count") == contacts_per_instance);

    const auto &first_contact  = report.at("instances").at(0).at("contacts").at(0);
    const auto &second_contact = report.at("instances").at(1).at("contacts").at(0);
    CHECK(first_contact.contains("support_tip_z"));
    CHECK(first_contact.contains("model_contact_z"));
    CHECK(first_contact.contains("nominal_radius"));
    CHECK(first_contact.contains("object_layer_id"));
    CHECK_THAT(second_contact.at("x").get<double>() - first_contact.at("x").get<double>(),
               Catch::Matchers::WithinAbs(40., 1e-9));
    CHECK_THAT(second_contact.at("y").get<double>(),
               Catch::Matchers::WithinAbs(first_contact.at("y").get<double>(), 1e-9));

    const double expected_area = 2. * contacts_per_instance * PI * 0.2 * 0.2;
    CHECK_THAT(report.at("total_nominal_circular_area").get<double>(),
               Catch::Matchers::WithinAbs(expected_area, 1e-9));
}

TEST_CASE("Support contact export rejects non-discrete support configurations", "[SupportMaterial][OrganicContacts]")
{
    SECTION("top interfaces") {
        DynamicPrintConfig config = organic_contact_config();
        config.set_deserialize_strict({ { "support_interface_top_layers", 1 } });
        Print print;
        Model model;
        init_print({ TestMesh::overhang }, print, model, config);
        print.process();
        CHECK(print.validate_support_contact_export().find("top interface layers") != std::string::npos);
        CHECK(print.objects().front()->support_contacts().empty());
        CHECK_FALSE(layers_with_role(gcode(print), "support").empty());
    }

    SECTION("non-Organic tree style") {
        DynamicPrintConfig config = organic_contact_config();
        config.set_deserialize_strict({ { "support_style", "tree_slim" } });
        Print print;
        Model model;
        init_print({ TestMesh::overhang }, print, model, config);
        CHECK(print.validate_support_contact_export().find("only for Organic") != std::string::npos);
    }

    SECTION("support disabled") {
        Print print;
        Model model;
        init_print({ cube(20) }, print, model);
        print.process();
        REQUIRE(print.validate_support_contact_export().empty());

        ScopedTemporaryFile report_file(".support-contacts.json");
        print.export_support_contacts(report_file.string(), 1);
        std::ifstream report_stream(report_file.string());
        const nlohmann::json report = nlohmann::json::parse(report_stream);
        CHECK(report.at("contact_count") == 0);
        CHECK(report.at("instances").at(0).at("contacts").empty());
    }
}

TEST_CASE("Existing Organic controls realize sparse contact spacing", "[SupportMaterial][OrganicContacts]")
{
    struct Result {
        size_t count;
        double median_distance;
    };
    auto slice_coupon = [](double spacing, double density) {
        Print print;
        Model model;
        init_print({ load_model("support_contact_coupon.obj") }, print, model, organic_contact_config(spacing, density));
        print.process();
        const std::vector<SupportContact> contacts = print.objects().front()->support_contacts();
        REQUIRE(contacts.size() > 1);
        return Result{ contacts.size(), median_nearest_neighbour_distance(contacts) };
    };

    const Result spacing_2 = slice_coupon(2., 20.);
    const Result spacing_3 = slice_coupon(3., 13.33);
    const Result spacing_4 = slice_coupon(4., 10.);
    CAPTURE(spacing_2.count, spacing_2.median_distance,
            spacing_3.count, spacing_3.median_distance,
            spacing_4.count, spacing_4.median_distance);

    CHECK(spacing_2.count > spacing_3.count);
    CHECK(spacing_3.count > spacing_4.count);
    CHECK_THAT(spacing_2.median_distance, Catch::Matchers::WithinAbs(2., 0.4));
    CHECK_THAT(spacing_3.median_distance, Catch::Matchers::WithinAbs(3., 0.6));
    CHECK_THAT(spacing_4.median_distance, Catch::Matchers::WithinAbs(4., 0.8));
}

TEST_CASE("Round Organic Pin tips meet terminal footprint limits", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    struct SliceResult {
        TerminalFootprintStats footprints;
        double                 support_volume;
        double                 support_path_length;
        bool                   terminal_layers_are_continuous;
    };
    CHECK_FALSE(DynamicPrintConfig::full_print_config().opt_bool("tree_support_round_tip"));

    auto slice_coupon = [](double layer_height, double top_gap, bool round_tip) {
        DynamicPrintConfig config = organic_contact_config(3., 13.33);
        config.set_deserialize_strict({
            { "layer_height",                  layer_height },
            { "initial_layer_print_height",    layer_height },
            { "support_top_z_distance",        top_gap },
            { "tree_support_angle_slow",       25. },
            { "tree_support_round_tip",        round_tip },
        });

        TriangleMesh coupon = load_model("support_contact_coupon.obj");
        coupon.scale(Vec3f(1.f, 1.f, 0.25f));
        Print print;
        Model model;
        init_print({ std::move(coupon) }, print, model, config);
        print.process();
        const PrintObject &object = *print.objects().front();
        const TerminalFootprintStats footprints = unobstructed_terminal_footprint_stats(object);
        const double volume = support_volume(object);
        return SliceResult{ footprints, volume, support_path_length(object), round_terminal_layers_are_continuous(object) };
    };

    for (const auto &[layer_height, top_gap] : std::array<std::pair<double, double>, 2>{
             std::pair{ 0.05, 0.05 }, std::pair{ 0.06, 0.06 } }) {
        DYNAMIC_SECTION("layer height " << layer_height) {
            const SliceResult baseline  = slice_coupon(layer_height, top_gap, false);
            const SliceResult candidate = slice_coupon(layer_height, top_gap, true);
            CAPTURE(baseline.footprints.count, baseline.footprints.median_aspect_ratio,
                    baseline.footprints.max_aspect_ratio, baseline.support_volume, baseline.support_path_length,
                    candidate.footprints.count, candidate.footprints.median_aspect_ratio,
                    candidate.footprints.max_aspect_ratio, candidate.support_volume, candidate.support_path_length,
                    candidate.terminal_layers_are_continuous);

            REQUIRE(candidate.footprints.count >= 6);
            CHECK(candidate.terminal_layers_are_continuous);
            CHECK(candidate.footprints.median_aspect_ratio <= 1.05);
            CHECK(candidate.footprints.max_aspect_ratio <= 1.10);
            CHECK(candidate.support_volume <= 1.15 * baseline.support_volume);
            // Support speed is unchanged, so extrusion path length is the direct
            // support-extrusion-time term without estimator startup noise.
            CHECK(candidate.support_path_length <= 1.15 * baseline.support_path_length);
        }
    }
}

TEST_CASE("Organic Pin contact Z matches one two and three layer gaps", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    const double layer_height = GENERATE(0.05, 0.06);
    const int gap_layers = GENERATE(1, 2, 3);
    const double top_gap = gap_layers * layer_height;
    CAPTURE(layer_height, gap_layers, top_gap);

    DynamicPrintConfig config = organic_contact_config(3., 13.33);
    config.set_deserialize_strict({
        { "layer_height",               layer_height },
        { "initial_layer_print_height", layer_height },
        { "support_top_z_distance",     top_gap },
        { "tree_support_round_tip",     true },
    });

    TriangleMesh coupon = load_model("support_contact_coupon.obj");
    coupon.scale(Vec3f(1.f, 1.f, 0.25f));
    Print print;
    Model model;
    init_print({ std::move(coupon) }, print, model, config);
    print.process();

    const std::vector<SupportContact> &contacts = print.objects().front()->support_contacts();
    REQUIRE_FALSE(contacts.empty());
    for (const SupportContact &contact : contacts)
        CHECK_THAT(contact.model_contact_z - contact.support_tip_z,
                   Catch::Matchers::WithinAbs(top_gap, 1e-6));
}

TEST_CASE("Prusa XL miniature profiles realize one-layer Pin contacts on physical tool 2", "[SupportMaterial][OrganicContacts][MiniaturePin]")
{
    static constexpr const char *machine_name = "Prusa XL 5T T2 0.25 nozzle (others 0.4)";
    static constexpr const char *filament_name = "Prusa Generic Miniature PLA @XL 5T";
    const std::array<const char *, 2> process_names {
        "0.05mm Miniature Ultra Detail + Pin @Prusa XL 5T T2 0.25",
        "0.06mm Miniature Balanced + Pin @Prusa XL 5T T2 0.25"
    };

    PresetBundle bundle;
    bundle.set_is_validation_mode(true);
    REQUIRE(bundle.load_vendor_configs_from_json(
                PROFILES_DIR, "Prusa", PresetBundle::LoadSystem,
                ForwardCompatibilitySubstitutionRule::EnableSilent).second > 0);

    Preset *machine = bundle.printers.find_preset(machine_name, false, true);
    Preset *filament = bundle.filaments.find_preset(filament_name, false, true);
    REQUIRE(machine != nullptr);
    REQUIRE(filament != nullptr);

    for (const char *process_name : process_names) {
        DYNAMIC_SECTION(process_name) {
            Preset *process = bundle.prints.find_preset(process_name, false, true);
            REQUIRE(process != nullptr);

            DynamicPrintConfig project_config = bundle.project_config;
            project_config.option<ConfigOptionInts>("filament_map", true)->values = { 1, 1, 1, 1, 1 };
            project_config.option<ConfigOptionInts>("filament_nozzle_map", true)->values = { 0, 0, 0, 0, 0 };
            project_config.option<ConfigOptionInts>("filament_volume_map", true)->values = { 0, 0, 0, 0, 0 };
            project_config.option<ConfigOptionStrings>("filament_colour", true)->values = {
                "#26A69A", "#26A69A", "#26A69A", "#26A69A", "#26A69A"
            };
            project_config.option<ConfigOptionFloats>("flush_multiplier", true)->values = { 0.3 };
            project_config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values.assign(25, 0.);
            std::vector<Preset> selected_filaments(5, *filament);
            DynamicPrintConfig config = PresetBundle::construct_full_config(
                *machine, *process, project_config, selected_filaments, true, std::nullopt);
            CHECK(config.opt_bool("tree_support_round_tip"));
            CHECK_THAT(config.opt_float("tree_support_angle_slow"),
                       Catch::Matchers::WithinAbs(25.0, 1e-9));
            const double layer_height = config.opt_float("layer_height");
            CHECK_THAT(config.opt_float("support_top_z_distance"),
                       Catch::Matchers::WithinAbs(layer_height, 1e-9));

            TriangleMesh coupon = load_model("support_contact_coupon.obj");
            coupon.scale(Vec3f(1.f, 1.f, 0.25f));
            Print print;
            Model model;
            init_print({ std::move(coupon) }, print, model, config);
            print.process();

            const PrintObject &object = *print.objects().front();
            REQUIRE(object.support_contacts().size() > 1);
            for (const SupportContact &contact : object.support_contacts())
                CHECK_THAT(contact.model_contact_z - contact.support_tip_z,
                           Catch::Matchers::WithinAbs(layer_height, 1e-6));
            const std::vector<ContactSignature> signatures = contact_signatures(object);
            CHECK(std::is_sorted(signatures.begin(), signatures.end()));
            CHECK_THAT(median_nearest_neighbour_distance(object.support_contacts()),
                       Catch::Matchers::WithinAbs(3., 0.6));
            CHECK(print.validate_support_contact_export().empty());

            TriangleMesh repeated_coupon = load_model("support_contact_coupon.obj");
            repeated_coupon.scale(Vec3f(1.f, 1.f, 0.25f));
            Print repeated_print;
            Model repeated_model;
            init_print({ std::move(repeated_coupon) }, repeated_print, repeated_model, config);
            repeated_print.process();
            CHECK(contact_signatures(*repeated_print.objects().front()) == signatures);

            ScopedTemporaryFile report_file(".support-contacts.json");
            print.export_support_contacts(report_file.string(), 0);
            std::ifstream report_stream(report_file.string());
            const nlohmann::json report = nlohmann::json::parse(report_stream);
            CHECK(report.at("schema_version") == 1);
            CHECK(report.at("contact_count") == object.support_contacts().size());

            ScopedTemporaryFile repeated_report_file(".support-contacts.json");
            repeated_print.export_support_contacts(repeated_report_file.string(), 0);
            std::ifstream repeated_report_stream(repeated_report_file.string());
            nlohmann::json repeated_report = nlohmann::json::parse(repeated_report_stream);
            nlohmann::json normalized_report = report;
            for (nlohmann::json *candidate : { &normalized_report, &repeated_report }) {
                for (nlohmann::json &instance : candidate->at("instances")) {
                    instance.erase("object_id");
                    instance.erase("instance_id");
                }
            }
            CHECK(repeated_report == normalized_report);

            TriangleMesh miniature_fixture = mesh(TestMesh::overhang);
            miniature_fixture.scale(Vec3f(0.2f, 0.2f, 0.25f));
            Print miniature_print;
            Model miniature_model;
            init_print({ std::move(miniature_fixture) }, miniature_print, miniature_model, config);
            miniature_print.process();
            CHECK_FALSE(miniature_print.objects().front()->support_contacts().empty());
            CHECK(miniature_print.validate_support_contact_export().empty());

            const std::string generated = gcode(print);
            CHECK_FALSE(generated.empty());
            CHECK_FALSE(layers_with_role(generated, "support").empty());
            CHECK(layers_with_role(generated, "support interface").empty());
            CHECK(generated.find("M900 K0.14") != std::string::npos);
            CHECK(generated.find("M572 S0.12") != std::string::npos);

            std::set<int> selected_tools;
            std::istringstream lines(generated);
            for (std::string line; std::getline(lines, line);) {
                if (line.size() < 2 || line.front() != 'T' || !std::isdigit(static_cast<unsigned char>(line[1])))
                    continue;
                size_t consumed = 0;
                const int tool = std::stoi(line.substr(1), &consumed);
                if (consumed > 0 && (consumed + 1 == line.size() || std::isspace(static_cast<unsigned char>(line[consumed + 1]))))
                    selected_tools.insert(tool);
            }
            CHECK(selected_tools == std::set<int>{ 1 });
            REQUIRE(print.get_filament_maps().size() >= 2);
            CHECK(print.get_filament_maps()[1] == 2);
        }
    }
}

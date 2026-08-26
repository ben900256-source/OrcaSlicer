#include <catch2/catch_all.hpp>

#include <boost/filesystem.hpp>
#include <array>
#include <fstream>

#include "nlohmann/json.hpp"

#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/AppConfig.hpp"

#include "test_utils.hpp"

using namespace Slic3r;

namespace {

namespace fs = boost::filesystem;

void write_print_preset(const DynamicPrintConfig &default_config, const fs::path &file, const std::string &name, const std::string &inherits = {})
{
    DynamicPrintConfig config(default_config);
    config.option<ConfigOptionString>("print_settings_id", true)->value = name;
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;

    fs::create_directories(file.parent_path());
    config.save_to_json(file.string(), name, "User", "1.0.0");
}

// Write a preset json carrying a name and an "inherits" value, using the given collection's
// default config so it loads back into that collection. Works for any preset type.
void write_preset_with_inherits(const DynamicPrintConfig &default_config, const fs::path &file,
                                const std::string &name, const std::string &inherits)
{
    DynamicPrintConfig config(default_config);
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;

    fs::create_directories(file.parent_path());
    config.save_to_json(file.string(), name, "User", "1.0.0");
}

// Add an in-memory preset (no file) with the given inherits value (empty => root preset).
Preset &add_inmemory_preset(PresetCollection &coll, const std::string &name, const std::string &inherits = {})
{
    DynamicPrintConfig config(coll.default_preset().config);
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;
    return coll.load_preset(std::string(), name, config, /*select=*/false);
}

// Mark an already-loaded preset as renamed from one or more former names.
void set_renamed_from(PresetCollection &coll, const std::string &preset_name, std::vector<std::string> old_names)
{
    for (auto it = coll.begin(); it != coll.end(); ++it)
        if (it->name == preset_name)
            it->renamed_from = std::move(old_names);
}

// A standalone print preset collection that exposes the protected rename-map builder, so a
// renamed_from scenario can be set up without the full system-profile load pipeline.
// (PresetCollection is non-copyable - it holds a mutex - so it is constructed directly with
// the same type/keys/defaults PresetBundle uses for its print collection.)
struct RenameTestCollection : public PresetCollection
{
    RenameTestCollection()
        : PresetCollection(Preset::TYPE_PRINT, Preset::print_options(),
                           static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()))
    {}
    using PresetCollection::update_map_system_profile_renamed;
};

} // namespace

TEST_CASE("Preset identity is canonicalized from load path", "[Preset][Identity]")
{
    ScopedTemporaryDir         temp_dir;
    PresetBundle               bundle;
    PresetsConfigSubstitutions substitutions;

    write_print_preset(bundle.prints.default_preset().config, temp_dir.path() / PRESET_PRINT_NAME / "User.json", "User");
    write_print_preset(bundle.prints.default_preset().config, temp_dir.path() / PRESET_LOCAL_DIR / "bundle-1" / PRESET_PRINT_NAME / "LocalBundle.json", "LocalBundle");
    write_print_preset(bundle.prints.default_preset().config, temp_dir.path() / PRESET_SUBSCRIBED_DIR / "remote-1" / PRESET_PRINT_NAME / "Subscribed.json", "Subscribed");

    bundle.prints.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.load_presets((temp_dir.path() / PRESET_LOCAL_DIR / "bundle-1").string(), PRESET_PRINT_NAME, substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.load_presets((temp_dir.path() / PRESET_SUBSCRIBED_DIR / "remote-1").string(), PRESET_PRINT_NAME, substitutions, ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *root_user = bundle.prints.find_preset("User");
    REQUIRE(root_user != nullptr);
    CHECK(root_user->name == "User");
    CHECK_FALSE(root_user->is_from_bundle());

    const Preset *local_bundle = bundle.prints.find_preset("_local/bundle-1/LocalBundle");
    REQUIRE(local_bundle != nullptr);
    CHECK(local_bundle->name == "_local/bundle-1/LocalBundle");
    CHECK(local_bundle->is_from_bundle());

    const Preset *subscribed = bundle.prints.find_preset("_subscribed/remote-1/Subscribed");
    REQUIRE(subscribed != nullptr);
    CHECK(subscribed->name == "_subscribed/remote-1/Subscribed");
    CHECK(subscribed->is_from_bundle());
}

TEST_CASE("Legacy bundle import without bundle metadata stays in the user preset directory", "[Preset][Identity]")
{
    ScopedTemporaryDir temp_dir;
    PresetBundle  bundle;

    PresetsConfigSubstitutions substitutions;
    std::vector<std::string>   result;
    int                        overwrite = 0;
    std::string                file      = (temp_dir.path() / "legacy-bundle" / "Imported.json").string();
    const fs::path             user_root = temp_dir.path() / "user";

    write_print_preset(bundle.prints.default_preset().config, file, "Imported");
    fs::create_directories(user_root);
    bundle.prints.update_user_presets_directory(user_root.string(), PRESET_PRINT_NAME);

    REQUIRE(bundle.import_json_presets(
        substitutions,
        file,
        [](std::string const &) { return 1; },
        ForwardCompatibilitySubstitutionRule::Disable,
        overwrite,
        result));

    const Preset *imported = bundle.prints.find_preset("Imported");
    REQUIRE(imported != nullptr);
    CHECK(imported->name == "Imported");
    CHECK(imported->bundle_id.empty());
    CHECK_FALSE(imported->is_from_bundle());
    // Detached user presets (no inherits) are saved in the "base" subfolder of the user preset root.
    CHECK(fs::equivalent(fs::path(imported->file).parent_path().parent_path(), user_root / PRESET_PRINT_NAME));
}

TEST_CASE("Current vendor type tolerates missing printer model", "[Preset][Bundle]")
{
    PresetBundle bundle;

    VendorProfile orca_vendor; orca_vendor.id = "ORCA";
    VendorProfile::PrinterModel model;
    model.name = "Orca Test";
    orca_vendor.models.emplace_back(model);
    bundle.vendors.emplace("ORCA", std::move(orca_vendor));

    bundle.printers.get_edited_preset().config.erase("printer_model");

    CHECK(bundle.get_current_vendor_type() == VendorType::Unknown);
}

TEST_CASE("A malformed entry in a vendor's preset list is counted, not thrown", "[Preset][Bundle]")
{
    ScopedTemporaryDir dir;

    // A bare number where the list wants an object. An array element has no key,
    // so reporting one as if it did throws nlohmann's invalid_iterator - which is
    // not a parse_error, and escapes the catch around the vendor profile parse.
    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[123,)"
        << R"({"name":"0.20mm Standard @Acme","sub_path":"process/standard.json"}]})";
    fs::create_directories(dir.path() / "Acme" / "process");
    std::ofstream((dir.path() / "Acme" / "process" / "standard.json").string())
        << R"({"type":"process","name":"0.20mm Standard @Acme","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";

    PresetBundle bundle;
    size_t       loaded = 0;
    REQUIRE_NOTHROW(loaded = bundle.load_vendor_configs_from_json(
                        dir.path().string(), "Acme", PresetBundle::LoadSystem,
                        ForwardCompatibilitySubstitutionRule::EnableSilent).second);

    CHECK(bundle.error_count() > 0);   // the malformed element was counted
    CHECK(loaded == 1);                // the well-formed one beside it still loaded
}

TEST_CASE("Printer extruder count tolerates missing nozzle diameter", "[Preset][Bundle]")
{
    PresetBundle bundle;
    DynamicPrintConfig& config = bundle.printers.get_edited_preset().config;

    config.erase("nozzle_diameter");
    CHECK(bundle.get_printer_extruder_count() == 1);

    config.set_key_value("nozzle_diameter", new ConfigOptionFloats());
    CHECK(bundle.get_printer_extruder_count() == 1);

    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({ 0.4, 0.6 }));
    CHECK(bundle.get_printer_extruder_count() == 2);
}

TEST_CASE("find_preset resolves a system preset's renamed_from", "[Preset][Rename]")
{
    RenameTestCollection coll;

    // "New Process" is the current preset; it was renamed from "Old Process".
    add_inmemory_preset(coll, "New Process");
    set_renamed_from(coll, "New Process", { "Old Process" });
    coll.update_map_system_profile_renamed();

    // The rename map knows the old name...
    const std::string *renamed = coll.get_preset_name_renamed("Old Process");
    REQUIRE(renamed != nullptr);
    CHECK(*renamed == "New Process");

    // ...and plain find_preset() now follows it (the core of this PR; previously this
    // resolution lived only in find_preset2 and a few call sites).
    const Preset *resolved = coll.find_preset("Old Process");
    REQUIRE(resolved != nullptr);
    CHECK(resolved->name == "New Process");

    // A genuinely unknown name still returns null (no spurious match).
    CHECK(coll.find_preset("Totally Unknown") == nullptr);

    // A child that still inherits the OLD name resolves through the runtime walker,
    // which uses plain find_preset().
    Preset       &child  = add_inmemory_preset(coll, "Child Process", "Old Process");
    const Preset *parent = coll.get_preset_parent(child);
    REQUIRE(parent != nullptr);
    CHECK(parent->name == "New Process");
}

TEST_CASE("find_preset resolves a preset renamed more than once", "[Preset][Rename]")
{
    RenameTestCollection coll;

    // "New Process" was renamed twice, so it carries both former names in renamed_from.
    add_inmemory_preset(coll, "New Process");
    set_renamed_from(coll, "New Process", { "Original Process", "Old Process" });
    coll.update_map_system_profile_renamed();

    // Each historical name resolves to the current preset.
    for (const char *old_name : { "Original Process", "Old Process" }) {
        INFO("resolving old name: " << old_name);
        const std::string *renamed = coll.get_preset_name_renamed(old_name);
        REQUIRE(renamed != nullptr);
        CHECK(*renamed == "New Process");

        const Preset *resolved = coll.find_preset(old_name);
        REQUIRE(resolved != nullptr);
        CHECK(resolved->name == "New Process");
    }

    // A child inheriting either former name resolves through the runtime walker.
    Preset &child = add_inmemory_preset(coll, "Child Process", "Original Process");
    REQUIRE(coll.get_preset_parent(child) != nullptr);
    CHECK(coll.get_preset_parent(child)->name == "New Process");
}

TEST_CASE("find_preset2 auto-matches removed Generic vendor profiles to the library", "[Preset][Rename]")
{
    PresetBundle bundle;

    // The OrcaFilamentLibrary replacement that removed empty "<vendor> Generic" profiles map to.
    add_inmemory_preset(bundle.filaments, "Generic PLA @System");

    // Plain lookups do NOT fuzzy-match a removed vendor profile.
    CHECK(bundle.filaments.find_preset("Voron Generic PLA") == nullptr);
    CHECK(bundle.filaments.find_preset2("Voron Generic PLA", /*auto_match=*/false) == nullptr);

    // With auto_match, the removed "Voron Generic PLA" resolves to "Generic PLA @System".
    const Preset *matched = bundle.filaments.find_preset2("Voron Generic PLA", /*auto_match=*/true);
    REQUIRE(matched != nullptr);
    CHECK(matched->name == "Generic PLA @System");

    // No library preset exists for an unrelated material => still no match.
    CHECK(bundle.filaments.find_preset2("BrandX Generic PETG", /*auto_match=*/true) == nullptr);
}

TEST_CASE("Renamed parent is normalized into a loaded preset's inherits", "[Preset][Rename]")
{
    ScopedTemporaryDir   temp_dir;
    RenameTestCollection coll;

    // Current parent, renamed from "Old Process".
    add_inmemory_preset(coll, "New Process");
    set_renamed_from(coll, "New Process", { "Old Process" });
    coll.update_map_system_profile_renamed();

    // A user preset on disk that still inherits the OLD name.
    write_preset_with_inherits(coll.default_preset().config,
                               temp_dir.path() / PRESET_PRINT_NAME / "Child.json", "Child", "Old Process");

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *child = coll.find_preset("Child");
    REQUIRE(child != nullptr);
    // The dangling "Old Process" was rewritten to the resolved parent name at load time,
    // so the runtime walker (plain find_preset) can resolve the chain.
    CHECK(child->inherits() == "New Process");
    REQUIRE(coll.get_preset_parent(*child) != nullptr);
    CHECK(coll.get_preset_parent(*child)->name == "New Process");
}

TEST_CASE("Removed Generic parent is normalized into a loaded filament's inherits", "[Preset][Rename]")
{
    ScopedTemporaryDir temp_dir;
    PresetBundle  bundle;

    add_inmemory_preset(bundle.filaments, "Generic PLA @System");

    // A user filament that still inherits a removed "<vendor> Generic PLA" profile.
    write_preset_with_inherits(bundle.filaments.default_preset().config,
                               temp_dir.path() / PRESET_FILAMENT_NAME / "MyPLA.json", "MyPLA", "Voron Generic PLA");

    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *child = bundle.filaments.find_preset("MyPLA");
    REQUIRE(child != nullptr);
    CHECK(child->inherits() == "Generic PLA @System");
    REQUIRE(bundle.filaments.get_preset_parent(*child) != nullptr);
    CHECK(bundle.filaments.get_preset_parent(*child)->name == "Generic PLA @System");
}

namespace {

// A live reference to a preset's compatible_printers / compatible_prints list. Fetches the *stored*
// preset (real=true) so writes and reads hit the same object; creates the option if absent.
std::vector<std::string> &compatible_list(PresetCollection &coll, const std::string &preset_name, const char *field_key)
{
    Preset *preset = coll.find_preset(preset_name, /*first_visible_if_not_found=*/false, /*real=*/true);
    REQUIRE(preset != nullptr);
    return preset->config.option<ConfigOptionStrings>(field_key, true)->values;
}

} // namespace

TEST_CASE("Renamed printer/process names are normalized into compatible lists on load", "[Preset][Rename]")
{
    PresetBundle bundle;

    // Current printer + process, each renamed from an older name.
    add_inmemory_preset(bundle.printers, "New Printer");
    set_renamed_from(bundle.printers, "New Printer", { "Old Printer" });
    add_inmemory_preset(bundle.prints, "New Process");
    set_renamed_from(bundle.prints, "New Process", { "Old Process" });

    // A user process still compatible with the OLD printer name.
    add_inmemory_preset(bundle.prints, "My Process");
    compatible_list(bundle.prints, "My Process", "compatible_printers") = { "Old Printer" };

    // A user filament referencing the OLD printer AND OLD process names, plus an unknown printer.
    add_inmemory_preset(bundle.filaments, "My Filament");
    compatible_list(bundle.filaments, "My Filament", "compatible_printers") = { "Old Printer", "Unknown Printer" };
    compatible_list(bundle.filaments, "My Filament", "compatible_prints")   = { "Old Process" };

    // Build the rename maps (done during system load in the real pipeline), then normalize.
    AppConfig app_config;
    bundle.load_installed_printers(app_config); // rebuilds every collection's rename map
    bundle.normalize_compatible_presets();

    // The stale printer name in a process' compatible_printers is rewritten to the current name.
    CHECK(compatible_list(bundle.prints, "My Process", "compatible_printers") == std::vector<std::string>{ "New Printer" });

    // The stale process name in a filament's compatible_prints is rewritten (this field has no
    // runtime rename fallback, so load-time normalization is the only fix).
    CHECK(compatible_list(bundle.filaments, "My Filament", "compatible_prints") == std::vector<std::string>{ "New Process" });

    // The renamed printer is rewritten while the unknown/deleted name is preserved as-is.
    CHECK(compatible_list(bundle.filaments, "My Filament", "compatible_printers") ==
          (std::vector<std::string>{ "New Printer", "Unknown Printer" }));

    // Normalizing rewrites config in place without flagging the preset dirty.
    CHECK_FALSE(bundle.prints.find_preset("My Process", false, true)->is_dirty);

    // A system preset that already references the current name is left untouched (idempotent no-op).
    bundle.normalize_compatible_presets();
    CHECK(compatible_list(bundle.prints, "My Process", "compatible_printers") == std::vector<std::string>{ "New Printer" });
}

TEST_CASE("Renamed names are normalized into a SYSTEM preset's compatible lists", "[Preset][Rename]")
{
    PresetBundle bundle;

    // Current printer + process, each renamed from an older name.
    add_inmemory_preset(bundle.printers, "New Printer");
    set_renamed_from(bundle.printers, "New Printer", { "Old Printer" });
    add_inmemory_preset(bundle.prints, "New Process");
    set_renamed_from(bundle.prints, "New Process", { "Old Process" });

    // A *system* (vendor) filament whose own compatible lists still reference the OLD names. A vendor
    // profile can point at a sibling preset that was later renamed, so system presets must be
    // normalized too (they are skipped by neither collection walk).
    add_inmemory_preset(bundle.filaments, "System Filament").is_system = true;
    compatible_list(bundle.filaments, "System Filament", "compatible_printers") = { "Old Printer" };
    compatible_list(bundle.filaments, "System Filament", "compatible_prints")   = { "Old Process" };

    AppConfig app_config;
    bundle.load_installed_printers(app_config); // build the rename maps
    bundle.normalize_compatible_presets();

    // The stale references in the system preset are rewritten to the current names.
    CHECK(compatible_list(bundle.filaments, "System Filament", "compatible_printers") ==
          std::vector<std::string>{ "New Printer" });
    CHECK(compatible_list(bundle.filaments, "System Filament", "compatible_prints") ==
          std::vector<std::string>{ "New Process" });

    // The rewrite does not flag the system preset dirty, and is idempotent.
    CHECK_FALSE(bundle.filaments.find_preset("System Filament", false, true)->is_dirty);
    bundle.normalize_compatible_presets();
    CHECK(compatible_list(bundle.filaments, "System Filament", "compatible_printers") ==
          std::vector<std::string>{ "New Printer" });
}

TEST_CASE("compatible_prints on SLA materials resolves against sla_prints, not prints", "[Preset][Rename]")
{
    PresetBundle bundle;

    // A renamed SLA process, and a same-named FFF process that must NOT be picked up: resolving the
    // SLA material's compatible_prints against `prints` would wrongly rewrite to "Wrong FFF Process".
    add_inmemory_preset(bundle.sla_prints, "New SLA Process");
    set_renamed_from(bundle.sla_prints, "New SLA Process", { "Old SLA Process" });
    add_inmemory_preset(bundle.prints, "Wrong FFF Process");
    set_renamed_from(bundle.prints, "Wrong FFF Process", { "Old SLA Process" });

    add_inmemory_preset(bundle.sla_materials, "My SLA Material");
    compatible_list(bundle.sla_materials, "My SLA Material", "compatible_prints") = { "Old SLA Process" };

    AppConfig app_config;
    bundle.load_installed_printers(app_config);
    bundle.normalize_compatible_presets();

    CHECK(compatible_list(bundle.sla_materials, "My SLA Material", "compatible_prints") ==
          std::vector<std::string>{ "New SLA Process" });
}

TEST_CASE("Profile validator flags dangling and renamed preset references", "[Preset][Validate]")
{
    PresetBundle bundle;

    // Current printers: a real one, and a renamed one (its old name resolves via renamed_from).
    add_inmemory_preset(bundle.printers, "Real Printer");
    add_inmemory_preset(bundle.printers, "New Printer");
    set_renamed_from(bundle.printers, "New Printer", { "Old Printer" });

    // A real process, referenced from a filament's compatible_prints.
    add_inmemory_preset(bundle.prints, "Real Process").is_system = true;

    // A fully valid system filament: references only current names.
    add_inmemory_preset(bundle.filaments, "Good Filament").is_system = true;
    compatible_list(bundle.filaments, "Good Filament", "compatible_printers") = { "Real Printer" };
    compatible_list(bundle.filaments, "Good Filament", "compatible_prints")   = { "Real Process" };

    AppConfig app_config;
    bundle.load_installed_printers(app_config); // build the rename maps

    // With only valid references, the validator is clean.
    CHECK_FALSE(bundle.check_preset_references());

    SECTION("deleted compatible_printers is flagged") {
        add_inmemory_preset(bundle.filaments, "Ghost Ref Filament").is_system = true;
        compatible_list(bundle.filaments, "Ghost Ref Filament", "compatible_printers") = { "Ghost Printer" };
        CHECK(bundle.check_preset_references());
    }

    SECTION("renamed compatible_printers (old name) is flagged") {
        add_inmemory_preset(bundle.filaments, "Old Ref Filament").is_system = true;
        compatible_list(bundle.filaments, "Old Ref Filament", "compatible_printers") = { "Old Printer" };
        CHECK(bundle.check_preset_references());
    }

    SECTION("deleted compatible_prints is flagged") {
        add_inmemory_preset(bundle.filaments, "Bad Process Ref").is_system = true;
        compatible_list(bundle.filaments, "Bad Process Ref", "compatible_prints") = { "Ghost Process" };
        CHECK(bundle.check_preset_references());
    }

    SECTION("deleted inherits parent is flagged") {
        add_inmemory_preset(bundle.filaments, "Orphan Filament", "Ghost Parent").is_system = true;
        CHECK(bundle.check_preset_references());
    }

    SECTION("non-system preset with a dangling reference is ignored") {
        add_inmemory_preset(bundle.filaments, "User Filament"); // is_system stays false
        compatible_list(bundle.filaments, "User Filament", "compatible_printers") = { "Ghost Printer" };
        CHECK_FALSE(bundle.check_preset_references());
    }
}

// Under a shared override key, the last preset merged into the full config overwrote the others', so an
// edited slicing-pipeline override never reached Print::apply's diff and re-configuring a plugin never
// re-sliced. Per-type keys make that collision impossible; guard the scoping here.
TEST_CASE("Plugin capability override keys are scoped per preset type", "[Preset][Plugin]")
{
    // Pin the key names: presets and 3mf files store them verbatim, so a rename is a format change.
    CHECK(Preset::plugin_overrides_key(Preset::TYPE_PRINT)    == std::string("print_plugin_config_overrides"));
    CHECK(Preset::plugin_overrides_key(Preset::TYPE_PRINTER)  == std::string("printer_plugin_config_overrides"));
    CHECK(Preset::plugin_overrides_key(Preset::TYPE_FILAMENT) == std::string("filament_plugin_config_overrides"));

    // ...and each key lives on exactly its own preset type's option list, so no two ever share a slot.
    const std::pair<Preset::Type, const std::vector<std::string>*> scopes[] = {
        {Preset::TYPE_PRINT,    &Preset::print_options()},
        {Preset::TYPE_PRINTER,  &Preset::printer_options()},
        {Preset::TYPE_FILAMENT, &Preset::filament_options()},
    };
    for (const auto &owner : scopes)
        for (const auto &scoped : scopes) {
            const std::string key = Preset::plugin_overrides_key(scoped.first);
            CAPTURE(owner.first, key);
            CHECK(contains(*owner.second, key) == (owner.first == scoped.first));
        }
}

namespace {

// A standalone filament collection that exposes the protected library masking builder, so the Orca
// Filament Library scenario can be set up without the full system-profile load pipeline.
struct LibraryFilamentTestCollection : public PresetCollection
{
    LibraryFilamentTestCollection()
        : PresetCollection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                           static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()))
    {}
    using PresetCollection::update_library_profile_excluded_from;
};

} // namespace

// Orca: a filament in the Orca Filament Library that names its compatible printers has to hide the generic
// library filament sharing its alias, the same way a vendor owned filament does. Otherwise both are compatible
// with that printer and the plater combo box lists the shared alias twice.
TEST_CASE("A printer specific filament supersedes the generic library filament with the same alias", "[Preset][Bundle]")
{
    LibraryFilamentTestCollection filaments;
    PresetCollection              printers(Preset::TYPE_PRINTER, Preset::printer_options(),
                                           static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    // The masking keys off the vendor name, which VendorProfile's constructor does not derive from the id.
    VendorProfile                 library(PresetBundle::ORCA_FILAMENT_LIBRARY);
    VendorProfile                 vendor("Vendor");
    library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
    vendor.name  = "Vendor";

    auto add_filament = [&filaments](const VendorProfile &owner, const std::string &name, std::vector<std::string> compatible_printers) {
        Preset &preset = add_inmemory_preset(filaments, name);
        preset.alias   = "Generic ABS";
        preset.vendor  = &owner;
        preset.config.option<ConfigOptionStrings>("compatible_printers", true)->values = std::move(compatible_printers);
    };

    add_filament(library, "Generic ABS @System", {});
    add_filament(library, "Generic ABS @Printer A", { "Printer A" });
    add_filament(vendor,  "Generic ABS @Printer B", { "Printer B" });

    filaments.update_library_profile_excluded_from();

    const Preset *generic = filaments.find_preset("Generic ABS @System");
    REQUIRE(generic != nullptr);
    CHECK(generic->m_excluded_from.count("Printer A") == 1);
    CHECK(generic->m_excluded_from.count("Printer B") == 1);
    CHECK(generic->m_excluded_from.size() == 2);

    // A printer specific profile names printers, so it is never the one being hidden - not even by itself.
    const Preset *specific = filaments.find_preset("Generic ABS @Printer A");
    REQUIRE(specific != nullptr);
    CHECK(specific->m_excluded_from.empty());

    // ...and the generic profile really drops out of the compatible set on the printer it is hidden from.
    add_inmemory_preset(printers, "Printer A");
    add_inmemory_preset(printers, "Printer C");
    const Preset *printer_a = printers.find_preset("Printer A");
    const Preset *printer_c = printers.find_preset("Printer C");
    REQUIRE(printer_a != nullptr);
    REQUIRE(printer_c != nullptr);

    const PresetWithVendorProfile generic_lib(*generic, &library);
    CHECK_FALSE(is_compatible_with_printer(generic_lib, PresetWithVendorProfile(*printer_a, nullptr)));
    CHECK(is_compatible_with_printer(generic_lib, PresetWithVendorProfile(*printer_c, nullptr)));
}

TEST_CASE("Prusa XL miniature Pin profiles resolve to the mixed-nozzle tool", "[Preset][Bundle][MiniaturePin]")
{
    static constexpr const char *machine_name = "Prusa XL 5T T2 0.25 nozzle (others 0.4)";
    static constexpr const char *filament_name = "Prusa Generic Miniature PLA @XL 5T";
    static constexpr const char *ultra_name = "0.05mm Miniature Ultra Detail + Pin @Prusa XL 5T T2 0.25";
    static constexpr const char *balanced_name = "0.06mm Miniature Balanced + Pin @Prusa XL 5T T2 0.25";

    // Non-instantiated bases are deliberately not exposed as selectable Presets. Loading both
    // leaves with their inherited values proves the registered base was parsed and resolved.
    std::ifstream base_stream(std::string(PROFILES_DIR) + "/Prusa/process/process_common_fdm_miniature_pin.json");
    REQUIRE(base_stream.good());
    const nlohmann::json base_json = nlohmann::json::parse(base_stream);
    CHECK(base_json.at("instantiation") == "false");
    CHECK(base_json.at("inherits") == "fdm_process_common");

    PresetBundle bundle;
    bundle.set_is_validation_mode(true);
    const size_t loaded = bundle.load_vendor_configs_from_json(
        PROFILES_DIR, "Prusa", PresetBundle::LoadSystem,
        ForwardCompatibilitySubstitutionRule::EnableSilent).second;
    REQUIRE(loaded > 0);
    CHECK(bundle.error_count() == 0);

    const Preset *mixed = bundle.printers.find_preset(machine_name);
    const Preset *all_025 = bundle.printers.find_preset("Prusa XL 5T 0.25 nozzle");
    const Preset *all_04 = bundle.printers.find_preset("Prusa XL 5T 0.4 nozzle");
    const Preset *filament = bundle.filaments.find_preset(filament_name);
    const Preset *ultra = bundle.prints.find_preset(ultra_name);
    const Preset *balanced = bundle.prints.find_preset(balanced_name);
    REQUIRE(mixed != nullptr);
    REQUIRE(all_025 != nullptr);
    REQUIRE(all_04 != nullptr);
    REQUIRE(filament != nullptr);
    REQUIRE(ultra != nullptr);
    REQUIRE(balanced != nullptr);

    CHECK(mixed->setting_id == "pnHHbTdiPFG91PkJ");
    CHECK(filament->setting_id == "gxDkVTWzwp2RGs1B");
    CHECK(ultra->setting_id == "8i9GCbiC0svpT5XW");
    CHECK(balanced->setting_id == "8UkqYcJwzi4txFkG");
    CHECK(mixed->config.opt_string("default_print_profile") == balanced_name);
    const ConfigOptionStrings *default_filament = mixed->config.option<ConfigOptionStrings>("default_filament_profile");
    REQUIRE(default_filament != nullptr);
    REQUIRE(default_filament->values.size() == 1);
    CHECK(default_filament->values.front() == filament_name);
    CHECK(mixed->config.opt_int("master_extruder_id") == 2);

    const ConfigOptionFloats *nozzles = mixed->config.option<ConfigOptionFloats>("nozzle_diameter");
    const ConfigOptionFloats *min_heights = mixed->config.option<ConfigOptionFloats>("min_layer_height");
    const ConfigOptionFloats *max_heights = mixed->config.option<ConfigOptionFloats>("max_layer_height");
    REQUIRE(nozzles != nullptr);
    REQUIRE(min_heights != nullptr);
    REQUIRE(max_heights != nullptr);
    REQUIRE(nozzles->values.size() == 5);
    REQUIRE(min_heights->values.size() == 5);
    REQUIRE(max_heights->values.size() == 5);
    const std::vector<double> expected_nozzles { 0.4, 0.25, 0.4, 0.4, 0.4 };
    const std::vector<double> expected_min_heights { 0.07, 0.05, 0.07, 0.07, 0.07 };
    const std::vector<double> expected_max_heights { 0.3, 0.15, 0.3, 0.3, 0.3 };
    for (size_t tool = 0; tool < expected_nozzles.size(); ++tool) {
        CAPTURE(tool);
        CHECK_THAT(nozzles->values[tool], Catch::Matchers::WithinAbs(expected_nozzles[tool], 1e-9));
        CHECK_THAT(min_heights->values[tool], Catch::Matchers::WithinAbs(expected_min_heights[tool], 1e-9));
        CHECK_THAT(max_heights->values[tool], Catch::Matchers::WithinAbs(expected_max_heights[tool], 1e-9));
    }

    const ConfigOptionFloats *all_025_nozzles = all_025->config.option<ConfigOptionFloats>("nozzle_diameter");
    const ConfigOptionFloats *all_04_nozzles = all_04->config.option<ConfigOptionFloats>("nozzle_diameter");
    REQUIRE(all_025_nozzles != nullptr);
    REQUIRE(all_04_nozzles != nullptr);
    REQUIRE(all_025_nozzles->values.size() == 5);
    REQUIRE(all_04_nozzles->values.size() == 5);
    for (size_t tool = 0; tool < 5; ++tool) {
        CAPTURE(tool);
        CHECK_THAT(all_025_nozzles->values[tool], Catch::Matchers::WithinAbs(0.25, 1e-9));
        CHECK_THAT(all_04_nozzles->values[tool], Catch::Matchers::WithinAbs(0.4, 1e-9));
    }

    const std::array<const Preset *, 2> miniature_profiles { ultra, balanced };
    const std::array<const char *, 8> tool_routed_options {
        "outer_wall_filament_id", "inner_wall_filament_id", "sparse_infill_filament_id",
        "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id",
        "support_filament", "support_interface_filament"
    };
    for (const Preset *profile : miniature_profiles) {
        CAPTURE(profile->name);
        CHECK(profile->config.option("wall_generator")->serialize() == "classic");
        CHECK(profile->config.option("wall_sequence")->serialize() == "inner-outer-inner wall");
        CHECK(profile->config.opt_int("wall_loops") == 3);
        CHECK(profile->config.option("sparse_infill_density")->serialize() == "20%");
        CHECK(profile->config.option("sparse_infill_pattern")->serialize() == "gyroid");
        CHECK(profile->config.option("line_width")->serialize() == "100%");
        CHECK(profile->config.option("initial_layer_line_width")->serialize() == "125%");
        CHECK(profile->config.option("inner_wall_line_width")->serialize() == "120%");
        CHECK(profile->config.option("outer_wall_line_width")->serialize() == "115%");
        CHECK(profile->config.option("sparse_infill_line_width")->serialize() == "110%");
        CHECK(profile->config.option("internal_solid_infill_line_width")->serialize() == "110%");
        CHECK(profile->config.option("top_surface_line_width")->serialize() == "105%");
        CHECK(profile->config.option("support_type")->serialize() == "tree(auto)");
        CHECK(profile->config.option("support_style")->serialize() == "organic");
        CHECK_FALSE(profile->config.opt_bool("support_on_build_plate_only"));
        CHECK(profile->config.opt_int("support_interface_top_layers") == 0);
        CHECK(profile->config.opt_int("support_interface_bottom_layers") == 0);
        CHECK(profile->config.opt_bool("tree_support_round_tip"));
        CHECK_THAT(profile->config.opt_float("tree_support_angle_slow"),
                   Catch::Matchers::WithinAbs(25.0, 1e-9));
        CHECK_THAT(profile->config.opt_float("tree_support_branch_distance_organic"),
                   Catch::Matchers::WithinAbs(3.0, 1e-9));
        for (const char *key : tool_routed_options) {
            CAPTURE(key);
            CHECK(profile->config.opt_int(key) == 2);
        }

        const PresetWithVendorProfile profile_with_vendor = bundle.prints.get_preset_with_vendor_profile(*profile);
        CHECK(is_compatible_with_printer(profile_with_vendor, bundle.printers.get_preset_with_vendor_profile(*mixed)));
        CHECK_FALSE(is_compatible_with_printer(profile_with_vendor, bundle.printers.get_preset_with_vendor_profile(*all_025)));
        CHECK_FALSE(is_compatible_with_printer(profile_with_vendor, bundle.printers.get_preset_with_vendor_profile(*all_04)));
    }
    CHECK_THAT(ultra->config.opt_float("support_top_z_distance"), Catch::Matchers::WithinAbs(0.15, 1e-9));
    CHECK_THAT(balanced->config.opt_float("support_top_z_distance"), Catch::Matchers::WithinAbs(0.18, 1e-9));

    const ConfigOptionFloats *volumetric_speed = filament->config.option<ConfigOptionFloats>("filament_max_volumetric_speed");
    const ConfigOptionStrings *start_gcode = filament->config.option<ConfigOptionStrings>("filament_start_gcode");
    REQUIRE(volumetric_speed != nullptr);
    REQUIRE(start_gcode != nullptr);
    REQUIRE(volumetric_speed->values.size() == 1);
    REQUIRE(start_gcode->values.size() == 1);
    CHECK_THAT(volumetric_speed->values.front(), Catch::Matchers::WithinAbs(3.0, 1e-9));
    CHECK(start_gcode->values.front().find("nozzle_diameter[filament_extruder_id]==0.25}0.14") != std::string::npos);
    CHECK(start_gcode->values.front().find("nozzle_diameter[filament_extruder_id]==0.25}0.12") != std::string::npos);
    CHECK(start_gcode->values.front().find("nozzle_diameter[0]") == std::string::npos);
    CHECK(is_compatible_with_printer(bundle.filaments.get_preset_with_vendor_profile(*filament),
                                     bundle.printers.get_preset_with_vendor_profile(*mixed)));
    CHECK_FALSE(is_compatible_with_printer(bundle.filaments.get_preset_with_vendor_profile(*filament),
                                           bundle.printers.get_preset_with_vendor_profile(*all_04)));
}


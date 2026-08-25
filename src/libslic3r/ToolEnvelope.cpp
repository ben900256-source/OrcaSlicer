#include "ToolEnvelope.hpp"

#include "Exception.hpp"
#include "Geometry.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace Slic3r {
namespace {

[[noreturn]] void invalid(const std::string &field, const std::string &reason)
{
    throw InvalidArgument("ToolEnvelope." + field + ": " + reason);
}

void require_finite(double value, const std::string &field)
{
    if (!std::isfinite(value))
        invalid(field, "must be finite");
}

bool rings_intersect(const Polygon &first, const Polygon &second)
{
    for (std::size_t i = 0; i < first.points.size(); ++i) {
        const Point &a = first.points[i];
        const Point &b = first.points[(i + 1) % first.points.size()];
        for (std::size_t j = 0; j < second.points.size(); ++j) {
            const Point &c = second.points[j];
            const Point &d = second.points[(j + 1) % second.points.size()];
            if (Geometry::segments_intersect(a, b, c, d))
                return true;
        }
    }
    return false;
}

void validate_ring(const Polygon &ring, const std::string &field)
{
    const Points &points = ring.points;
    if (points.size() < 3)
        invalid(field, "must contain at least three points");

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (points[i] == points[(i + 1) % points.size()])
            invalid(field, "contains a zero-length edge");
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            if (points[i] == points[j])
                invalid(field, "contains a duplicate vertex");
        }
    }

    if (std::abs(ring.area()) < 0.5)
        invalid(field, "is degenerate");

    for (std::size_t i = 0; i < points.size(); ++i) {
        const std::size_t i_next = (i + 1) % points.size();
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            const std::size_t j_next = (j + 1) % points.size();
            if (i == j || i_next == j || j_next == i)
                continue;
            if (Geometry::segments_intersect(points[i], points[i_next], points[j], points[j_next]))
                invalid(field, "self-intersects");
        }
    }
}

void validate_expolygon(const ExPolygon &polygon, const std::string &field)
{
    validate_ring(polygon.contour, field + ".contour");
    if (!polygon.contour.is_counter_clockwise())
        invalid(field + ".contour", "must be counter-clockwise");

    for (std::size_t hole_index = 0; hole_index < polygon.holes.size(); ++hole_index) {
        const Polygon &hole = polygon.holes[hole_index];
        const std::string hole_field = field + ".holes[" + std::to_string(hole_index) + "]";
        validate_ring(hole, hole_field);
        if (hole.is_counter_clockwise())
            invalid(hole_field, "must be clockwise");
        if (rings_intersect(polygon.contour, hole) ||
            !Slic3r::contains(polygon.contour, hole.points.front(), false))
            invalid(hole_field, "must lie strictly inside its contour");

        for (std::size_t previous = 0; previous < hole_index; ++previous) {
            const Polygon &other = polygon.holes[previous];
            if (rings_intersect(other, hole) ||
                Slic3r::contains(other, hole.points.front(), true) ||
                Slic3r::contains(hole, other.points.front(), true))
                invalid(hole_field, "overlaps another hole");
        }
    }
}

BoundingBoxf bounds_in_mm(const ExPolygons &geometry)
{
    const BoundingBox scaled_bounds = get_extents(geometry);
    return BoundingBoxf(unscale(scaled_bounds.min), unscale(scaled_bounds.max));
}

const nlohmann::json &required_field(const nlohmann::json &object,
                                     const char *name,
                                     const std::string &field)
{
    const auto found = object.find(name);
    if (found == object.end())
        invalid(field + "." + name, "is required");
    return *found;
}

double json_number(const nlohmann::json &value, const std::string &field)
{
    if (!value.is_number())
        invalid(field, "must be a number");
    const double result = value.get<double>();
    require_finite(result, field);
    return result;
}

std::string json_string(const nlohmann::json &value, const std::string &field)
{
    if (!value.is_string())
        invalid(field, "must be a string");
    return value.get<std::string>();
}

Polygon parse_ring(const nlohmann::json &json, const std::string &field)
{
    if (!json.is_array())
        invalid(field, "must be an array of [x, y] points");
    if (json.size() < 3)
        invalid(field, "must contain at least three points");

    Points points;
    points.reserve(json.size());
    for (std::size_t point_index = 0; point_index < json.size(); ++point_index) {
        const nlohmann::json &point = json[point_index];
        const std::string point_field = field + "[" + std::to_string(point_index) + "]";
        if (!point.is_array() || point.size() != 2)
            invalid(point_field, "must be [x, y]");
        const double x = json_number(point[0], point_field + "[0]");
        const double y = json_number(point[1], point_field + "[1]");
        const double max_mm = static_cast<double>(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR;
        if (std::abs(x) >= max_mm || std::abs(y) >= max_mm)
            invalid(point_field, "is outside the scaled-coordinate range");
        points.emplace_back(Point::new_scale(x, y));
    }

    // A single repeated closing point is common in external polygon formats. It
    // carries no information in OrcaSlicer's implicitly closed Polygon.
    if (points.size() > 3 && points.front() == points.back())
        points.pop_back();
    return Polygon(std::move(points));
}

ExPolygon parse_expolygon(const nlohmann::json &json, const std::string &field)
{
    ExPolygon polygon;
    if (json.is_array()) {
        polygon.contour = parse_ring(json, field + ".contour");
    } else if (json.is_object()) {
        polygon.contour = parse_ring(required_field(json, "contour", field), field + ".contour");
        const auto holes = json.find("holes");
        if (holes != json.end()) {
            if (!holes->is_array())
                invalid(field + ".holes", "must be an array of rings");
            polygon.holes.reserve(holes->size());
            for (std::size_t hole_index = 0; hole_index < holes->size(); ++hole_index)
                polygon.holes.emplace_back(parse_ring((*holes)[hole_index],
                    field + ".holes[" + std::to_string(hole_index) + "]"));
        }
    } else {
        invalid(field, "must be a contour ring or an object");
    }

    if (!polygon.contour.is_counter_clockwise())
        polygon.contour.reverse();
    for (Polygon &hole : polygon.holes)
        if (hole.is_counter_clockwise())
            hole.reverse();
    validate_expolygon(polygon, field);
    return polygon;
}

nlohmann::json ring_json(const Polygon &ring)
{
    nlohmann::json result = nlohmann::json::array();
    for (const Point &point : ring.points)
        result.push_back({unscale<double>(point.x()), unscale<double>(point.y())});
    return result;
}

nlohmann::json expolygon_json(const ExPolygon &polygon)
{
    nlohmann::json holes = nlohmann::json::array();
    for (const Polygon &hole : polygon.holes)
        holes.push_back(ring_json(hole));
    return {{"contour", ring_json(polygon.contour)}, {"holes", std::move(holes)}};
}

} // namespace

const char *tool_envelope_zone_name(ToolEnvelopeZone zone) noexcept
{
    switch (zone) {
    case ToolEnvelopeZone::Deposition:     return "deposition";
    case ToolEnvelopeZone::HardCollision: return "hard_collision";
    case ToolEnvelopeZone::Thermal:        return "thermal";
    case ToolEnvelopeZone::Toolhead:       return "toolhead";
    }
    return "unknown";
}

ToolEnvelopeZone tool_envelope_zone_from_string(const std::string &value)
{
    if (value == "deposition")
        return ToolEnvelopeZone::Deposition;
    if (value == "hard_collision")
        return ToolEnvelopeZone::HardCollision;
    if (value == "thermal")
        return ToolEnvelopeZone::Thermal;
    if (value == "toolhead")
        return ToolEnvelopeZone::Toolhead;
    invalid("zone", "unsupported value '" + value + "'");
}

ToolEnvelopeSlice::ToolEnvelopeSlice(double z_min_mm,
                                     double z_max_mm,
                                     ExPolygons geometry,
                                     ToolEnvelopeZone zone,
                                     std::string name)
    : m_z_min_mm(z_min_mm)
    , m_z_max_mm(z_max_mm)
    , m_geometry(std::move(geometry))
    , m_zone(zone)
    , m_name(std::move(name))
{
    require_finite(m_z_min_mm, "slice.z_min_mm");
    require_finite(m_z_max_mm, "slice.z_max_mm");
    if (m_z_min_mm >= m_z_max_mm)
        invalid("slice.z_range", "z_min_mm must be less than z_max_mm");
    switch (m_zone) {
    case ToolEnvelopeZone::Deposition:
    case ToolEnvelopeZone::HardCollision:
    case ToolEnvelopeZone::Thermal:
    case ToolEnvelopeZone::Toolhead:
        break;
    default:
        invalid("slice.zone", "is not a supported zone");
    }
    if (m_geometry.empty())
        invalid("slice.geometry", "must not be empty");
    for (std::size_t index = 0; index < m_geometry.size(); ++index)
        validate_expolygon(m_geometry[index], "slice.geometry[" + std::to_string(index) + "]");
    m_local_bounds_mm = bounds_in_mm(m_geometry);
    if (!m_local_bounds_mm.defined)
        invalid("slice.geometry", "has empty or degenerate bounds");
}

ToolEnvelope::ToolEnvelope(std::vector<ToolEnvelopeSlice> slices, std::string name)
    : m_slices(std::move(slices))
    , m_name(std::move(name))
    , m_z_min_mm(0.0)
    , m_z_max_mm(0.0)
{
    if (m_slices.empty())
        invalid("slices", "must not be empty");

    m_local_bounds_mm = m_slices.front().local_bounds_mm();
    m_z_min_mm = m_slices.front().z_min_mm();
    m_z_max_mm = m_slices.front().z_max_mm();
    for (std::size_t index = 1; index < m_slices.size(); ++index) {
        m_local_bounds_mm.merge(m_slices[index].local_bounds_mm());
        m_z_min_mm = std::min(m_z_min_mm, m_slices[index].z_min_mm());
        m_z_max_mm = std::max(m_z_max_mm, m_slices[index].z_max_mm());
    }
}

nlohmann::json ToolEnvelope::to_json() const
{
    nlohmann::json slices = nlohmann::json::array();
    for (const ToolEnvelopeSlice &slice : m_slices) {
        nlohmann::json geometry = nlohmann::json::array();
        for (const ExPolygon &polygon : slice.geometry())
            geometry.push_back(expolygon_json(polygon));
        slices.push_back({
            {"name", slice.name()},
            {"zone", tool_envelope_zone_name(slice.zone())},
            {"z_min_mm", slice.z_min_mm()},
            {"z_max_mm", slice.z_max_mm()},
            {"geometry", std::move(geometry)}
        });
    }
    return {
        {"version", JSON_VERSION},
        {"origin", "nozzle_tip"},
        {"name", m_name},
        {"slices", std::move(slices)}
    };
}

std::string ToolEnvelope::to_json_string(int indent) const
{
    return to_json().dump(indent);
}

ToolEnvelope ToolEnvelope::from_json(const nlohmann::json &json)
{
    if (!json.is_object())
        invalid("json", "root must be an object");

    const nlohmann::json &version_json = required_field(json, "version", "json");
    if (!version_json.is_number_integer())
        invalid("json.version", "must be an integer");
    int version = 0;
    try {
        version = version_json.get<int>();
    } catch (const nlohmann::json::exception &) {
        invalid("json.version", "is outside the supported integer range");
    }
    if (version != JSON_VERSION)
        invalid("json.version", "unsupported version " + std::to_string(version));

    const std::string origin = json_string(required_field(json, "origin", "json"), "json.origin");
    if (origin != "nozzle_tip")
        invalid("json.origin", "must be 'nozzle_tip'");

    std::string name;
    if (const auto found = json.find("name"); found != json.end())
        name = json_string(*found, "json.name");

    const nlohmann::json &slices_json = required_field(json, "slices", "json");
    if (!slices_json.is_array() || slices_json.empty())
        invalid("json.slices", "must be a non-empty array");

    std::vector<ToolEnvelopeSlice> slices;
    slices.reserve(slices_json.size());
    for (std::size_t slice_index = 0; slice_index < slices_json.size(); ++slice_index) {
        const nlohmann::json &slice_json = slices_json[slice_index];
        const std::string field = "json.slices[" + std::to_string(slice_index) + "]";
        if (!slice_json.is_object())
            invalid(field, "must be an object");

        const nlohmann::json *z_min_json = nullptr;
        const nlohmann::json *z_max_json = nullptr;
        if (const auto found = slice_json.find("z_min_mm"); found != slice_json.end())
            z_min_json = &*found;
        else if (const auto found = slice_json.find("z_min"); found != slice_json.end())
            z_min_json = &*found;
        else
            invalid(field + ".z_min_mm", "is required");
        if (const auto found = slice_json.find("z_max_mm"); found != slice_json.end())
            z_max_json = &*found;
        else if (const auto found = slice_json.find("z_max"); found != slice_json.end())
            z_max_json = &*found;
        else
            invalid(field + ".z_max_mm", "is required");
        const double z_min = json_number(*z_min_json, field + ".z_min_mm");
        const double z_max = json_number(*z_max_json, field + ".z_max_mm");
        const std::string zone_string = json_string(required_field(slice_json, "zone", field), field + ".zone");
        ToolEnvelopeZone zone;
        try {
            zone = tool_envelope_zone_from_string(zone_string);
        } catch (const InvalidArgument &) {
            invalid(field + ".zone", "unsupported value '" + zone_string + "'");
        }

        std::string slice_name;
        if (const auto found = slice_json.find("name"); found != slice_json.end())
            slice_name = json_string(*found, field + ".name");

        const nlohmann::json *geometry_json = nullptr;
        if (const auto found = slice_json.find("geometry"); found != slice_json.end())
            geometry_json = &*found;
        else if (const auto found = slice_json.find("polygons"); found != slice_json.end())
            geometry_json = &*found;
        else
            invalid(field + ".geometry", "is required");
        if (!geometry_json->is_array() || geometry_json->empty())
            invalid(field + ".geometry", "must be a non-empty array");

        ExPolygons geometry;
        geometry.reserve(geometry_json->size());
        for (std::size_t polygon_index = 0; polygon_index < geometry_json->size(); ++polygon_index)
            geometry.emplace_back(parse_expolygon((*geometry_json)[polygon_index],
                field + ".geometry[" + std::to_string(polygon_index) + "]"));
        slices.emplace_back(z_min, z_max, std::move(geometry), zone, std::move(slice_name));
    }
    return ToolEnvelope(std::move(slices), std::move(name));
}

ToolEnvelope ToolEnvelope::from_json(const std::string &json_text)
{
    try {
        return from_json(nlohmann::json::parse(json_text));
    } catch (const InvalidArgument &) {
        throw;
    } catch (const nlohmann::json::exception &error) {
        invalid("json", std::string("malformed JSON: ") + error.what());
    }
}

void to_json(nlohmann::json &json, const ToolEnvelope &envelope)
{
    json = envelope.to_json();
}

} // namespace Slic3r

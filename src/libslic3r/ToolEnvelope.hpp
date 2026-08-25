#pragma once

#include "BoundingBox.hpp"
#include "ExPolygon.hpp"

#include <nlohmann/json_fwd.hpp>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

enum class ToolEnvelopeZone {
    Deposition,
    HardCollision,
    Thermal,
    Toolhead
};

const char *tool_envelope_zone_name(ToolEnvelopeZone zone) noexcept;
ToolEnvelopeZone tool_envelope_zone_from_string(const std::string &value);

class ToolEnvelopeSlice
{
public:
    ToolEnvelopeSlice(double z_min_mm,
                      double z_max_mm,
                      ExPolygons geometry,
                      ToolEnvelopeZone zone,
                      std::string name = {});

    ToolEnvelopeSlice(ExPolygons geometry,
                      double z_min_mm,
                      double z_max_mm,
                      ToolEnvelopeZone zone,
                      std::string name = {})
        : ToolEnvelopeSlice(z_min_mm, z_max_mm, std::move(geometry), zone, std::move(name))
    {}

    double              z_min_mm() const noexcept { return m_z_min_mm; }
    double              z_max_mm() const noexcept { return m_z_max_mm; }
    ToolEnvelopeZone    zone() const noexcept { return m_zone; }
    const std::string  &name() const noexcept { return m_name; }
    const ExPolygons   &geometry() const noexcept { return m_geometry; }
    const ExPolygons   &polygons() const noexcept { return m_geometry; }
    const BoundingBoxf &local_bounds_mm() const noexcept { return m_local_bounds_mm; }

private:
    double           m_z_min_mm;
    double           m_z_max_mm;
    ExPolygons       m_geometry;
    ToolEnvelopeZone m_zone;
    std::string      m_name;
    BoundingBoxf     m_local_bounds_mm;
};

class ToolEnvelope
{
public:
    static constexpr int JSON_VERSION = 1;

    explicit ToolEnvelope(std::vector<ToolEnvelopeSlice> slices, std::string name = {});

    const std::string                    &name() const noexcept { return m_name; }
    const std::vector<ToolEnvelopeSlice> &slices() const noexcept { return m_slices; }
    const BoundingBoxf                   &local_bounds_mm() const noexcept { return m_local_bounds_mm; }
    double                                z_min_mm() const noexcept { return m_z_min_mm; }
    double                                z_max_mm() const noexcept { return m_z_max_mm; }

    nlohmann::json to_json() const;
    std::string    to_json_string(int indent = -1) const;

    static ToolEnvelope from_json(const nlohmann::json &json);
    static ToolEnvelope from_json(const std::string &json_text);

private:
    std::vector<ToolEnvelopeSlice> m_slices;
    std::string                    m_name;
    BoundingBoxf                   m_local_bounds_mm;
    double                         m_z_min_mm;
    double                         m_z_max_mm;
};

void to_json(nlohmann::json &json, const ToolEnvelope &envelope);

} // namespace Slic3r

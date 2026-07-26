// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/map_manifest.hpp"

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace fs = std::filesystem;

namespace go2_map_localizer
{
namespace
{

template<typename T>
T required(const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key]) {
    throw std::runtime_error(context + ": missing required field '" + key + "'");
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            context + ": invalid field '" + key + "': " + error.what());
  }
}

std::array<double, 3> requiredVector3(
  const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key] || !node[key].IsSequence() || node[key].size() != 3) {
    throw std::runtime_error(context + ": '" + key + "' must contain 3 numbers");
  }
  const std::array<double, 3> result{
    node[key][0].as<double>(),
    node[key][1].as<double>(),
    node[key][2].as<double>()};
  if (!std::all_of(result.begin(), result.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    throw std::runtime_error(context + ": '" + key + "' must contain finite numbers");
  }
  return result;
}

std::array<double, 2> requiredVector2(
  const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key] || !node[key].IsSequence() || node[key].size() != 2) {
    throw std::runtime_error(context + ": '" + key + "' must contain 2 numbers");
  }
  const std::array<double, 2> result{
    node[key][0].as<double>(), node[key][1].as<double>()};
  if (!std::all_of(result.begin(), result.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    throw std::runtime_error(context + ": '" + key + "' must contain finite numbers");
  }
  return result;
}

Bounds3 requiredBounds(const YAML::Node & node, const std::string & context)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error(context + ": bounds must be an object");
  }
  Bounds3 bounds{requiredVector3(node, "min", context), requiredVector3(node, "max", context)};
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(bounds.min[i]) || !std::isfinite(bounds.max[i]) ||
      bounds.min[i] > bounds.max[i])
    {
      throw std::runtime_error(context + ": invalid min/max bounds");
    }
  }
  return bounds;
}

bool startsWithPath(const fs::path & child, const fs::path & parent)
{
  auto child_it = child.begin();
  for (auto parent_it = parent.begin(); parent_it != parent.end(); ++parent_it, ++child_it) {
    if (child_it == child.end() || *child_it != *parent_it) {
      return false;
    }
  }
  return true;
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

void requireSha256(const std::string & value, const std::string & context)
{
  if (value.size() != 64 ||
    !std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return std::isxdigit(character) != 0;
    }))
  {
    throw std::runtime_error(context + ": expected a 64-character SHA-256");
  }
}

double squaredDistanceToBounds(double x, double y, const Bounds3 & bounds)
{
  const double dx = x < bounds.min[0] ? bounds.min[0] - x :
    (x > bounds.max[0] ? x - bounds.max[0] : 0.0);
  const double dy = y < bounds.min[1] ? bounds.min[1] - y :
    (y > bounds.max[1] ? y - bounds.max[1] : 0.0);
  return dx * dx + dy * dy;
}

std::string sha256Bytes(const std::string & bytes)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_length = 0;
  if (EVP_Digest(
      bytes.data(), bytes.size(), digest.data(), &digest_length,
      EVP_sha256(), nullptr) != 1 ||
    digest_length != 32)
  {
    throw std::runtime_error("SHA-256 memory digest failed");
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_length; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

std::string readTextFile(const fs::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open metadata file: " + path.string());
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  if (stream.bad()) {
    throw std::runtime_error("metadata read failed: " + path.string());
  }
  return contents.str();
}

}  // namespace

std::string sha256File(const fs::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open for SHA-256: " + path.string());
  }
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
    EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 1024 * 1024> buffer{};
  while (stream.good()) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    if (count > 0) {
      if (EVP_DigestUpdate(
          context.get(), buffer.data(), static_cast<std::size_t>(count)) != 1)
      {
        throw std::runtime_error("SHA-256 update failed: " + path.string());
      }
    }
  }
  if (stream.bad()) {
    throw std::runtime_error("SHA-256 read failed: " + path.string());
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_length = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_length) != 1 ||
    digest_length != 32)
  {
    throw std::runtime_error("SHA-256 finalization failed: " + path.string());
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_length; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

MapManifest MapManifest::load(const fs::path & requested_path, bool verify_hashes)
{
  MapManifest result;
  result.hashes_verified = verify_hashes;
  result.manifest_path = fs::weakly_canonical(requested_path);
  if (!fs::is_regular_file(result.manifest_path)) {
    throw std::runtime_error("manifest is not a regular file: " + requested_path.string());
  }
  result.root_directory = result.manifest_path.parent_path();
  // Parse the exact bytes whose identity is checked. Loading the path again
  // after hashing would leave a hash-to-parser TOCTOU window.
  const std::string manifest_document = readTextFile(result.manifest_path);
  result.manifest_sha256 = sha256Bytes(manifest_document);

  if (verify_hashes) {
    const auto checksum_path = result.root_directory / "manifest.sha256";
    std::ifstream checksum_stream(checksum_path);
    if (!checksum_stream) {
      throw std::runtime_error("missing manifest checksum: " + checksum_path.string());
    }
    std::string expected;
    std::string filename;
    checksum_stream >> expected >> filename;
    requireSha256(expected, "manifest.sha256");
    if (filename != result.manifest_path.filename().string()) {
      throw std::runtime_error("manifest.sha256 names a different file: " + filename);
    }
    if (lower(expected) != result.manifest_sha256) {
      throw std::runtime_error("manifest SHA-256 mismatch");
    }
  }

  const YAML::Node root = YAML::Load(manifest_document);
  result.schema = required<std::string>(root, "schema", "manifest");
  const bool legacy_v1 = result.schema == "go2.map_tiles/v1";
  if (result.schema != "go2.map_tiles/v2" && !legacy_v1) {
    throw std::runtime_error("unsupported manifest schema: " + result.schema);
  }
  if (legacy_v1 && verify_hashes) {
    throw std::runtime_error(
            "go2.map_tiles/v1 does not bind descriptor identity; "
            "rebuild as v2 or load unverified for diagnostics only");
  }
  result.map_id = required<std::string>(root, "map_id", "manifest");
  result.created_utc = required<std::string>(root, "created_utc", "manifest");
  result.frame_id = required<std::string>(root, "frame_id", "manifest");
  result.tile_size_m = root["tile_size_m"] ? root["tile_size_m"].as<double>() : 20.0;
  result.voxel_size_m = required<double>(root, "voxel_size_m", "manifest");
  result.bounds = requiredBounds(root["bounds"], "manifest.bounds");
  const YAML::Node source_metadata = root["source"];
  result.source_filename =
    required<std::string>(source_metadata, "filename", "manifest.source");
  result.source_sha256 =
    lower(required<std::string>(source_metadata, "sha256", "manifest.source"));
  result.source_point_count =
    required<std::uint64_t>(source_metadata, "point_count", "manifest.source");
  requireSha256(result.source_sha256, "manifest.source.sha256");
  if (result.map_id.empty() || result.frame_id.empty() ||
    !std::isfinite(result.tile_size_m) || !std::isfinite(result.voxel_size_m) ||
    result.tile_size_m <= 0.0 ||
    result.voxel_size_m <= 0.0 || result.source_filename.empty() ||
    result.source_point_count == 0)
  {
    throw std::runtime_error("manifest contains invalid map metadata");
  }
  if (result.map_id.find('/') != std::string::npos ||
    result.map_id.find('\\') != std::string::npos ||
    fs::path(result.source_filename).filename().string() != result.source_filename)
  {
    throw std::runtime_error("manifest map_id/source filename is unsafe");
  }
  if (!root["tiles"] || !root["tiles"].IsSequence() || root["tiles"].size() == 0) {
    throw std::runtime_error("manifest.tiles must be a non-empty array");
  }
  if (root["tiles"].size() > 100000) {
    throw std::runtime_error("manifest.tiles exceeds the bounded tile count");
  }

  const fs::path canonical_root = fs::weakly_canonical(result.root_directory);
  std::unordered_set<std::string> tile_ids;
  std::uint64_t tiled_point_count = 0;
  const double grid_tolerance = std::max(1.0e-8, result.tile_size_m * 1.0e-9);
  for (std::size_t index = 0; index < root["tiles"].size(); ++index) {
    const YAML::Node source = root["tiles"][index];
    const std::string context = "manifest.tiles[" + std::to_string(index) + "]";
    TileRecord tile;
    tile.id = required<std::string>(source, "id", context);
    tile.ix = required<int>(source, "ix", context);
    tile.iy = required<int>(source, "iy", context);
    if (tile.ix < -999999 || tile.ix > 999999 ||
      tile.iy < -999999 || tile.iy > 999999)
    {
      throw std::runtime_error(context + ": tile index exceeds the supported range");
    }
    const fs::path relative_path = required<std::string>(source, "path", context);
    if (relative_path.is_absolute()) {
      throw std::runtime_error(context + ": absolute tile paths are forbidden");
    }
    tile.path = fs::weakly_canonical(result.root_directory / relative_path);
    if (!startsWithPath(tile.path, canonical_root)) {
      throw std::runtime_error(context + ": tile path escapes map directory");
    }
    if (!fs::is_regular_file(tile.path)) {
      throw std::runtime_error(context + ": missing tile " + tile.path.string());
    }
    tile.sha256 = lower(required<std::string>(source, "sha256", context));
    requireSha256(tile.sha256, context + ".sha256");
    tile.point_count = required<std::uint64_t>(source, "point_count", context);
    tile.bounds = requiredBounds(source["bounds"], context + ".bounds");
    const YAML::Node grid_bounds = source["grid_bounds"];
    tile.grid_min = requiredVector2(grid_bounds, "min", context + ".grid_bounds");
    tile.grid_max = requiredVector2(grid_bounds, "max", context + ".grid_bounds");
    if (tile.grid_min[0] >= tile.grid_max[0] || tile.grid_min[1] >= tile.grid_max[1]) {
      throw std::runtime_error(context + ": invalid grid bounds");
    }
    const std::array<double, 2> expected_grid_min{
      tile.ix * result.tile_size_m, tile.iy * result.tile_size_m};
    const std::array<double, 2> expected_grid_max{
      (tile.ix + 1) * result.tile_size_m, (tile.iy + 1) * result.tile_size_m};
    for (std::size_t axis = 0; axis < 2; ++axis) {
      if (std::abs(tile.grid_min[axis] - expected_grid_min[axis]) > grid_tolerance ||
        std::abs(tile.grid_max[axis] - expected_grid_max[axis]) > grid_tolerance ||
        tile.bounds.min[axis] < tile.grid_min[axis] - grid_tolerance ||
        tile.bounds.max[axis] > tile.grid_max[axis] + grid_tolerance)
      {
        throw std::runtime_error(context + ": grid index/bounds consistency check failed");
      }
    }
    if (tile.id.empty() || tile.point_count == 0 || !tile_ids.insert(tile.id).second) {
      throw std::runtime_error(context + ": empty/duplicate tile id or zero point count");
    }
    if (verify_hashes && tile.sha256 != sha256File(tile.path)) {
      throw std::runtime_error(context + ": tile SHA-256 mismatch");
    }
    result.tile_lookup_[tile.id] = result.tiles.size();
    if (tile.point_count >
      std::numeric_limits<std::uint64_t>::max() - tiled_point_count)
    {
      throw std::runtime_error("tiled point count overflows uint64");
    }
    tiled_point_count += tile.point_count;
    result.tiles.push_back(std::move(tile));
  }
  if (tiled_point_count > result.source_point_count) {
    throw std::runtime_error("tiled point count exceeds source point count");
  }
  Bounds3 derived_bounds = result.tiles.front().bounds;
  for (const auto & tile : result.tiles) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      derived_bounds.min[axis] = std::min(derived_bounds.min[axis], tile.bounds.min[axis]);
      derived_bounds.max[axis] = std::max(derived_bounds.max[axis], tile.bounds.max[axis]);
    }
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (std::abs(derived_bounds.min[axis] - result.bounds.min[axis]) > grid_tolerance ||
      std::abs(derived_bounds.max[axis] - result.bounds.max[axis]) > grid_tolerance)
    {
      throw std::runtime_error("manifest bounds do not match derived tile bounds");
    }
  }

  if (legacy_v1) {
    result.descriptor_index_path =
      fs::weakly_canonical(result.root_directory / "descriptor_index.json");
  } else {
    const YAML::Node descriptor_metadata = root["descriptor_index"];
    const fs::path relative_descriptor_path =
      required<std::string>(descriptor_metadata, "path", "manifest.descriptor_index");
    if (relative_descriptor_path.is_absolute()) {
      throw std::runtime_error("manifest.descriptor_index.path must be relative");
    }
    result.descriptor_index_path =
      fs::weakly_canonical(result.root_directory / relative_descriptor_path);
    if (!startsWithPath(result.descriptor_index_path, canonical_root)) {
      throw std::runtime_error("descriptor index path escapes map directory");
    }
    result.descriptor_index_sha256 = lower(
      required<std::string>(
        descriptor_metadata, "sha256", "manifest.descriptor_index"));
    requireSha256(
      result.descriptor_index_sha256, "manifest.descriptor_index.sha256");
  }
  if (!fs::is_regular_file(result.descriptor_index_path)) {
    throw std::runtime_error(
            "missing descriptor index: " + result.descriptor_index_path.string());
  }
  // As for the manifest, compute identity and parse from one immutable byte
  // snapshot rather than reopening a mutable pathname.
  const std::string descriptor_document = readTextFile(result.descriptor_index_path);
  const std::string actual_descriptor_sha = sha256Bytes(descriptor_document);
  if (verify_hashes) {
    if (actual_descriptor_sha != result.descriptor_index_sha256) {
      throw std::runtime_error(
              "descriptor index SHA-256 does not match manifest anchor");
    }
    const fs::path descriptor_checksum_path =
      result.descriptor_index_path.string() + ".sha256";
    std::ifstream checksum_stream(descriptor_checksum_path);
    if (!checksum_stream) {
      throw std::runtime_error(
              "missing descriptor checksum: " + descriptor_checksum_path.string());
    }
    std::string expected;
    std::string filename;
    checksum_stream >> expected >> filename;
    requireSha256(expected, "descriptor_index.json.sha256");
    if (filename != result.descriptor_index_path.filename().string()) {
      throw std::runtime_error(
              "descriptor checksum names a different file: " + filename);
    }
    if (lower(expected) != actual_descriptor_sha) {
      throw std::runtime_error("descriptor index SHA-256 mismatch");
    }
  }
  const YAML::Node descriptor_root = YAML::Load(descriptor_document);
  if (required<std::string>(descriptor_root, "schema", "descriptor index") !=
    "go2.polar_descriptor_index/v1")
  {
    throw std::runtime_error("unsupported descriptor index schema");
  }
  if (required<std::string>(descriptor_root, "map_id", "descriptor index") != result.map_id) {
    throw std::runtime_error("descriptor index map_id does not match manifest");
  }
  (void)required<std::string>(descriptor_root, "created_utc", "descriptor index");
  const YAML::Node parameters = descriptor_root["parameters"];
  result.descriptor_config.rings = required<std::size_t>(parameters, "rings", "parameters");
  result.descriptor_config.sectors =
    required<std::size_t>(parameters, "sectors", "parameters");
  result.descriptor_config.max_radius_m =
    required<double>(parameters, "max_radius_m", "parameters");
  result.descriptor_config.min_radius_m = 0.0;
  result.descriptor_config.min_z_m = required<double>(parameters, "min_z_m", "parameters");
  result.descriptor_config.max_z_m = required<double>(parameters, "max_z_m", "parameters");
  if (required<std::string>(parameters, "value", "parameters") != "max_height_normalized") {
    throw std::runtime_error("unsupported descriptor value encoding");
  }
  validateConfig(result.descriptor_config);

  if (!descriptor_root["entries"] || !descriptor_root["entries"].IsSequence() ||
    descriptor_root["entries"].size() == 0)
  {
    throw std::runtime_error("descriptor index entries must be non-empty");
  }
  std::unordered_set<std::string> descriptor_ids;
  std::unordered_set<std::string> descriptor_tile_ids;
  const std::size_t value_count =
    result.descriptor_config.rings * result.descriptor_config.sectors;
  for (std::size_t index = 0; index < descriptor_root["entries"].size(); ++index) {
    const YAML::Node source = descriptor_root["entries"][index];
    const std::string context = "descriptor.entries[" + std::to_string(index) + "]";
    DescriptorRecord record;
    record.id = required<std::string>(source, "id", context);
    record.tile_id = required<std::string>(source, "tile_id", context);
    record.center = requiredVector3(source, "center", context);
    record.source_sha256 = lower(required<std::string>(source, "source_sha256", context));
    if (record.id.empty() || !descriptor_ids.insert(record.id).second) {
      throw std::runtime_error(context + ": empty or duplicate descriptor id");
    }
    if (result.tile_lookup_.count(record.tile_id) == 0) {
      throw std::runtime_error(context + ": unknown tile_id");
    }
    if (!descriptor_tile_ids.insert(record.tile_id).second) {
      throw std::runtime_error(context + ": duplicate descriptor for tile_id");
    }
    const auto & tile = result.tile(record.tile_id);
    if (record.source_sha256 != tile.sha256) {
      throw std::runtime_error(context + ": descriptor source hash does not match tile");
    }
    const double expected_center_x = (tile.grid_min[0] + tile.grid_max[0]) * 0.5;
    const double expected_center_y = (tile.grid_min[1] + tile.grid_max[1]) * 0.5;
    if (std::abs(record.center[0] - expected_center_x) > grid_tolerance ||
      std::abs(record.center[1] - expected_center_y) > grid_tolerance ||
      std::abs(record.center[2]) > grid_tolerance)
    {
      throw std::runtime_error(context + ": descriptor center does not match tile center");
    }
    record.descriptor.config = result.descriptor_config;
    record.descriptor.values = required<std::vector<float>>(source, "values", context);
    record.descriptor.ring_key = required<std::vector<float>>(source, "ring_key", context);
    record.descriptor.sector_key = required<std::vector<float>>(source, "sector_key", context);
    if (record.descriptor.values.size() != value_count ||
      record.descriptor.ring_key.size() != result.descriptor_config.rings ||
      record.descriptor.sector_key.size() != result.descriptor_config.sectors)
    {
      throw std::runtime_error(context + ": descriptor vector length mismatch");
    }
    if (!std::all_of(
        record.descriptor.values.begin(), record.descriptor.values.end(),
        [](float value) {return std::isfinite(value) && value >= 0.0F && value <= 1.0F;}))
    {
      throw std::runtime_error(context + ": descriptor values must be finite in [0,1]");
    }
    if (!std::all_of(
        record.descriptor.ring_key.begin(), record.descriptor.ring_key.end(),
        [](float value) {return std::isfinite(value);}) ||
      !std::all_of(
        record.descriptor.sector_key.begin(), record.descriptor.sector_key.end(),
        [](float value) {return std::isfinite(value);}))
    {
      throw std::runtime_error(context + ": descriptor keys must be finite");
    }
    result.descriptors.push_back(std::move(record));
  }
  if (result.descriptors.size() != result.tiles.size() ||
    descriptor_tile_ids.size() != tile_ids.size())
  {
    throw std::runtime_error(
            "descriptor index must cover every manifest tile exactly once");
  }
  for (const auto & tile_id : tile_ids) {
    if (descriptor_tile_ids.count(tile_id) == 0) {
      throw std::runtime_error(
              "descriptor index is missing tile_id: " + tile_id);
    }
  }
  return result;
}

const TileRecord & MapManifest::tile(const std::string & id) const
{
  const auto found = tile_lookup_.find(id);
  if (found == tile_lookup_.end()) {
    throw std::out_of_range("unknown tile id: " + id);
  }
  return tiles.at(found->second);
}

std::vector<const TileRecord *> MapManifest::tilesNear(
  double x, double y, double radius_m, std::size_t maximum) const
{
  std::vector<std::pair<double, const TileRecord *>> ranked;
  const double radius_squared = radius_m * radius_m;
  for (const auto & candidate : tiles) {
    const double distance = squaredDistanceToBounds(x, y, candidate.bounds);
    if (distance <= radius_squared) {
      ranked.emplace_back(distance, &candidate);
    }
  }
  std::sort(ranked.begin(), ranked.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });
  if (maximum > 0 && ranked.size() > maximum) {
    ranked.resize(maximum);
  }
  std::vector<const TileRecord *> output;
  output.reserve(ranked.size());
  for (const auto & pair : ranked) {
    output.push_back(pair.second);
  }
  return output;
}

}  // namespace go2_map_localizer

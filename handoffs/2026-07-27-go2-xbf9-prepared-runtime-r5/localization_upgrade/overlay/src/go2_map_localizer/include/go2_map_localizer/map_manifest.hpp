// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "go2_map_localizer/polar_descriptor.hpp"

namespace go2_map_localizer
{

struct Bounds3
{
  std::array<double, 3> min{};
  std::array<double, 3> max{};
};

struct TileRecord
{
  std::string id;
  int ix{0};
  int iy{0};
  std::filesystem::path path;
  std::string sha256;
  std::uint64_t point_count{0};
  Bounds3 bounds;
  std::array<double, 2> grid_min{};
  std::array<double, 2> grid_max{};
};

struct DescriptorRecord
{
  std::string id;
  std::string tile_id;
  std::array<double, 3> center{};
  std::string source_sha256;
  PolarDescriptor descriptor;
};

class MapManifest
{
public:
  static MapManifest load(const std::filesystem::path & manifest_path, bool verify_hashes);

  const TileRecord & tile(const std::string & id) const;
  std::vector<const TileRecord *> tilesNear(
    double x, double y, double radius_m, std::size_t maximum) const;

  std::filesystem::path manifest_path;
  std::filesystem::path root_directory;
  std::string schema;
  std::string map_id;
  std::string manifest_sha256;
  bool hashes_verified{false};
  std::filesystem::path descriptor_index_path;
  std::string descriptor_index_sha256;
  std::filesystem::path stable_layer_path;
  std::string stable_layer_sha256;
  std::uint64_t stable_layer_point_count{0};
  std::string stable_layer_role;
  std::string frame_id;
  std::string created_utc;
  std::string source_filename;
  std::string source_sha256;
  std::uint64_t source_point_count{0};
  double tile_size_m{20.0};
  double voxel_size_m{0.2};
  Bounds3 bounds;
  PolarDescriptorConfig descriptor_config;
  std::vector<TileRecord> tiles;
  std::vector<DescriptorRecord> descriptors;

private:
  std::unordered_map<std::string, std::size_t> tile_lookup_;
};

std::string sha256File(const std::filesystem::path & path);

}  // namespace go2_map_localizer

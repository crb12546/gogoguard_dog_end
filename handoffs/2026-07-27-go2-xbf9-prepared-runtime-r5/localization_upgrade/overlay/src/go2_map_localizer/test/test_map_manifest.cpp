// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/map_manifest.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <system_error>

namespace fs = std::filesystem;
using go2_map_localizer::MapManifest;
using go2_map_localizer::sha256File;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void writeFile(const fs::path & path, const std::string & contents)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write fixture: " + path.string());
  }
  output << contents;
  if (!output) {
    throw std::runtime_error("failed writing fixture: " + path.string());
  }
}

void expectFailure(const std::function<void()> & operation, const std::string & context)
{
  try {
    operation();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error("expected contract rejection: " + context);
}

std::string descriptorEntry(
  const std::string & id,
  const std::string & tile_id,
  double center_x,
  double center_z,
  const std::string & tile_sha)
{
  return
    "{\"id\":\"" + id + "\",\"tile_id\":\"" + tile_id +
    "\",\"center\":[" + std::to_string(center_x) + ",10.0," +
    std::to_string(center_z) + "],\"source_sha256\":\"" + tile_sha +
    "\",\"values\":[0.5,0.0,0.0,0.0],\"ring_key\":[0.125],"
    "\"sector_key\":[0.5,0.0,0.0,0.0]}";
}

class Fixture
{
public:
  Fixture()
  {
    const auto suffix = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    root = fs::temp_directory_path() / ("go2_map_manifest_test_" + suffix);
    fs::create_directories(root / "tiles");
    fs::create_directories(root / "review_assets");
    writeFile(root / "tiles/t0.pcd", "tile-zero\n");
    writeFile(root / "tiles/t1.pcd", "tile-one\n");
    writeFile(root / "review_assets/stable_layer.pcd", "stable-layer\n");
    tile0_sha = sha256File(root / "tiles/t0.pcd");
    tile1_sha = sha256File(root / "tiles/t1.pcd");
    stable_layer_sha = sha256File(root / "review_assets/stable_layer.pcd");
  }

  ~Fixture()
  {
    std::error_code error;
    fs::remove_all(root, error);
  }

  std::string writeDescriptor(const std::string & entries)
  {
    const std::string contents =
      "{\"schema\":\"go2.polar_descriptor_index/v1\","
      "\"map_id\":\"contract-map\",\"created_utc\":\"2026-07-25T00:00:00Z\","
      "\"source_layer\":{\"role\":\"global_retrieval\","
      "\"path\":\"review_assets/stable_layer.pcd\",\"sha256\":\"" +
      stable_layer_sha + "\",\"point_count\":1},"
      "\"parameters\":{\"rings\":1,\"sectors\":4,\"max_radius_m\":40.0,"
      "\"min_z_m\":-2.0,\"max_z_m\":4.0,"
      "\"value\":\"max_height_normalized\"},\"entries\":[" + entries + "]}";
    const fs::path path = root / "descriptor_index.json";
    writeFile(path, contents);
    const std::string hash = sha256File(path);
    writeFile(root / "descriptor_index.json.sha256", hash + "  descriptor_index.json\n");
    return hash;
  }

  void writeManifest(
    const std::string & schema,
    const std::string & descriptor_sha,
    const std::string & tile_size = "20.0",
    const std::string & voxel_size = "0.2")
  {
    const std::string contents =
      "{\"schema\":\"" + schema + "\",\"map_id\":\"contract-map\","
      "\"created_utc\":\"2026-07-25T00:00:00Z\",\"frame_id\":\"map\","
      "\"tile_size_m\":" + tile_size + ",\"voxel_size_m\":" + voxel_size + ","
      "\"bounds\":{\"min\":[1.0,1.0,0.0],\"max\":[22.0,2.0,1.0]},"
      "\"source\":{\"filename\":\"source.pcd\",\"sha256\":\"" +
      std::string(64, 'a') + "\",\"point_count\":2},"
      "\"descriptor_index\":{\"path\":\"descriptor_index.json\",\"sha256\":\"" +
      descriptor_sha + "\"},\"tiles\":["
      "{\"id\":\"t0\",\"ix\":0,\"iy\":0,\"path\":\"tiles/t0.pcd\","
      "\"sha256\":\"" + tile0_sha + "\",\"point_count\":1,"
      "\"bounds\":{\"min\":[1.0,1.0,0.0],\"max\":[2.0,2.0,1.0]},"
      "\"grid_bounds\":{\"min\":[0.0,0.0],\"max\":[20.0,20.0]}},"
      "{\"id\":\"t1\",\"ix\":1,\"iy\":0,\"path\":\"tiles/t1.pcd\","
      "\"sha256\":\"" + tile1_sha + "\",\"point_count\":1,"
      "\"bounds\":{\"min\":[21.0,1.0,0.0],\"max\":[22.0,2.0,1.0]},"
      "\"grid_bounds\":{\"min\":[20.0,0.0],\"max\":[40.0,20.0]}}]}";
    const fs::path path = root / "manifest.json";
    writeFile(path, contents);
    writeFile(root / "manifest.sha256", sha256File(path) + "  manifest.json\n");
  }

  std::string validEntries() const
  {
    return descriptorEntry("d0", "t0", 10.0, 0.0, tile0_sha) + "," +
           descriptorEntry("d1", "t1", 30.0, 0.0, tile1_sha);
  }

  fs::path root;
  std::string tile0_sha;
  std::string tile1_sha;
  std::string stable_layer_sha;
};

}  // namespace

int main(int argc, char * argv[])
{
  if (argc == 2) {
    const auto production = MapManifest::load(argv[1], true);
    require(!production.tiles.empty(), "production manifest contains no tiles");
    require(
      production.tiles.size() == production.descriptors.size(),
      "production manifest descriptor coverage mismatch");
    return 0;
  }
  require(argc == 1, "usage: test_map_manifest [manifest.json]");

  Fixture fixture;
  auto descriptor_sha = fixture.writeDescriptor(fixture.validEntries());
  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha);
  const auto valid = MapManifest::load(fixture.root / "manifest.json", true);
  require(valid.hashes_verified, "verified v2 bundle did not preserve integrity state");
  require(valid.tiles.size() == 2 && valid.descriptors.size() == 2, "valid v2 bundle failed");
  require(
    valid.stable_layer_path ==
    fs::weakly_canonical(fixture.root / "review_assets/stable_layer.pcd"),
    "stable-layer path was not bound to the descriptor index");
  require(
    valid.stable_layer_sha256 == fixture.stable_layer_sha &&
    valid.stable_layer_point_count == 1 &&
    valid.stable_layer_role == "global_retrieval",
    "stable-layer identity metadata was not preserved");

  writeFile(fixture.root / "review_assets/stable_layer.pcd", "tampered-layer\n");
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "stable-layer hash not bound to descriptor index");
  writeFile(fixture.root / "review_assets/stable_layer.pcd", "stable-layer\n");

  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha, ".nan", "0.2");
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "non-finite tile size");

  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha, "20.0", ".inf");
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "non-finite voxel size");

  descriptor_sha = fixture.writeDescriptor(
    descriptorEntry("d0", "t0", 10.0, 0.0, fixture.tile0_sha));
  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha);
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "missing per-tile descriptor");

  descriptor_sha = fixture.writeDescriptor(
    descriptorEntry("d0", "t0", 10.0, 0.0, fixture.tile0_sha) + "," +
    descriptorEntry("d1", "t0", 10.0, 0.0, fixture.tile0_sha));
  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha);
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "duplicate tile descriptor");

  descriptor_sha = fixture.writeDescriptor(
    descriptorEntry("d0", "t0", 10.0, 1.0, fixture.tile0_sha) + "," +
    descriptorEntry("d1", "t1", 30.0, 0.0, fixture.tile1_sha));
  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha);
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "non-planar descriptor center");

  descriptor_sha = fixture.writeDescriptor(fixture.validEntries());
  fixture.writeManifest("go2.map_tiles/v2", descriptor_sha);
  writeFile(fixture.root / "descriptor_index.json", "{\"tampered\":true}\n");
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "descriptor hash not bound to manifest");

  descriptor_sha = fixture.writeDescriptor(fixture.validEntries());
  fixture.writeManifest("go2.map_tiles/v1", descriptor_sha);
  expectFailure(
    [&fixture]() {
      (void)MapManifest::load(fixture.root / "manifest.json", true);
    },
    "legacy v1 verified load");

  return 0;
}

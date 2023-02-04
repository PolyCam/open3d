// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <tiny_obj_loader.h>

#include <chrono>
#include <fstream>
#include <numeric>
#include <random>
#include <vector>

#include "open3d/io/FileFormatIO.h"
#include "open3d/io/ImageIO.h"
#include "open3d/io/TriangleMeshIO.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/FileSystem.h"

#define USE_MULTITHREADING
#ifdef USE_MULTITHREADING
#include "open3d/utility/Parallelize.h"
#endif

#include "fmt/core.h"

namespace open3d {
namespace io {

std::string random_string(std::size_t length) {
  const std::string CHARACTERS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_int_distribution<> distribution(0, CHARACTERS.size() - 1);

  std::string random_string;

  for (std::size_t i = 0; i < length; ++i) {
    random_string += CHARACTERS[distribution(generator)];
  }
  return random_string;
}

FileGeometry ReadFileGeometryTypeOBJ(const std::string &path) { return FileGeometry(CONTAINS_TRIANGLES | CONTAINS_POINTS); }

bool ReadTriangleMeshFromOBJ(const std::string &filename, geometry::TriangleMesh &mesh, bool print_progress) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn;
  std::string err;

  std::string mtl_base_path = utility::filesystem::GetFileParentDirectory(filename);
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str(), mtl_base_path.c_str());
  if (!warn.empty()) {
    utility::LogWarning("Read OBJ failed: {}", warn);
  }
  if (!err.empty()) {
    utility::LogWarning("Read OBJ failed: {}", err);
  }
  if (!ret) {
    return false;
  }

  mesh.Clear();

  // copy vertex and data
  for (size_t vidx = 0; vidx < attrib.vertices.size(); vidx += 3) {
    tinyobj::real_t vx = attrib.vertices[vidx + 0];
    tinyobj::real_t vy = attrib.vertices[vidx + 1];
    tinyobj::real_t vz = attrib.vertices[vidx + 2];
    mesh.vertices_.push_back(Eigen::Vector3d(vx, vy, vz));
  }

  for (size_t vidx = 0; vidx < attrib.colors.size(); vidx += 3) {
    tinyobj::real_t r = attrib.colors[vidx + 0];
    tinyobj::real_t g = attrib.colors[vidx + 1];
    tinyobj::real_t b = attrib.colors[vidx + 2];
    mesh.vertex_colors_.push_back(Eigen::Vector3d(r, g, b));
  }

  // resize normal data and create bool indicator vector
  mesh.vertex_normals_.resize(mesh.vertices_.size());
  std::vector<bool> normals_indicator(mesh.vertices_.size(), false);

  // copy face data and copy normals data
  // append face-wise uv data
  for (size_t s = 0; s < shapes.size(); s++) {
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      int fv = shapes[s].mesh.num_face_vertices[f];
      if (fv != 3) {
        utility::LogWarning(
            "Read OBJ failed: facet with number of vertices not "
            "equal to 3");
        return false;
      }

      Eigen::Vector3i facet;
      for (int v = 0; v < fv; v++) {
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        int vidx = idx.vertex_index;
        facet(v) = vidx;

        if (!attrib.normals.empty() && !normals_indicator[vidx] && (3 * idx.normal_index + 2) < int(attrib.normals.size())) {
          tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
          tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
          tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
          mesh.vertex_normals_[vidx](0) = nx;
          mesh.vertex_normals_[vidx](1) = ny;
          mesh.vertex_normals_[vidx](2) = nz;
          normals_indicator[vidx] = true;
        }

        if (!attrib.texcoords.empty() && 2 * idx.texcoord_index + 1 < int(attrib.texcoords.size())) {
          tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
          tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];
          mesh.triangle_uvs_.push_back(Eigen::Vector2d(tx, ty));
        }
      }
      mesh.triangles_.push_back(facet);
      mesh.triangle_material_ids_.push_back(shapes[s].mesh.material_ids[f]);
      index_offset += fv;
    }
  }

  // if not all normals have been set, then remove the vertex normals
  bool all_normals_set = std::accumulate(normals_indicator.begin(), normals_indicator.end(), true, [](bool a, bool b) { return a && b; });
  if (!all_normals_set) {
    mesh.vertex_normals_.clear();
  }

  // if not all triangles have corresponding uvs, then remove uvs
  if (3 * mesh.triangles_.size() != mesh.triangle_uvs_.size()) {
    mesh.triangle_uvs_.clear();
  }

  auto textureLoader = [&mtl_base_path](std::string &relativePath) {
    auto image = io::CreateImageFromFile(mtl_base_path + relativePath);
    return image->HasData() ? image : std::shared_ptr<geometry::Image>();
  };

  using MaterialParameter = geometry::TriangleMesh::Material::MaterialParameter;

  for (auto &material : materials) {
    auto &meshMaterial = mesh.materials_[material.name];

    meshMaterial.baseColor = MaterialParameter::CreateRGB(material.diffuse[0], material.diffuse[1], material.diffuse[2]);

    if (!material.normal_texname.empty()) {
      meshMaterial.normalMap = textureLoader(material.normal_texname)->FlipVertical();
    } else if (!material.bump_texname.empty()) {
      // try bump, because there is often a misunderstanding of
      // what bump map or normal map is
      meshMaterial.normalMap = textureLoader(material.bump_texname)->FlipVertical();
    }

    if (!material.ambient_occlusion_texname.empty()) {
      meshMaterial.ambientOcclusion = textureLoader(material.ambient_occlusion_texname)->FlipVertical();
    }

    if (!material.diffuse_texname.empty()) {
      meshMaterial.albedo = textureLoader(material.diffuse_texname)->FlipVertical();
      mesh.textures_names_.push_back(material.diffuse_texname);

      // Legacy texture map support
      if (meshMaterial.albedo) {
        mesh.textures_.push_back(*meshMaterial.albedo);
      }
    }

    if (!material.metallic_texname.empty()) {
      meshMaterial.metallic = textureLoader(material.metallic_texname);
    }

    if (!material.roughness_texname.empty()) {
      meshMaterial.roughness = textureLoader(material.roughness_texname)->FlipVertical();
      std::cout << "Loaded roughness from OBJ file " << std::endl;
    }

    if (!material.sheen_texname.empty()) {
      meshMaterial.reflectance = textureLoader(material.sheen_texname);
    }

    // NOTE: We want defaults of 0.0 and 1.0 for baseMetallic and
    // baseRoughness respectively but 0.0 is a valid value for both and
    // tiny_obj_loader defaults to 0.0 for both. So, we will assume that
    // only if one of the values is greater than 0.0 that there are
    // non-default values set in the .mtl file
    if (material.roughness > 0.f || material.metallic > 0.f) {
      meshMaterial.baseMetallic = material.metallic;
      meshMaterial.baseRoughness = material.roughness;
    }

    if (material.sheen > 0.f) {
      meshMaterial.baseReflectance = material.sheen;
    }

    // NOTE: We will unconditionally copy the following parameters because
    // the TinyObj defaults match Open3D's internal defaults
    meshMaterial.baseClearCoat = material.clearcoat_thickness;
    meshMaterial.baseClearCoatRoughness = material.clearcoat_roughness;
    meshMaterial.baseAnisotropy = material.anisotropy;
  }

  return true;
}

bool WriteTriangleMeshToOBJ(const std::string &filename, const geometry::TriangleMesh &mesh, bool write_ascii /* = false*/,
                            bool compressed /* = false*/, bool write_vertex_normals /* = true*/, bool write_vertex_colors /* = true*/,
                            bool write_triangle_uvs /* = true*/, bool print_progress) {
  const auto timer_start = std::chrono::high_resolution_clock::now();
  std::string object_name = utility::filesystem::GetFileNameWithoutExtension(utility::filesystem::GetFileNameWithoutDirectory(filename));

  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
  if (!file) {
    utility::LogWarning("Write OBJ failed: unable to open file.");
    return false;
  }

  if (mesh.HasTriangleNormals()) {
    utility::LogWarning("Write OBJ can not include triangle normals.");
  }

  const auto split_timer_start = std::chrono::high_resolution_clock::now();

  auto out = fmt::memory_buffer();
  format_to(std::back_inserter(out), "# Created by Polycam with FMT\n");
  format_to(std::back_inserter(out), "# object name: {}\n", object_name);
  format_to(std::back_inserter(out), "# number of vertices: {}\n", mesh.vertices_.size());
  format_to(std::back_inserter(out), "# number of triangles: {}\n", mesh.triangles_.size());

  utility::ConsoleProgressBar progress_bar(mesh.vertices_.size() + mesh.triangles_.size(), "Writing OBJ: ", print_progress);

  // we are less strict and allows writing to uvs without known material
  // potentially this will be useful for exporting conformal map generation
  write_triangle_uvs = write_triangle_uvs && mesh.HasTriangleUvs_Any();

  // write material filename only when uvs is written or has textures
  if (write_triangle_uvs) {
    format_to(std::back_inserter(out), "mtllib {}.mtl\n", object_name.c_str());
  }

  write_vertex_normals = write_vertex_normals && mesh.HasVertexNormals();
  write_vertex_colors = write_vertex_colors && mesh.HasVertexColors();
  for (size_t vidx = 0; vidx < mesh.vertices_.size(); ++vidx) {
    const Eigen::Vector3d &vertex = mesh.vertices_[vidx];
    format_to(std::back_inserter(out), "v {:.6f} {:.6f} {:.6f}", vertex(0), vertex(1), vertex(2));
    if (write_vertex_colors) {
      const Eigen::Vector3d &color = mesh.vertex_colors_[vidx];
      format_to(std::back_inserter(out), " {:.6f} {:.6f} {:.6f}", color(0), color(1), color(2));
    }
    format_to(std::back_inserter(out), "\n");

    if (write_vertex_normals) {
      const Eigen::Vector3d &normal = mesh.vertex_normals_[vidx];
      format_to(std::back_inserter(out), "vn {:.6f} {:.6f} {:.6f}\n", normal(0), normal(1), normal(2));
    }
    ++progress_bar;
  }

  // we don't compress uvs into vertex-wise representation.
  // loose triangle-wise representation is provided
  if (write_triangle_uvs) {
    for (auto &uv : mesh.triangle_uvs_) {
      // Note CH: We reflect the y-coordinate of the UVs because this is necessary to achieve the correct
      // orientation of the materials. We used to flip the materials instead, but this is faster.
      format_to(std::back_inserter(out), "vt {:.6f} {:.6f}\n", uv(0), 1.0 - uv(1));
    }
  }

  // write faces with (possibly multiple) material ids
  // map faces with material ids
  std::map<int, std::vector<size_t>> material_id_faces_map;
  if (mesh.HasTriangleMaterialIds()) {
    for (size_t i = 0; i < mesh.triangle_material_texture_ids_.size(); ++i) {
      int mi = mesh.triangle_material_texture_ids_[i];
      auto it = material_id_faces_map.find(mi);
      if (it == material_id_faces_map.end()) {
        material_id_faces_map[mi] = {i};
      } else {
        it->second.push_back(i);
      }
    }
  } else {  // every face falls to the default material
    material_id_faces_map[0].resize(mesh.triangles_.size());
    std::iota(material_id_faces_map[0].begin(), material_id_faces_map[0].end(), 0);
  }

  // enumerate ids and their corresponding faces
  for (auto it = material_id_faces_map.begin(); it != material_id_faces_map.end(); ++it) {
    // write the mtl name
    if (write_triangle_uvs) {
      std::string mtl_name = object_name + "_" + std::to_string(it->first);
      format_to(std::back_inserter(out), "usemtl {}\n", mtl_name.c_str());
    }

    // write the corresponding faces
    for (auto tidx : it->second) {
      const Eigen::Vector3i &triangle = mesh.triangles_[tidx];
      const Eigen::Vector3i &triangle_uvs_idx = mesh.triangles_uvs_idx_[tidx];
      bool write_triangle_uv = triangle_uvs_idx(0) >= 0 && triangle_uvs_idx(1) >= 0 && triangle_uvs_idx(2) >= 0;
      if (write_vertex_normals && write_triangle_uv) {
        format_to(std::back_inserter(out), "f {}/{}/{} {}/{}/{} {}/{}/{}\n", triangle(0) + 1, 3 * tidx + 1, triangle(0) + 1, triangle(1) + 1,
                  3 * tidx + 2, triangle(1) + 1, triangle(2) + 1, 3 * tidx + 3, triangle(2) + 1);
      } else if (!write_vertex_normals && write_triangle_uv) {
        format_to(std::back_inserter(out), "f {}/{} {}/{} {}/{}\n", triangle(0) + 1, 3 * tidx + 1, triangle(1) + 1, 3 * tidx + 2, triangle(2) + 1,
                  3 * tidx + 3);
      } else if (write_vertex_normals && !write_triangle_uv) {
        format_to(std::back_inserter(out), "f {}//{} {}//{} {}//{}\n", triangle(0) + 1, triangle(0) + 1, triangle(1) + 1, triangle(1) + 1,
                  triangle(2) + 1, triangle(2) + 1);
      } else {
        format_to(std::back_inserter(out), "f {} {} {}\n", triangle(0) + 1, triangle(1) + 1, triangle(2) + 1);
      }
      ++progress_bar;
    }
  }
  file << out.data();
  file.flush();
  file.close();
  const auto split_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - split_timer_start).count();
  std::cout << "\tWriteTriangleMeshToOBJ: OBJ file alone took " << split_duration << " ms." << std::endl;
  // end of writing obj.
  //////

  //////
  // write mtl file when uvs are written
  {
    // start to write to mtl and texture
    std::string parent_dir = utility::filesystem::GetFileParentDirectory(filename);
    std::string mtl_filename = parent_dir + object_name + ".mtl";

    // write headers
    std::ofstream mtl_file(mtl_filename.c_str(), std::ios::out);
    if (!mtl_file) {
      utility::LogWarning("Write OBJ successful, but failed to write material file.");
      return true;
    }

    bool add_random_postfix = true;
    std::string material_postfix = ".jpg";
    if (add_random_postfix) {
      material_postfix = "_" + random_string(8) + ".jpg";
    }

    mtl_file << "# Created by Polycam\n";
    mtl_file << "# object name: " << object_name << "\n";
    int material_idx = 0;
    for (auto it : mesh.materials_) {
      const geometry::TriangleMesh::Material &material = it.second;
      int texture_idx = material.gltfExtras.texture_idx;
      std::string mtl_name = object_name + "_" + std::to_string(material_idx);
      std::string tex_name = object_name + "_" + std::to_string(texture_idx);
      if (texture_idx < 0) { // Solid color - not a texture
        const auto &spectral = material.gltfExtras.emissiveFactor;
        mtl_file << "newmtl " << mtl_name << "\n";
        mtl_file << "Ka " << material.baseColor.r() << " " << material.baseColor.g() << " " << material.baseColor.b() << "\n";
        mtl_file << "Kd 1.000 1.000 1.000\n";
        if (spectral.has_value())
          mtl_file << "Ks " << spectral.value()(0) << " " << spectral.value()(1) << " " << spectral.value()(2) << "\n";
        else
          mtl_file << "Ks 0.000 0.000 0.000\n";
        mtl_file << "Tr " << 1.0 - material.baseColor.a() << "\n";
        mtl_file << "illum 1\n";
        mtl_file << "Ns 1.000000\n"; // Spectral exponent
        mtl_file << "map_Kd " << mtl_name << material_postfix << "\n";
      } else { // Texture
        mtl_file << "newmtl " << mtl_name << "\n";
        mtl_file << "Ka 0.000 0.000 0.000\n";
        mtl_file << "Kd 1.000 1.000 1.000\n";
        mtl_file << "Ks 0.000 0.000 0.000\n";
        mtl_file << "Tr 0.000000\n";
        mtl_file << "illum 1\n";
        mtl_file << "Ns 1.000000\n";
        mtl_file << "map_Kd " << tex_name << material_postfix << "\n";
        if (material.normalMap)
          mtl_file << "normal " << mtl_name << "_norm" << material_postfix << "\n";
        if (material.ambientOcclusion)
          mtl_file << "map_ao " << mtl_name << "_occl" << material_postfix << "\n";
        if (material.roughness)
          mtl_file << "map_Pr " << mtl_name << "_roughness" << material_postfix << "\n";
      }
      material_idx++;
    }

    // write textures (if existing)
    auto write_texture = [&](size_t i) {
      // Don't write images for which no face was seen.
      if (material_id_faces_map.find(i) == material_id_faces_map.end())
        return;
      std::string tex_name = object_name + "_" + std::to_string(i);
      std::string tex_filename = parent_dir + tex_name + material_postfix;

      if (!io::WriteImage(tex_filename, mesh.textures_[i])) {
        utility::LogWarning("Write OBJ successful, but failed to write texture file.");
      }

      if (i < mesh.materials_.size()) {
        auto it = mesh.materials_.begin();
        std::advance(it, i);
        const geometry::TriangleMesh::Material &material = it->second;
        if (material.normalMap) {
          std::string tex_filename = parent_dir + tex_name + "_norm" + material_postfix;
          if (!io::WriteImage(tex_filename, *material.normalMap)) {
            utility::LogWarning("Write OBJ successful, but failed to write texture file.");
          }
        }
        if (material.ambientOcclusion) {
          std::string tex_filename = parent_dir + tex_name + "_occl" + material_postfix;
          if (!io::WriteImage(tex_filename, *material.ambientOcclusion)) {
            utility::LogWarning("Write OBJ successful, but failed to write AO file.");
          }
        }
        if (material.roughness) {
          std::string tex_filename = parent_dir + tex_name + "_roughness" + material_postfix;
          if (!io::WriteImage(tex_filename, *material.roughness)) {
            utility::LogWarning("Write OBJ successful, but failed to write roughness map.");
          }
        }
      }
    };

#ifdef USE_MULTITHREADING
    utility::Parallelize(write_texture, mesh.textures_.size());
#else
    for (size_t i = 0; i < mesh.textures_.size(); ++i) {
      write_texture(i);
    }
#endif

    // write the default material
    if (!mesh.HasTextures()) {
      std::string mtl_name = object_name + "_0";
      mtl_file << "newmtl " << mtl_name << "\n";
      mtl_file << "Ka 1.000 1.000 1.000\n";
      mtl_file << "Kd 1.000 1.000 1.000\n";
      mtl_file << "Ks 0.000 0.000 0.000\n";
    }
  }

  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timer_start).count();
  std::cout << "\tWriteTriangleMeshToOBJ took " << duration << " ms." << std::endl;

  return true;
}  // namespace io

}  // namespace io
}  // namespace open3d

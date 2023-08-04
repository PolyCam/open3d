#include <open3d/geometry/Reorganization.h>
#include <open3d/geometry/TriangleMesh.h>

#include <algorithm>
#include <cassert>
#include <list>
#include <unordered_map>
#include <vector>

namespace open3d::geometry {

bool DuplicateConsolidation::ShouldConsolidate() const {
  return (consolidated_to_original_indices_.size() < original_to_consolidated_indices_.size());
}

template <typename Element>
DuplicateConsolidation GetConsolidation(const std::vector<Element> &originals, bool (*order_comparator)(const Element &first, const Element &second),
                                        bool (*equality_comparator)(const Element &first, const Element &second)) {
  if (originals.empty()) {
    return (DuplicateConsolidation());
  }

  using Proxy = unsigned int;  // Index into originals.
  auto proxies = std::vector<Proxy>();
  proxies.reserve(originals.size());
  while (proxies.size() < originals.size()) {
    proxies.push_back(proxies.size());
  }
  auto proxy_comparator = [&](Proxy first_proxy, Proxy second_proxy) {
    return ((*order_comparator)(originals[first_proxy], originals[second_proxy]));
  };
  std::sort(proxies.begin(), proxies.end(), proxy_comparator);

  auto consolidation = DuplicateConsolidation();
  consolidation.original_to_consolidated_indices_.resize(originals.size());
  consolidation.consolidated_to_original_indices_.reserve(originals.size());
  const auto *last_equivalent_value = &originals[proxies.front()];
  consolidation.consolidated_to_original_indices_.push_back(proxies.front());

  for (const auto proxy : proxies) {
    const auto &value = originals[proxy];
    if (!(*equality_comparator)(value, *last_equivalent_value)) {
      last_equivalent_value = &value;
      consolidation.consolidated_to_original_indices_.push_back(proxy);
    }
    consolidation.original_to_consolidated_indices_[proxy] = consolidation.consolidated_to_original_indices_.size() - 1u;
  }

  assert(consolidation.original_to_consolidated_indices_.size() == originals.size());
  consolidation.consolidated_to_original_indices_.shrink_to_fit();
  return (consolidation);
}

template <typename Element>
void ApplyConsolidation(std::vector<Element> &elements, const DuplicateConsolidation &consolidation) {
  assert(consolidation.original_to_consolidated_indices_.size() == elements.size());
  assert(consolidation.consolidated_to_original_indices_.size() <= elements.size());
  auto consolidated_elements = std::vector<Element>();
  consolidated_elements.reserve(consolidation.consolidated_to_original_indices_.size());
  for (const auto original_coordinates_index : consolidation.consolidated_to_original_indices_) {
    assert(original_coordinates_index < elements.size());
    consolidated_elements.push_back(elements[original_coordinates_index]);
  }
  elements = std::move(consolidated_elements);
}

static bool CompareTextureCoordinatesForOrder(const Eigen::Vector2d &first, const Eigen::Vector2d &second) {
  if (first.x() != second.x()) {
    return (first.x() < second.x());
  }
  return (first.y() < second.y());
}

static bool CompareTextureCoordinatesForEquality(const Eigen::Vector2d &first, const Eigen::Vector2d &second) { return (first == second); }

DuplicateConsolidation GetTextureCoordinatesConsolidation(const TriangleMesh &mesh) {
  return (GetConsolidation(mesh.triangle_uvs_, &CompareTextureCoordinatesForOrder, &CompareTextureCoordinatesForEquality));
}

void ConsolidateTextureCoordinates(TriangleMesh &mesh) { ConsolidateTextureCoordinates(mesh, GetTextureCoordinatesConsolidation(mesh)); }

void ConsolidateTextureCoordinates(TriangleMesh &mesh, const DuplicateConsolidation &consolidation) {
  if (!consolidation.ShouldConsolidate()) {
    return;
  }
  ApplyConsolidation(mesh.triangle_uvs_, consolidation);
  for (auto &triangle_uv_indices : mesh.triangles_uvs_idx_) {
    for (auto vertex_in_triangle_index = 0u; vertex_in_triangle_index < 3u; ++vertex_in_triangle_index) {
      auto &coordinates_index = triangle_uv_indices[vertex_in_triangle_index];
      if (coordinates_index >= 0) {
        assert(coordinates_index < consolidation.original_to_consolidated_indices_.size());
        coordinates_index = consolidation.original_to_consolidated_indices_[coordinates_index];
        assert(coordinates_index < consolidation.consolidated_to_original_indices_.size());
      }
    }
  }
}

struct TexturedVertex {
  unsigned int vertex_coordinates_;   // Index into TriangleMesh::vertices_.
  unsigned int texture_coordinates_;  // Index into TriangleMesh::triangle_uvs_.
};

static bool operator==(const TexturedVertex &first, const TexturedVertex &second) {
  return (first.vertex_coordinates_ == second.vertex_coordinates_ && first.texture_coordinates_ == second.texture_coordinates_);
}

static const auto unsigned_int_hasher = std::hash<unsigned int>();

static std::size_t HashTexturedVertex(const TexturedVertex &indices) {
  return (unsigned_int_hasher(indices.vertex_coordinates_) + unsigned_int_hasher(indices.texture_coordinates_));
}

static void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh, const DuplicateConsolidation *texture_coordinates_consolidation) {
  assert(mesh.triangles_uvs_idx_.size() == mesh.triangles_.size());
  assert((texture_coordinates_consolidation != nullptr)
             ? (texture_coordinates_consolidation->original_to_consolidated_indices_.size() == mesh.triangle_uvs_.size())
             : true);
  assert((texture_coordinates_consolidation != nullptr)
             ? (texture_coordinates_consolidation->consolidated_to_original_indices_.size() < mesh.triangle_uvs_.size())
             : true);
  auto vertices = std::vector<Eigen::Vector3d>();
  vertices.reserve(mesh.triangles_.size() * 3u);  // Worst case scenario.
  auto texture_coordinates = std::vector<Eigen::Vector2d>();
  texture_coordinates.reserve(mesh.triangles_.size() * 3u);  // Worst case scenario.
  auto duplication_eliminator =
      std::unordered_map<TexturedVertex, unsigned int /* index into vertices and texture_coordinates */, decltype(&HashTexturedVertex)>(
          mesh.triangles_.size() * 3u, &HashTexturedVertex);
  for (auto triangle_index = 0u; triangle_index < mesh.triangles_.size(); ++triangle_index) {
    auto &triangle = mesh.triangles_[triangle_index];
    auto &triangle_uv_indices = mesh.triangles_uvs_idx_[triangle_index];
    for (auto vertex_in_triangle = 0u; vertex_in_triangle < 3u; ++vertex_in_triangle) {
      auto &vertex = triangle[vertex_in_triangle];
      assert(vertex >= 0);
      assert(vertex < mesh.vertices_.size());
      auto &vertex_uv_indices = triangle_uv_indices[vertex_in_triangle];
      assert(vertex_uv_indices >= 0);
      assert(vertex_uv_indices < mesh.triangle_uvs_.size());
      const auto insertion_result = duplication_eliminator.insert(std::make_pair(
          TexturedVertex{(unsigned int)vertex, (texture_coordinates_consolidation != nullptr)
                                                   ? texture_coordinates_consolidation->original_to_consolidated_indices_[vertex_uv_indices]
                                                   : vertex_uv_indices},
          (unsigned int)vertices.size()));
      if (insertion_result.second) {  // If a new vertex, uv index pair was found.
        vertices.push_back(mesh.vertices_[vertex]);
        texture_coordinates.push_back(mesh.triangle_uvs_[vertex_uv_indices]);
      }
      vertex = insertion_result.first->second;
      vertex_uv_indices = vertex;
    }
  }
  vertices.shrink_to_fit();
  mesh.vertices_ = std::move(vertices);
  texture_coordinates.shrink_to_fit();
  mesh.triangle_uvs_ = std::move(texture_coordinates);
}

void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh, const DuplicateConsolidation &texture_coordinates_consolidation) {
  ConsolidateTextureCoordinateIndices(mesh, &texture_coordinates_consolidation);
}

void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh) { ConsolidateTextureCoordinateIndices(mesh, nullptr); }

static bool CompareMaterialsForOrder(const TriangleMesh::Material &first, const TriangleMesh::Material &second) {
  return (first.IsBeforeIgnoringName(second));
}
static bool CompareMaterialsForEquality(const TriangleMesh::Material &first, const TriangleMesh::Material &second) {
  return (first.IsEqualIgnoringName(second));
}

DuplicateConsolidation GetMaterialConsolidation(const TriangleMesh &mesh) {
  return (GetConsolidation(mesh.materials_, &CompareMaterialsForOrder, &CompareMaterialsForEquality));
}

void ConsolidateMaterials(TriangleMesh &mesh) { ConsolidateMaterials(mesh, GetMaterialConsolidation(mesh)); }

void ConsolidateMaterials(TriangleMesh &mesh, const DuplicateConsolidation &consolidation) {
  if (!consolidation.ShouldConsolidate()) {
    return;
  }
  ApplyConsolidation(mesh.materials_, consolidation);
  for (auto &triangle_material : mesh.triangle_material_ids_) {
    assert(triangle_material < consolidation.original_to_consolidated_indices_.size());
    triangle_material = consolidation.original_to_consolidated_indices_[triangle_material];
    assert(triangle_material < consolidation.consolidated_to_original_indices_.size());
  }
}

static MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh, const DuplicateConsolidation *consolidation) {
  assert(mesh.triangle_material_ids_.size() == mesh.triangles_.size());
  auto intermediate_materials_triangle_usage = std::vector<std::list<unsigned int>>(
      (consolidation != nullptr) ? consolidation->consolidated_to_original_indices_.size() : mesh.materials_.size());
  if (consolidation != nullptr) {
    assert(consolidation->original_to_consolidated_indices_.size() == mesh.materials_.size());
    for (auto triangle = 0u; triangle < mesh.triangles_.size(); ++triangle) {
      const auto original_material = mesh.triangle_material_ids_[triangle];
      assert(original_material < consolidation->original_to_consolidated_indices_.size());
      const auto consolidated_material = consolidation->original_to_consolidated_indices_[original_material];
      assert(consolidated_material < consolidation->consolidated_to_original_indices_.size());
      intermediate_materials_triangle_usage[consolidated_material].push_back(triangle);
    }
  } else {
    for (auto triangle = 0u; triangle < mesh.triangles_.size(); ++triangle) {
      const auto material = mesh.triangle_material_ids_[triangle];
      assert(material < mesh.materials_.size());
      intermediate_materials_triangle_usage[material].push_back(triangle);
    }
  }
  auto materials_triangle_usage = MaterialsTriangleUsage();
  materials_triangle_usage.reserve(intermediate_materials_triangle_usage.size());
  for (const auto &intermediate_material_summary : intermediate_materials_triangle_usage) {
    auto material_triangle_usage = MaterialTriangleUsage();
    material_triangle_usage.reserve(intermediate_material_summary.size());
    for (const auto triangle : intermediate_material_summary) {
      material_triangle_usage.push_back(triangle);
    }
    materials_triangle_usage.push_back(std::move(material_triangle_usage));
  }
  return (materials_triangle_usage);
}

MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh) { return (GetMaterialsTriangleUsage(mesh, nullptr)); }

MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh, const DuplicateConsolidation &consolidation) {
  return (GetMaterialsTriangleUsage(mesh, &consolidation));
}

struct OnlyInUseConsolidation {
  std::unordered_map<unsigned int, unsigned int> original_to_consolidated_indices_;
  std::vector<unsigned int> consolidated_to_original_indices_;
};

static OnlyInUseConsolidation ConsolidateOnlyInUseVertices(const std::vector<Eigen::Vector3i> &triangles,
                                                           const MaterialTriangleUsage &material_triangle_usage) {
  auto consolidation = OnlyInUseConsolidation();
  consolidation.original_to_consolidated_indices_.reserve(triangles.size() * 3u);  // Worst case scenario.
  consolidation.consolidated_to_original_indices_.reserve(triangles.size() * 3u);  // Worst case scenario.
  for (const auto triangle_index : material_triangle_usage) {
    assert(triangle_index >= 0);
    assert(triangle_index < triangles.size());
    const auto &triangle = triangles[triangle_index];
    for (auto vertex_in_triangle_index = 0u; vertex_in_triangle_index < 3u; ++vertex_in_triangle_index) {
      const auto original_vertex_index = triangle[vertex_in_triangle_index];
      if (consolidation.original_to_consolidated_indices_
              .insert(std::make_pair(original_vertex_index, consolidation.consolidated_to_original_indices_.size()))
              .second) {
        consolidation.consolidated_to_original_indices_.push_back(original_vertex_index);
      }
    }
  }
  consolidation.consolidated_to_original_indices_.shrink_to_fit();
  return (consolidation);
}

static std::vector<Eigen::Vector3i> GetSingleMaterialMeshVertexIndices(const std::vector<Eigen::Vector3i> &original_indices,
                                                                       const OnlyInUseConsolidation &vertices_in_use_consolidation,
                                                                       const MaterialTriangleUsage &material_triangle_usage) {
  auto new_indices = std::vector<Eigen::Vector3i>();
  new_indices.reserve(material_triangle_usage.size());
  for (const auto used_triangle_index : material_triangle_usage) {
    assert(used_triangle_index < original_indices.size());
    const auto &used_triangle = original_indices[used_triangle_index];
    auto new_triangle = Eigen::Vector3i();
    for (auto vertex_in_triangle_index = 0u; vertex_in_triangle_index < 3u; ++vertex_in_triangle_index) {
      const auto consolidated_vertex = vertices_in_use_consolidation.original_to_consolidated_indices_.find(used_triangle[vertex_in_triangle_index]);
      assert(consolidated_vertex != vertices_in_use_consolidation.original_to_consolidated_indices_.end());
      new_triangle[vertex_in_triangle_index] = consolidated_vertex->second;
    }
    new_indices.push_back(new_triangle);
  }
  return (new_indices);
}

template <typename VertexAttribute>
static std::vector<VertexAttribute> GetSingleMaterialMeshVertexAttributes(const std::vector<VertexAttribute> &original_attributes,
                                                                          const OnlyInUseConsolidation &vertices_in_use_consolidation) {
  auto new_attributes = std::vector<VertexAttribute>();
  new_attributes.reserve(vertices_in_use_consolidation.consolidated_to_original_indices_.size());
  for (const auto vertex_index : vertices_in_use_consolidation.consolidated_to_original_indices_) {
    assert(vertex_index < original_attributes.size());
    new_attributes.push_back(original_attributes[vertex_index]);
  }
  return (new_attributes);
}

static std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh, const DuplicateConsolidation *material_consolidation) {
  const auto has_material_consolidation = (material_consolidation != nullptr);
  const auto mesh_count = (has_material_consolidation ? material_consolidation->consolidated_to_original_indices_.size() : mesh.materials_.size());
  auto single_material_meshes = std::vector<TriangleMesh>();
  single_material_meshes.reserve(mesh_count);
  const auto materials_triangle_usage = GetMaterialsTriangleUsage(mesh, material_consolidation);
  assert(materials_triangle_usage.size() == mesh_count);
  for (auto mesh_index = 0u; mesh_index < mesh_count; ++mesh_index) {
    const auto &material_triangle_usage = materials_triangle_usage[mesh_index];
    const auto &material =
        mesh.materials_[has_material_consolidation ? material_consolidation->consolidated_to_original_indices_[mesh_index] : mesh_index];
    const auto vertices_in_use_consolidation = ConsolidateOnlyInUseVertices(mesh.triangles_, material_triangle_usage);
    auto single_material_mesh = TriangleMesh();

    // Add the triangles.
    single_material_mesh.triangles_ = GetSingleMaterialMeshVertexIndices(mesh.triangles_, vertices_in_use_consolidation, material_triangle_usage);

    // Add the vertices.
    single_material_mesh.vertices_ = GetSingleMaterialMeshVertexAttributes(mesh.vertices_, vertices_in_use_consolidation);

    // Add the vertex normals, if needed.
    if (mesh.HasVertexNormals()) {
      single_material_mesh.vertex_normals_ = GetSingleMaterialMeshVertexAttributes(mesh.vertex_normals_, vertices_in_use_consolidation);
    }

    // Add the vertex colors, if needed.
    if (mesh.HasVertexColors()) {
      single_material_mesh.vertex_colors_ = GetSingleMaterialMeshVertexAttributes(mesh.vertex_colors_, vertices_in_use_consolidation);
    }

    // Add the triangle normals, if needed.
    if (mesh.HasTriangleNormals()) {
      single_material_mesh.triangle_normals_.reserve(material_triangle_usage.size());
      for (const auto used_triangle_index : material_triangle_usage) {
        assert(used_triangle_index < mesh.triangle_normals_.size());
        single_material_mesh.triangle_normals_.push_back(mesh.triangle_normals_[used_triangle_index]);
      }
    }

    // Add the texture coordinates, if needed.
    if (material.IsTextured()) {
      const auto texture_coordinates_in_use_consolidation = ConsolidateOnlyInUseVertices(mesh.triangles_uvs_idx_, material_triangle_usage);
      single_material_mesh.triangles_uvs_idx_ =
          GetSingleMaterialMeshVertexIndices(mesh.triangles_uvs_idx_, texture_coordinates_in_use_consolidation, material_triangle_usage);
      single_material_mesh.triangle_uvs_ = GetSingleMaterialMeshVertexAttributes(mesh.triangle_uvs_, texture_coordinates_in_use_consolidation);
    }

    // Add the material.
    single_material_mesh.materials_.push_back(material);
    if (material.gltfExtras.texture_idx.has_value()) {
      assert(*material.gltfExtras.texture_idx < mesh.textures_.size());
      single_material_mesh.materials_.begin()->gltfExtras.texture_idx = 0u;
      single_material_mesh.textures_.push_back(mesh.textures_[*material.gltfExtras.texture_idx]);
    }

    single_material_meshes.push_back(std::move(single_material_mesh));
  }
  return (single_material_meshes);
}

std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh) { return (SeparateMeshByMaterial(mesh, nullptr)); }

std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh, const DuplicateConsolidation &material_consolidation) {
  return (SeparateMeshByMaterial(mesh, &material_consolidation));
}

}  // namespace open3d::geometry

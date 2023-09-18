#include <open3d/geometry/Reorganization.h>

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
  assert(mesh.GetTriangleUvUsage() == TriangleMesh::TriangleUvUsage::indices);
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
  const auto uv_usage = mesh.GetTriangleUvUsage();
  if (!uv_usage.has_value()) {
    assert(mesh.triangle_uvs_.empty());
    return;
  }
  if (*uv_usage == TriangleMesh::TriangleUvUsage::indices && texture_coordinates_consolidation == nullptr) {
    if (mesh.triangles_ == mesh.triangles_uvs_idx_) {
      mesh.triangles_uvs_idx_.clear();
      return;
    }
  }
  assert((texture_coordinates_consolidation != nullptr)
             ? (texture_coordinates_consolidation->original_to_consolidated_indices_.size() == mesh.triangle_uvs_.size())
             : true);
  assert((texture_coordinates_consolidation != nullptr)
             ? (texture_coordinates_consolidation->consolidated_to_original_indices_.size() <= mesh.triangle_uvs_.size())
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
    const auto triangle_uv_indices = mesh.GetTriangleUvIndices(triangle_index, *uv_usage);
    for (auto vertex_in_triangle = 0u; vertex_in_triangle < 3u; ++vertex_in_triangle) {
      auto &vertex = triangle[vertex_in_triangle];
      assert(vertex >= 0);
      assert(vertex < mesh.vertices_.size());
      const auto vertex_uv_indices = triangle_uv_indices[vertex_in_triangle];
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
    }
  }
  vertices.shrink_to_fit();
  mesh.vertices_ = std::move(vertices);
  mesh.triangles_uvs_idx_.clear();
  texture_coordinates.shrink_to_fit();
  mesh.triangle_uvs_ = std::move(texture_coordinates);
}

void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh, const DuplicateConsolidation &texture_coordinates_consolidation) {
  ConsolidateTextureCoordinateIndicesWithVertices(mesh, &texture_coordinates_consolidation);
}

void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh) { ConsolidateTextureCoordinateIndicesWithVertices(mesh, nullptr); }

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

static MaterialsTriangleUsageWithInvalids GetMaterialsTriangleUsageWithUnassigneds(const TriangleMesh &mesh,
                                                                                   const DuplicateConsolidation *consolidation) {
  if (mesh.triangle_material_ids_.empty()) {
    auto unassigned_material_triangle_usage = MaterialTriangleUsage();
    unassigned_material_triangle_usage.reserve(mesh.triangles_.size());
    while (unassigned_material_triangle_usage.size() < mesh.triangles_.size()) {
      unassigned_material_triangle_usage.push_back(unassigned_material_triangle_usage.size());
    }
    return (MaterialsTriangleUsageWithInvalids{MaterialsTriangleUsage(), std::move(unassigned_material_triangle_usage)});
  }
  assert(mesh.triangle_material_ids_.size() == mesh.triangles_.size());
  auto intermediate_materials_triangle_usage = std::vector<std::list<unsigned int>>(
      (consolidation != nullptr) ? consolidation->consolidated_to_original_indices_.size() : mesh.materials_.size());
  auto intermediate_unassigned_material_triangle_usage = std::list<unsigned int>();
  if (consolidation != nullptr) {
    assert(consolidation->original_to_consolidated_indices_.size() == mesh.materials_.size());
    for (auto triangle = 0u; triangle < mesh.triangles_.size(); ++triangle) {
      const auto original_material = mesh.triangle_material_ids_[triangle];
      if (original_material >= 0 && original_material < consolidation->original_to_consolidated_indices_.size()) {
        const auto consolidated_material = consolidation->original_to_consolidated_indices_[original_material];
        assert(consolidated_material < consolidation->consolidated_to_original_indices_.size());
        intermediate_materials_triangle_usage[consolidated_material].push_back(triangle);
      } else {
        intermediate_unassigned_material_triangle_usage.push_back(triangle);
      }
    }
  } else {
    for (auto triangle = 0u; triangle < mesh.triangles_.size(); ++triangle) {
      const auto material = mesh.triangle_material_ids_[triangle];
      if (material >= 0 && material < mesh.materials_.size()) {
        intermediate_materials_triangle_usage[material].push_back(triangle);
      } else {
        intermediate_unassigned_material_triangle_usage.push_back(triangle);
      }
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
  auto unassigned_material_triangle_usage = MaterialTriangleUsage();
  unassigned_material_triangle_usage.reserve(intermediate_unassigned_material_triangle_usage.size());
  for (const auto triangle : intermediate_unassigned_material_triangle_usage) {
    unassigned_material_triangle_usage.push_back(triangle);
  }
  return (MaterialsTriangleUsageWithInvalids{std::move(materials_triangle_usage), std::move(unassigned_material_triangle_usage)});
}

MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh) {
  auto usage = GetMaterialsTriangleUsageWithUnassigneds(mesh, nullptr);
  return (std::move(usage.materials_triangle_usage_));
}

MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh, const DuplicateConsolidation &consolidation) {
  auto usage = GetMaterialsTriangleUsageWithUnassigneds(mesh, &consolidation);
  return (std::move(usage.materials_triangle_usage_));
}

MaterialsTriangleUsageWithInvalids GetMaterialsTriangleUsageWithUnassigneds(const TriangleMesh &mesh) {
  return (GetMaterialsTriangleUsageWithUnassigneds(mesh, nullptr));
}

MaterialsTriangleUsageWithInvalids GetMaterialsTriangleUsageWithUnassigneds(const TriangleMesh &mesh, const DuplicateConsolidation &consolidation) {
  return (GetMaterialsTriangleUsageWithUnassigneds(mesh, &consolidation));
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
  const auto materials_triangle_usage = GetMaterialsTriangleUsageWithUnassigneds(mesh, material_consolidation);
  const auto has_material_consolidation = (material_consolidation != nullptr);
  const auto basic_mesh_count =
      (has_material_consolidation ? material_consolidation->consolidated_to_original_indices_.size() : mesh.materials_.size());
  assert(materials_triangle_usage.materials_triangle_usage_.size() == basic_mesh_count);
  const auto mesh_count = (materials_triangle_usage.unassigned_material_triangle_usage_.empty() ? basic_mesh_count : (basic_mesh_count + 1u));
  auto single_material_meshes = std::vector<TriangleMesh>();
  single_material_meshes.reserve(mesh_count);
  auto create_single_material_mesh = [&](const MaterialTriangleUsage &material_triangle_usage) {
    const auto vertices_in_use_consolidation = ConsolidateOnlyInUseVertices(mesh.triangles_, material_triangle_usage);
    auto single_material_mesh = TriangleMesh();

    // Add the triangles.
    single_material_mesh.triangles_ = GetSingleMaterialMeshVertexIndices(mesh.triangles_, vertices_in_use_consolidation, material_triangle_usage);

    // Add the per triangle material ids.
    single_material_mesh.triangle_material_ids_.resize(single_material_mesh.triangles_.size(), 0);

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

    return (single_material_mesh);
  };

  const auto triangle_uv_usage = mesh.GetTriangleUvUsage();
  const auto *effective_triangles_uv_idx = (const std::vector<Eigen::Vector3i> *)nullptr;
  auto per_triangle_effective_triangles_uv_idx = std::vector<Eigen::Vector3i>();
  if (triangle_uv_usage.has_value()) {
    switch (*triangle_uv_usage) {
      case TriangleMesh::TriangleUvUsage::indices: {
        effective_triangles_uv_idx = &mesh.triangles_uvs_idx_;
        break;
      }
      case TriangleMesh::TriangleUvUsage::per_vertex: {
        effective_triangles_uv_idx = &mesh.triangles_;
        break;
      }
      case TriangleMesh::TriangleUvUsage::per_triangle: {
        per_triangle_effective_triangles_uv_idx.reserve(mesh.triangles_.size());
        while (per_triangle_effective_triangles_uv_idx.size() < mesh.triangles_.size()) {
          const auto base_index = per_triangle_effective_triangles_uv_idx.size() * 3u;
          per_triangle_effective_triangles_uv_idx.push_back(Eigen::Vector3i(base_index, base_index + 1u, base_index + 2u));
        }
        effective_triangles_uv_idx = &per_triangle_effective_triangles_uv_idx;
        break;
      }
    }
  }

  for (auto mesh_index = 0u; mesh_index < basic_mesh_count; ++mesh_index) {
    const auto &material_triangle_usage = materials_triangle_usage.materials_triangle_usage_[mesh_index];
    if (material_triangle_usage.empty()) {
      continue;
    }
    const auto &material =
        mesh.materials_[has_material_consolidation ? material_consolidation->consolidated_to_original_indices_[mesh_index] : mesh_index];
    auto single_material_mesh = create_single_material_mesh(material_triangle_usage);

    // Add the texture coordinates, if needed.
    if (material.IsTextured()) {
      assert(effective_triangles_uv_idx != nullptr);
      const auto texture_coordinates_in_use_consolidation = ConsolidateOnlyInUseVertices(*effective_triangles_uv_idx, material_triangle_usage);
      single_material_mesh.triangles_uvs_idx_ =
          GetSingleMaterialMeshVertexIndices(*effective_triangles_uv_idx, texture_coordinates_in_use_consolidation, material_triangle_usage);
      single_material_mesh.triangle_uvs_ = GetSingleMaterialMeshVertexAttributes(mesh.triangle_uvs_, texture_coordinates_in_use_consolidation);
    }

    // Add the material.
    single_material_mesh.materials_.push_back(material);

    single_material_meshes.push_back(std::move(single_material_mesh));
  }
  if (!materials_triangle_usage.unassigned_material_triangle_usage_.empty()) {
    auto single_material_mesh = create_single_material_mesh(materials_triangle_usage.unassigned_material_triangle_usage_);
    auto default_material = geometry::TriangleMesh::Material();
    default_material.baseColor = geometry::TriangleMesh::Material::MaterialParameter(1.0f, 1.0f, 1.0f);
    single_material_mesh.materials_.push_back(std::move(default_material));
    single_material_meshes.push_back(std::move(single_material_mesh));
  }
  return (single_material_meshes);
}

std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh) { return (SeparateMeshByMaterial(mesh, nullptr)); }

std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh, const DuplicateConsolidation &material_consolidation) {
  return (SeparateMeshByMaterial(mesh, &material_consolidation));
}

static inline bool ShouldMakeEffectiveMaterials(const TriangleMesh &mesh) { return (mesh.materials_.empty() && !mesh.textures_.empty()); }

static std::vector<TriangleMesh::Material> MakeEffectiveMaterials(const std::vector<Image> &textures) {
  auto materials = std::vector<TriangleMesh::Material>();
  materials.reserve(textures.size());
  auto material = geometry::TriangleMesh::Material();
  material.baseColor = geometry::TriangleMesh::Material::MaterialParameter(1.0f, 1.0f, 1.0f);
  while (materials.size() < textures.size()) {
    material.gltfExtras.texture_idx = materials.size();
    materials.push_back(material);
  }
  return (materials);
}

std::vector<TriangleMesh::Material> GetEffectiveMaterials(const TriangleMesh &mesh) {
  if (ShouldMakeEffectiveMaterials(mesh)) {
    return (MakeEffectiveMaterials(mesh.textures_));
  } else {
    return (mesh.materials_);
  }
}

void MakeEffectiveMaterials(TriangleMesh &mesh) {
  if (ShouldMakeEffectiveMaterials(mesh)) {
    mesh.materials_ = MakeEffectiveMaterials(mesh.textures_);
  }
}

bool IsTextureInUse(unsigned int texture, const std::vector<TriangleMesh::Material> &materials) {
  return (std::any_of(materials.begin(), materials.end(), [&](const TriangleMesh::Material &material) {
    return (material.gltfExtras.texture_idx == texture || material.albedo == texture || material.normalMap == texture ||
            material.ambientOcclusion == texture || material.metallic == texture || material.roughness == texture ||
            material.reflectance == texture || material.clearCoat == texture || material.clearCoatRoughness == texture ||
            material.anisotropy == texture || material.gltfExtras.emissiveTexture == texture ||
            std::find(material.gltfExtras.extension_images.begin(), material.gltfExtras.extension_images.end(), texture) !=
                material.gltfExtras.extension_images.end());
  }));
}

void ConvertTriangleUvUsage(TriangleMesh &mesh, TriangleMesh::TriangleUvUsage usage) {
  const auto old_usage = mesh.GetTriangleUvUsage();
  assert(old_usage.has_value());
  if (usage == *old_usage) {
    return;
  }
  switch (usage) {
    case TriangleMesh::TriangleUvUsage::indices: {
      auto uv_indices = std::vector<Eigen::Vector3i>();
      uv_indices.reserve(mesh.triangles_.size());
      while (uv_indices.size() < mesh.triangles_.size()) {
        uv_indices.push_back(mesh.GetTriangleUvIndices(uv_indices.size(), *old_usage));
      }
      mesh.triangles_uvs_idx_ = std::move(uv_indices);
      break;
    }
    case TriangleMesh::TriangleUvUsage::per_vertex: {
      ConsolidateTextureCoordinateIndicesWithVertices(mesh);
      break;
    }
    case TriangleMesh::TriangleUvUsage::per_triangle: {
      auto uvs = std::vector<Eigen::Vector2d>();
      uvs.reserve(mesh.triangles_.size() * 3u);
      for (auto triangle = 0u; triangle < mesh.triangles_.size(); ++triangle) {
        const auto uv_indices = mesh.GetTriangleUvIndices(triangle, *old_usage);
        for (auto vertex_in_triangle = 0u; vertex_in_triangle < 3u; ++vertex_in_triangle) {
          const auto uvs_index = uv_indices[vertex_in_triangle];
          if (uvs_index >= 0) {
            uvs.push_back(mesh.triangle_uvs_[uvs_index]);
          } else {
            uvs.push_back(Eigen::Vector2d(0.0, 0.0));
          }
        }
      }
      assert(uvs.size() == mesh.triangles_.size() * 3u);
      mesh.triangle_uvs_ = std::move(uvs);
      mesh.triangles_uvs_idx_.clear();
      break;
    }
  }
}

MeshProblems RemoveProblematicGeometry(open3d::geometry::TriangleMesh &mesh) {
  if (mesh.triangles_.empty()) {
    throw std::runtime_error("Mesh has no triangles!");
  }
  if (mesh.vertices_.empty()) {
    throw std::runtime_error("Mesh has no vertices!");
  }
  if (mesh.materials_.empty()) {
    throw std::runtime_error("Mesh has no materials!");
  }
  if (mesh.triangle_material_ids_.empty()) {
    throw std::runtime_error("Mesh has no material triangle IDs!");
  }
  if (mesh.triangle_material_ids_.size() != mesh.triangles_.size()) {
    throw std::runtime_error("Mesh material triangle IDs has the wrong size!");
  }

  auto problems = MeshProblems{MeshProblems::Problems(), 0u, 0u};

  struct MaterialSummary {
    bool is_textured;
  };
  auto has_textured_materials = false;
  auto is_material_valid = true;  //! @note Defined here instead of in the loop so that check_texture_reference can be defined outside the loop.
  bool is_material_textured;      //! @note Defined here instead of in the loop so that check_texture_reference can be defined outside the loop.
  auto check_texture_reference = [&](const std::optional<unsigned int> &texture_reference) {
    if (texture_reference.has_value()) {
      is_material_textured = true;
      has_textured_materials = true;
      if (*texture_reference >= mesh.textures_.size()) {
        auto problem_tally = problems.problems_.insert(std::make_pair(MeshProblems::Problem::invalid_texture_index, 0u)).first;
        ++problem_tally->second;
        is_material_valid = false;
      }
    }
  };
  auto material_summaries = std::vector<std::optional<MaterialSummary>>();
  material_summaries.reserve(mesh.materials_.size());
  for (const auto &material : mesh.materials_) {
    is_material_textured = false;

    check_texture_reference(material.albedo);
    check_texture_reference(material.normalMap);
    check_texture_reference(material.ambientOcclusion);
    check_texture_reference(material.metallic);
    check_texture_reference(material.roughness);
    check_texture_reference(material.reflectance);
    check_texture_reference(material.clearCoat);
    check_texture_reference(material.clearCoatRoughness);
    check_texture_reference(material.anisotropy);
    check_texture_reference(material.gltfExtras.texture_idx);
    check_texture_reference(material.gltfExtras.emissiveTexture);

    if (is_material_valid) {
      material_summaries.push_back(MaterialSummary{is_material_textured});
    } else {
      material_summaries.push_back(std::optional<MaterialSummary>());
      ++problems.discarded_materials_;
      is_material_valid = true;
    }
  }
  const auto triangle_uv_usage =
      has_textured_materials ? mesh.GetTriangleUvUsage() : std::optional<open3d::geometry::TriangleMesh::TriangleUvUsage>();

  auto is_triangle_valid = true;
  auto add_triangle_in_mesh_problem = [&](MeshProblems::Problem problem) {
    is_triangle_valid = false;
    auto problem_tally = problems.problems_.insert(std::make_pair(problem, 0u)).first;
    ++problem_tally->second;
  };
  unsigned int triangle_index = 0u;
  while (triangle_index < mesh.triangles_.size()) {
    const auto &triangle = mesh.triangles_[triangle_index];

    auto has_vertex_index_problems = false;
    for (auto vertex_in_triangle = 0u; vertex_in_triangle < 3u; ++vertex_in_triangle) {
      const auto vertex = triangle[vertex_in_triangle];
      if (vertex < 0) {
        add_triangle_in_mesh_problem(MeshProblems::Problem::missing_vertex_index);
        has_vertex_index_problems = true;
      } else if (vertex >= mesh.vertices_.size()) {
        add_triangle_in_mesh_problem(MeshProblems::Problem::invalid_vertex_index);
        has_vertex_index_problems = true;
      }
    }
    if (!has_vertex_index_problems) {
      if (triangle[0u] == triangle[1u] || triangle[0u] == triangle[2u] || triangle[1u] == triangle[2u]) {
        add_triangle_in_mesh_problem(MeshProblems::Problem::topological_degenerate);
      } else if (mesh.vertices_[triangle[0u]] == mesh.vertices_[triangle[1u]] || mesh.vertices_[triangle[0u]] == mesh.vertices_[triangle[2u]] ||
                 mesh.vertices_[triangle[1u]] == mesh.vertices_[triangle[2u]]) {
        add_triangle_in_mesh_problem(MeshProblems::Problem::geometrical_degenerate);
      }
    }
    const auto material_index = mesh.triangle_material_ids_[triangle_index];
    if (material_index < 0) {
      add_triangle_in_mesh_problem(MeshProblems::Problem::missing_material_index);
    } else if (material_index >= mesh.materials_.size()) {
      add_triangle_in_mesh_problem(MeshProblems::Problem::invalid_material_index);
    } else {
      const auto &material_summary = material_summaries[material_index];
      if (material_summary.has_value()) {
        if (material_summary->is_textured) {
          if (triangle_uv_usage == open3d::geometry::TriangleMesh::TriangleUvUsage::indices) {
            const auto &texture_coordinates_indices = mesh.triangles_uvs_idx_[triangle_index];
            for (auto vertex_in_triangle = 0u; vertex_in_triangle < 3u; ++vertex_in_triangle) {
              const auto texture_coordinates_index = texture_coordinates_indices[vertex_in_triangle];
              if (texture_coordinates_index < 0) {
                add_triangle_in_mesh_problem(MeshProblems::Problem::missing_texture_coordinates_index);
              } else if (texture_coordinates_index >= mesh.triangle_uvs_.size()) {
                add_triangle_in_mesh_problem(MeshProblems::Problem::invalid_texture_coordinates_index);
              }
            }
          } else if (!triangle_uv_usage.has_value()) {
            add_triangle_in_mesh_problem(MeshProblems::Problem::bad_triangle_uv_usage);
          }
        }
      } else {
        is_triangle_valid = false;
      }
    }

    if (is_triangle_valid) {
      ++triangle_index;
    } else {
      ++problems.discarded_triangles_;
      const auto last_triangle_index = mesh.triangles_.size() - 1u;
      mesh.triangles_[triangle_index] = mesh.triangles_[last_triangle_index];
      mesh.triangles_.pop_back();
      mesh.triangle_material_ids_[triangle_index] = mesh.triangle_material_ids_[last_triangle_index];
      mesh.triangle_material_ids_.pop_back();
      if (triangle_uv_usage == open3d::geometry::TriangleMesh::TriangleUvUsage::indices) {
        mesh.triangles_uvs_idx_[triangle_index] = mesh.triangles_uvs_idx_[last_triangle_index];
        mesh.triangles_uvs_idx_.pop_back();
      }
      is_triangle_valid = true;
    }
  }

  if (problems.discarded_materials_ > 0u) {
    auto materials = std::vector<open3d::geometry::TriangleMesh::Material>();
    materials.reserve(mesh.materials_.size() - problems.discarded_materials_);
    auto remapped_material_indices = std::vector<unsigned int>();
    remapped_material_indices.reserve(mesh.materials_.size());
    for (auto material_index = 0u; material_index < mesh.materials_.size(); ++material_index) {
      remapped_material_indices.push_back(materials.size());
      if (material_summaries[material_index].has_value()) {
        materials.push_back(std::move(mesh.materials_[material_index]));
      }
    }
    mesh.materials_ = std::move(materials);
    for (auto &triangle_material_index : mesh.triangle_material_ids_) {
      triangle_material_index = remapped_material_indices[triangle_material_index];
    }
  }

  return (problems);
}

bool MeshProblems::DidEncounterProblems() const { return (discarded_triangles_ > 0u || discarded_materials_ > 0u); }

static const auto problem_descriptions = std::unordered_map<MeshProblems::Problem, std::string>{
    std::make_pair(MeshProblems::Problem::missing_vertex_index, "Missing vertex index"),
    std::make_pair(MeshProblems::Problem::invalid_vertex_index, "Invalid vertex index"),
    std::make_pair(MeshProblems::Problem::topological_degenerate, "Topological degenerate triangle"),
    std::make_pair(MeshProblems::Problem::geometrical_degenerate, "Geometrical degenerate triangle"),
    std::make_pair(MeshProblems::Problem::missing_texture_coordinates_index, "Missing texture coordinates index"),
    std::make_pair(MeshProblems::Problem::invalid_texture_coordinates_index, "Invalid texture coordinates index"),
    std::make_pair(MeshProblems::Problem::bad_triangle_uv_usage, "Bad triangle UV usage"),
    std::make_pair(MeshProblems::Problem::missing_material_index, "Missing material index"),
    std::make_pair(MeshProblems::Problem::invalid_material_index, "Invalid material index"),
    std::make_pair(MeshProblems::Problem::invalid_texture_index, "Invalid texture index")};

std::ostream &operator<<(std::ostream &stream, const MeshProblems &problems) {
  if (problems.DidEncounterProblems()) {
    stream << "The following mesh problems were encountered:" << std::endl;
    for (const auto &problem : problems.problems_) {
      stream << problem_descriptions.find(problem.first)->second << ": " << problem.second << " time(s)" << std::endl;
    }
    if (problems.discarded_materials_ > 0u) {
      stream << problems.discarded_materials_ << " material(s) discarded" << std::endl;
    }
    if (problems.discarded_triangles_ > 0u) {
      stream << problems.discarded_triangles_ << " triangle(s) discarded" << std::endl;
    }
  }
  return (stream);
}

}  // namespace open3d::geometry

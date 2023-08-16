#pragma once

#include <open3d/geometry/TriangleMesh.h>

#include <vector>

namespace open3d::geometry {

struct DuplicateConsolidation {
  std::vector<unsigned int> original_to_consolidated_indices_;
  std::vector<unsigned int> consolidated_to_original_indices_;
  bool ShouldConsolidate() const;
};

DuplicateConsolidation GetTextureCoordinatesConsolidation(const TriangleMesh &mesh);
//! @brief Ensures that all elements in mesh.triangle_uvs_ are unique.
void ConsolidateTextureCoordinates(TriangleMesh &mesh);
//! @pre consolidation must have been created by GetTextureCoordinatesConsolidation() on mesh.
//! @brief Ensures that all elements in mesh.triangle_uvs_ are unique.
void ConsolidateTextureCoordinates(TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

//! @brief Reorganizes mesh such that for every element in mesh.triangles_ the corresponding element in mesh.triangle_uvs_idx_ is the same.
void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh, const DuplicateConsolidation &texture_coordinates_consolidation);
//! @brief Reorganizes mesh such that for every element in mesh.triangles_ the corresponding element in mesh.triangle_uvs_idx_ is the same.
void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh);

DuplicateConsolidation GetMaterialConsolidation(const TriangleMesh &mesh);
//! @brief Ensures that all elements in mesh.materials_ are unique.
void ConsolidateMaterials(TriangleMesh &mesh);
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
//! @brief Ensures that all elements in mesh.materials_ are unique.
void ConsolidateMaterials(TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

using MaterialTriangleUsage = std::vector<unsigned int>;  // Indices into TriangleMesh::triangles_ of triangles belonging to a material.
using MaterialsTriangleUsage =
    std::vector<MaterialTriangleUsage>;  // Each element's index correspond to a material index (whether consolidated or original).

//! @returns The triangles belonging to each material.
//! @note Does not apply material consolidation.
MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh);
//! @returns The triangles belonging to each material.
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

struct MaterialsTriangleUsageWithInvalids {
  MaterialsTriangleUsage materials_triangle_usage_;
  MaterialTriangleUsage unassigned_material_triangle_usage_;  // Triangles that do not have a valid material index or no material index at all.
};
//! @returns The triangles belonging to each material.
//! @note Does not apply material consolidation.
MaterialsTriangleUsageWithInvalids GetMaterialsTriangleUsageWithUnassigneds(const TriangleMesh &mesh);
//! @returns The triangles belonging to each material.
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
MaterialsTriangleUsageWithInvalids GetMaterialsTriangleUsageWithUnassigneds(const TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

//! @returns A vector of meshes, each containing just 1 material from the original mesh.
std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh);
//! @returns A vector of meshes, each containing just 1 material from the original mesh.
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh, const DuplicateConsolidation &material_consolidation);

//! @returns On meshes that have no materials but have textures, returns materials referencing these textures. Returns mesh.materials_ otherwise.
std::vector<TriangleMesh::Material> GetEffectiveMaterials(const TriangleMesh &mesh);
bool IsTextureInUse(unsigned int texture, const std::vector<TriangleMesh::Material> &materials);

}  // namespace open3d::geometry

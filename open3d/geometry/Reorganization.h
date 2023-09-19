#pragma once

#include <open3d/geometry/TriangleMesh.h>

#include <vector>
#include <ostream>
#include <string>
#include <unordered_map>

namespace open3d::geometry {

struct DuplicateConsolidation {
  std::vector<unsigned int> original_to_consolidated_indices_;
  std::vector<unsigned int> consolidated_to_original_indices_;
  bool ShouldConsolidate() const;
};

DuplicateConsolidation GetTextureCoordinatesConsolidation(const TriangleMesh &mesh);
//! @brief Ensures that all elements in mesh.triangle_uvs_ are unique.
//! @pre mesh.GetTriangleUvUsage() == TriangleMesh::TriangleUvUsage::indices, if mesh has UVs.
void ConsolidateTextureCoordinates(TriangleMesh &mesh);
//! @pre consolidation must have been created by GetTextureCoordinatesConsolidation() on mesh.
//! @brief Ensures that all elements in mesh.triangle_uvs_ are unique.
//! @pre mesh.GetTriangleUvUsage() == TriangleMesh::TriangleUvUsage::indices, if mesh has UVs.
void ConsolidateTextureCoordinates(TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

//! @brief Reorganizes mesh such that for every element in mesh.vertices_ there is a corresponding element in mesh.triangle_uvs_ and
//! mesh.triangles_uvs_idx_ is cleared altogether.
void ConsolidateTextureCoordinateIndicesWithVertices(TriangleMesh &mesh, const DuplicateConsolidation &texture_coordinates_consolidation);
//! @brief Reorganizes mesh such that for every element in mesh.vertices_ there is a corresponding element in mesh.triangle_uvs_ and
//! mesh.triangles_uvs_idx_ is cleared altogether.
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
//! @note textures_ of the returned meshes isn't populated and the texture indices in the returned materials_ reference the original mesh.textures_.
std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh);
//! @returns A vector of meshes, each containing just 1 material from the original mesh.
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
//! @note textures_ of the returned meshes isn't populated and the texture indices in the returned materials_ reference the original mesh.textures_.
std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh, const DuplicateConsolidation &material_consolidation);

//! @returns On meshes that have no materials but have textures, returns materials referencing these textures. Returns mesh.materials_ otherwise.
std::vector<TriangleMesh::Material> GetEffectiveMaterials(const TriangleMesh &mesh);
//! @brief On meshes that have no materials but have textures, makes materials_ reference these textures. Otherwise, does nothing.
void MakeEffectiveMaterials(TriangleMesh &mesh);
bool IsTextureInUse(unsigned int texture, const std::vector<TriangleMesh::Material> &materials);

//! @pre mesh.GetTriangleUvUsage().has_value()
void ConvertTriangleUvUsage(TriangleMesh &mesh, TriangleMesh::TriangleUvUsage usage);

struct MeshProblems {
  enum class Problem {
    missing_vertex_index,               // A vertex index was negative.
    invalid_vertex_index,               // A vertex index was too large.
    topological_degenerate,             // A triangle where multiple vertices had the same vertex index.
    geometrical_degenerate,             // A triangle where multiple vertices had the same vertex coordinate.
    missing_texture_coordinates_index,  // A triangle that has a textured material had texture coordinate indices that were negative.
    invalid_texture_coordinates_index,  // A texture coordinate index was too large.
    bad_triangle_uv_usage,              // A textured triangle was found where the triangle UV usage couldn't be determined.
    missing_material_index,             // A triangle's material index was negative.
    invalid_material_index,             // A triangle's material index was too large.
    invalid_texture_index               // A texture reference was invalid.
  };
  using Count = unsigned int;
  using Problems = std::unordered_map<Problem, Count>;
  Problems problems_;
  Count discarded_materials_;
  Count discarded_triangles_;
  bool DidEncounterProblems() const;
};

std::ostream &operator<<(std::ostream &stream, const MeshProblems &problems);

//! @brief Removes problematic triangles and materials and returns a tally of how many times each type of problem was encountered.
//! @note Will throw on an empty mesh or if a fundamental problem is encountered such as vectors that should be the same size, but isn't that
//! prevents the rest of the algorithm from executing.
MeshProblems RemoveProblematicGeometry(open3d::geometry::TriangleMesh &mesh);

}  // namespace open3d::geometry

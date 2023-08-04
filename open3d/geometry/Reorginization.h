#pragma once

#include <vector>

namespace open3d::geometry {

class TriangleMesh;

struct DuplicateConsolidation {
  std::vector<unsigned int> original_to_consolidated_indices_;
  std::vector<unsigned int> consolidated_to_original_indices_;
  bool ShouldConsolidate() const;
};

DuplicateConsolidation GetTextureCoordinatesConsolidation(const TriangleMesh &mesh);
void ConsolidateTextureCoordinates(TriangleMesh &mesh);
//! @pre consolidation must have been created by GetTextureCoordinatesConsolidation() on mesh.
void ConsolidateTextureCoordinates(TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

DuplicateConsolidation GetMaterialConsolidation(const TriangleMesh &mesh);
void ConsolidateMaterials(TriangleMesh &mesh);
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
void ConsolidateMaterials(TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

using MaterialTriangleUsage = std::vector<unsigned int>;  // Indices into TriangleMesh::triangles_ of triangles belonging to a material.
using MaterialsTriangleUsage =
    std::vector<MaterialTriangleUsage>;  // Each element's index correspond to a material index (whether consolidated or original).

//! @note Does not apply material consolidation.
MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh);
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
MaterialsTriangleUsage GetMaterialsTriangleUsage(const TriangleMesh &mesh, const DuplicateConsolidation &consolidation);

std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh);
//! @pre consolidation must have been created by GetMaterialConsolidation() on mesh.
std::vector<TriangleMesh> SeparateMeshByMaterial(const TriangleMesh &mesh, const DuplicateConsolidation &material_consolidation);

}  // namespace open3d::geometry

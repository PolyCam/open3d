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

// #define TINYGLTF_USE_RAPIDJSON
#include <tiny_gltf.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <vector>

#include "open3d/geometry/Reorganization.h"
#include "open3d/io/FileFormatIO.h"
#include "open3d/io/ImageIO.h"
#include "open3d/io/TriangleMeshIO.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/FileSystem.h"

#define ARR2IDX(arr) typename std::remove_reference<decltype(arr)>::type::size_type
#define FOREACH(var, arr) for (ARR2IDX(arr) var = 0, var##Size = (arr).size(); var < var##Size; ++var)
#define RFOREACH(var, arr) for (ARR2IDX(arr) var = (arr).size(); var-- > 0;)

namespace open3d {
namespace io {

static constexpr auto specular_glossiness_extension = "KHR_materials_pbrSpecularGlossiness";
static constexpr auto clear_coat_extension = "KHR_materials_clearcoat";
static constexpr auto clear_coat_extension_factor_key = "clearcoatFactor";
static constexpr auto clear_coat_extension_roughness_factor_key = "clearcoatRoughnessFactor";

static std::string GetMimeType(const tinygltf::Image &image) {
  if (!image.mimeType.empty()) {
    return (image.mimeType);
  }
  return (utility::filesystem::GetMimeType(image.uri));
}

static void WriteFileFromBuffer(const std::string &path, const std::vector<uint8_t> &buffer) {
  auto stream = std::ofstream(path, std::ios::out | std::ios::binary);
  stream.write(reinterpret_cast<const std::ofstream::char_type *>(buffer.data()), buffer.size());
  stream.close();
}

static geometry::Image::EncodedData EncodeImage(const geometry::Image &image, const std::string &temporary_file_path,
                                                const std::string temporary_file_mime_type = "image/jpeg") {
  if (image.pass_through_.has_value()) {
    return (std::visit(
        [&](const auto &pass_through) {
          using PassThroughType = typename std::decay<decltype(pass_through)>::type;
          if constexpr (std::is_same<PassThroughType, geometry::Image::EncodedData>::value) {
            return (geometry::Image::EncodedData{pass_through.data_, pass_through.mime_type_});
          } else if constexpr (std::is_same<PassThroughType, std::filesystem::path>::value) {
            return (geometry::Image::EncodedData{ReadFileIntoBuffer(pass_through.string()), utility::filesystem::GetMimeType(pass_through.string())});
          }
        },
        *image.pass_through_));
  } else {
    io::WriteImage(temporary_file_path, image);
    auto buffer = ReadFileIntoBuffer(temporary_file_path);
    utility::filesystem::RemoveFile(temporary_file_path);
    return (geometry::Image::EncodedData{std::move(buffer), temporary_file_mime_type});
  }
}

static void PreparePassThroughImage(geometry::Image &image) {
  // Make a fake 1x1 RGB image just in case somewhere else in Open3D the image integrity is verified.
  image.Prepare(1, 1, 3, 1);
  image.data_ = std::vector<uint8_t>(3, uint8_t(0));
}

static geometry::Image ToOpen3d(const tinygltf::Image &tinygltf_image, TextureLoadMode texture_load_mode,
                                const std::filesystem::path &parent_directory) {
  geometry::Image open3d_image;
  if (texture_load_mode == TextureLoadMode::ignore_external_files && !tinygltf_image.uri.empty() && !tinygltf::IsDataURI(tinygltf_image.uri) &&
      tinygltf_image.image.empty()) {
    open3d_image.pass_through_ = std::filesystem::canonical(parent_directory / std::filesystem::path(tinygltf_image.uri));
    PreparePassThroughImage(open3d_image);
  } else if (tinygltf_image.as_is) {
    open3d_image.pass_through_ = geometry::Image::EncodedData{tinygltf_image.image, GetMimeType(tinygltf_image)};
    PreparePassThroughImage(open3d_image);
  } else {
    assert(tinygltf_image.pixel_type == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE);
    open3d_image.Prepare(tinygltf_image.width, tinygltf_image.height, tinygltf_image.component, tinygltf_image.bits / 8);
    open3d_image.data_ = tinygltf_image.image;
  }
  return (open3d_image);
}

static std::vector<Eigen::Matrix4d> GetNodeTransforms(const tinygltf::Model &model) {
  std::vector<Eigen::Matrix4d> node_transforms(model.nodes.size(), Eigen::Matrix4d::Identity());
  auto is_root = std::vector<bool>(model.nodes.size(), true);
  for (const auto &node : model.nodes) {
    for (const auto &child : node.children) {
      is_root[child] = false;
    }
  }
  auto visited = std::vector<bool>(model.nodes.size(), false);
  std::function<void(size_t, const Eigen::Matrix4d &)> visit = [&](size_t node_idx, const Eigen::Matrix4d &parent_transform) {
    if (visited[node_idx]) {
      throw std::runtime_error("Cycle detected in glTF scene graph.");
    }
    visited[node_idx] = true;
    node_transforms[node_idx] = parent_transform;
    const auto &node = model.nodes[node_idx];
    if (!node.matrix.empty()) {
      std::vector<double> matrix = node.matrix;
      Eigen::Matrix4d transform = Eigen::Map<Eigen::Matrix4d>(&matrix[0], 4, 4);
      node_transforms[node_idx] *= transform;
    } else {
      // The specification states that first the scale is
      // applied to the vertices, then the rotation, and then the
      // translation. Multiply the matrices in the reverse order.
      if (!node.translation.empty()) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.topRightCorner<3, 1>() = Eigen::Vector3d(node.translation[0], node.translation[1], node.translation[2]);
        node_transforms[node_idx] *= transform;
      }
      if (!node.rotation.empty()) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        // glTF represents a quaternion as qx, qy, qz, qw, while
        // Eigen::Quaterniond orders the parameters as qw, qx,
        // qy, qz.
        transform.topLeftCorner<3, 3>() =
            Eigen::Quaterniond(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]).toRotationMatrix();
        node_transforms[node_idx] *= transform;
      }
      if (!node.scale.empty()) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(0, 0) = node.scale[0];
        transform(1, 1) = node.scale[1];
        transform(2, 2) = node.scale[2];
        node_transforms[node_idx] *= transform;
      }
    }
    for (const auto &child : node.children) {
      visit(child, node_transforms[node_idx]);
    }
  };
  for (auto i = 0; i < model.nodes.size(); ++i) {
    if (is_root[i]) {
      visit(i, Eigen::Matrix4d::Identity());
    }
  }
  return (node_transforms);
}

// Adapts an array of bytes to an array of T. Will advance of byte_stride each
// elements.
template <typename T>
struct ArrayAdapter {
  // Pointer to the bytes
  const unsigned char *data_ptr;
  // Number of elements in the array
  const size_t elem_count;
  // Stride in bytes between two elements
  const size_t stride;

  // Construct an array adapter.
  // \param ptr Pointer to the start of the data, with offset applied
  // \param count Number of elements in the array
  // \param byte_stride Stride betweens elements in the array
  ArrayAdapter(const unsigned char *ptr, size_t count, size_t byte_stride) : data_ptr(ptr), elem_count(count), stride(byte_stride) {}

  // Returns a *copy* of a single element. Can't be used to modify it.
  T operator[](size_t pos) const {
    if (pos >= elem_count)
      throw std::out_of_range(
          "Tried to access beyond the last element of an array "
          "adapter with count " +
          std::to_string(elem_count) + " while getting element number " + std::to_string(pos));
    return *(reinterpret_cast<const T *>(data_ptr + pos * stride));
  }
};

// Interface of any adapted array that returns integer data
struct IntArrayBase {
  virtual ~IntArrayBase() = default;
  virtual unsigned int operator[](size_t) const = 0;
  virtual size_t size() const = 0;
};

// An array that loads integer types, and returns them as int
template <class T>
struct IntArray : public IntArrayBase {
  ArrayAdapter<T> adapter;

  IntArray(const ArrayAdapter<T> &a) : adapter(a) {}
  unsigned int operator[](size_t position) const override { return static_cast<unsigned int>(adapter[position]); }

  size_t size() const override { return adapter.elem_count; }
};

bool PassThroughImageData(tinygltf::Image *gltf_image, const int image_idx, std::string *err, std::string *warn, int req_width, int req_height,
                          const unsigned char *bytes, int size, void *user_data) {
  gltf_image->as_is = true;
  gltf_image->image = std::vector<unsigned char>(bytes, bytes + size);
  return true;
}

bool LoadImageData(tinygltf::Image *gltf_image, const int image_idx, std::string *err, std::string *warn, int req_width, int req_height,
                   const unsigned char *bytes, int size, void *user_data) {
  geometry::Image image;
  if (!ReadImageFromMemoryJPG(bytes, size, image) && !ReadImageFromMemoryPNG(bytes, size, image))
    return false;

  gltf_image->width = image.width_;
  gltf_image->height = image.height_;
  gltf_image->component = image.num_of_channels_;
  gltf_image->bits = image.bytes_per_channel_ * 8;
  gltf_image->pixel_type = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
  gltf_image->image.resize(static_cast<size_t>(image.width_ * image.height_ * image.num_of_channels_) * size_t(image.bytes_per_channel_));
  std::copy(image.data_.data(), image.data_.data() + gltf_image->image.size(), gltf_image->image.begin());

  return true;
}

FileGeometry ReadFileGeometryTypeGLTF(const std::string &path) { return FileGeometry(CONTAINS_TRIANGLES | CONTAINS_POINTS); }

static std::optional<Eigen::Vector2d> GetVector2Child(const tinygltf::Value::Object &object, const char *child_name) {
  const auto child = object.find(child_name);
  if (child == object.end()) {
    return (std::optional<Eigen::Vector2d>());
  }
  if (!child->second.IsArray()) {
    return (std::optional<Eigen::Vector2d>());
  }
  if (child->second.ArrayLen() != 2u) {
    return (std::optional<Eigen::Vector2d>());
  }
  if (!child->second.Get(0u).IsNumber() || !child->second.Get(1u).IsNumber()) {
    return (std::optional<Eigen::Vector2d>());
  }
  return (Eigen::Vector2d{child->second.Get(0u).GetNumberAsDouble(), child->second.Get(1u).GetNumberAsDouble()});
}

static std::optional<double> GetDoubleChild(const tinygltf::Value::Object &object, const char *child_name) {
  const auto child = object.find(child_name);
  if (child == object.end()) {
    return (std::optional<double>());
  }
  if (!child->second.IsNumber()) {
    return (std::optional<double>());
  }
  return (child->second.GetNumberAsDouble());
}

static Eigen::Matrix3d GetTextureTransformation(const tinygltf::Value::Object &object) {
  Eigen::Matrix3d transformation = Eigen::Matrix3d::Identity();
  const auto translation = GetVector2Child(object, "offset");
  if (translation.has_value()) {
    transformation.row(2u) << (*translation)(0u), (*translation)(1u), 1.0;
  }
  const auto rotation = GetDoubleChild(object, "rotation");
  if (rotation.has_value()) {
    const auto sine = std::sin(*rotation);
    const auto cosine = std::cos(*rotation);
    Eigen::Matrix3d rotation_transformation;
    rotation_transformation << cosine, sine, 0.0, -sine, cosine, 0.0, 0.0, 0.0, 1.0;
    transformation = transformation * rotation_transformation;
  }
  const auto scale = GetVector2Child(object, "scale");
  if (scale.has_value()) {
    Eigen::Matrix3d scale_transformation;
    scale_transformation << (*scale)(0u), 0.0, 0.0, 0.0, (*scale)(1u), 0.0, 0.0, 0.0, 1.0;
    transformation = transformation * scale_transformation;
  }
  return (transformation);
}

bool ReadTriangleMeshFromGLTFWithOptions(const std::string &filename, geometry::TriangleMesh &mesh, bool print_progress,
                                         TextureLoadMode texture_load_mode) {
  std::string filename_ext = utility::filesystem::GetFileExtensionInLowerCase(filename);
  const bool bBinary(filename_ext == "glb");
  const auto parent_directory = std::filesystem::path(filename).parent_path();

  tinygltf::Model model;
  tinygltf::TinyGLTF loader;
  std::string warn;
  std::string err;
  switch (texture_load_mode) {
    case TextureLoadMode::normal: {
      loader.SetImageLoader(LoadImageData, NULL);
      break;
    }
    case TextureLoadMode::pass_through: {
      loader.SetImageLoader(PassThroughImageData, NULL);
      break;
    }
    case TextureLoadMode::ignore_external_files: {
      loader.SetSkipGltfExternalImageFiles(true);
      //! @note If using external image files, this callback will never be called, so this only resorts to pass through if something isn't right.
      loader.SetImageLoader(PassThroughImageData, NULL);
      break;
    }
  }

  bool ret;
  if (bBinary) {
    ret = loader.LoadBinaryFromFile(&model, &err, &warn, filename.c_str());
  } else {
    ret = loader.LoadASCIIFromFile(&model, &err, &warn, filename.c_str());
  }

  if (!warn.empty() || !err.empty()) {
    utility::LogWarning("Read GLTF failed: unable to open file {}", filename);
  }
  if (!ret) {
    return false;
  }

  if (model.meshes.size() > 1) {
    utility::LogInfo(
        "The file contains more than one mesh. All meshes will be "
        "loaded as a single mesh.");
  }

  const auto node_transforms = GetNodeTransforms(model);

  std::vector<geometry::Image> textures;
  textures.reserve(model.images.size());
  while (textures.size() < model.images.size()) {
    textures.emplace_back(ToOpen3d(model.images[textures.size()], texture_load_mode, parent_directory));
  }
  auto reference_texture_if_needed = [&model](std::optional<unsigned int> &open3d_texture, std::optional<unsigned int> &texture_coordinates_index,
                                              std::optional<Eigen::Matrix3d> &texture_transformation, const auto &tiny_gltf_texture) {
    if (tiny_gltf_texture.index >= 0 && tiny_gltf_texture.index < model.textures.size()) {
      const auto source = model.textures[tiny_gltf_texture.index].source;
      if (source >= 0 && source < model.images.size() && tiny_gltf_texture.texCoord >= 0) {
        auto specific_texture_transformation = std::optional<Eigen::Matrix3d>();
        const auto texture_transformation_extension = tiny_gltf_texture.extensions.find("KHR_texture_transform");
        if (texture_transformation_extension != tiny_gltf_texture.extensions.end()) {
          if (texture_transformation_extension->second.IsObject()) {
            specific_texture_transformation =
                GetTextureTransformation(texture_transformation_extension->second.template Get<tinygltf::Value::Object>());
          }
        }
        if (texture_transformation.has_value()) {
          if (specific_texture_transformation != texture_transformation) {
            return;
          }
        } else if (specific_texture_transformation.has_value()) {
          texture_transformation = specific_texture_transformation;
        }
      }
      if (texture_coordinates_index.has_value()) {
        if (texture_coordinates_index != (unsigned int)tiny_gltf_texture.texCoord) {
          return;
        }
      } else {
        texture_coordinates_index = (unsigned int)tiny_gltf_texture.texCoord;
      }
      open3d_texture = (unsigned int)source;
    }
  };

  struct MaterialWithAncillaries {
    geometry::TriangleMesh::Material material_;
    std::optional<unsigned int> texture_coordinates_index_;
    std::optional<Eigen::Matrix3d> texture_transformation_;
  };
  const auto read_material = [&](const tinygltf::Primitive &primitive) {
    if (primitive.material < 0 || primitive.material >= model.materials.size()) {
      auto default_material = geometry::TriangleMesh::Material();
      default_material.baseColor = geometry::TriangleMesh::Material::MaterialParameter(1.0f, 1.0f, 1.0f);
      return MaterialWithAncillaries{default_material, std::optional<unsigned int>()};
    }

    auto material = geometry::TriangleMesh::Material();
    const tinygltf::Material &gltf_material = model.materials[primitive.material];
    const auto &color = gltf_material.pbrMetallicRoughness.baseColorFactor;
    if (color.size() == 4u) {
      material.baseColor = geometry::TriangleMesh::Material::MaterialParameter(color[0], color[1], color[2], color[3]);
    }
    const auto &emissiveFactor = gltf_material.emissiveFactor;
    if (emissiveFactor.size() == 3u) {
      material.gltfExtras.emissiveFactor = Eigen::Vector3d(emissiveFactor[0], emissiveFactor[1], emissiveFactor[2]);
    }
    material.gltfExtras.doubleSided = gltf_material.doubleSided;
    material.gltfExtras.alphaMode = gltf_material.alphaMode;
    material.gltfExtras.alphaCutoff = gltf_material.alphaCutoff;
    material.baseMetallic = gltf_material.pbrMetallicRoughness.metallicFactor;
    material.baseRoughness = gltf_material.pbrMetallicRoughness.roughnessFactor;
    auto base_texture_info = tinygltf::TextureInfo();
    if (gltf_material.pbrMetallicRoughness.baseColorTexture.index >= 0) {
      base_texture_info = gltf_material.pbrMetallicRoughness.baseColorTexture;
    } else if (const auto it = gltf_material.extensions.find(specular_glossiness_extension); it != gltf_material.extensions.end()) {
      const auto &specularGlossiness = it->second;
      // Treat the diffuse texture as the base color texture.
      if (specularGlossiness.Has("diffuseTexture")) {
        base_texture_info.index = specularGlossiness.Get("diffuseTexture").Get("index").Get<int>();
        base_texture_info.texCoord = specularGlossiness.Get("diffuseTexture").Get("texCoord").Get<int>();
        material.gltfExtras.texture_from_specular_glossiness_diffuse = true;
      }
    }
    auto texture_coordinates_index = std::optional<unsigned int>();
    auto texture_transformation = std::optional<Eigen::Matrix3d>();
    reference_texture_if_needed(material.gltfExtras.texture_idx, texture_coordinates_index, texture_transformation, base_texture_info);
    reference_texture_if_needed(material.normalMap, texture_coordinates_index, texture_transformation, gltf_material.normalTexture);
    reference_texture_if_needed(material.ambientOcclusion, texture_coordinates_index, texture_transformation, gltf_material.occlusionTexture);
    reference_texture_if_needed(material.roughness, texture_coordinates_index, texture_transformation,
                                gltf_material.pbrMetallicRoughness.metallicRoughnessTexture);
    reference_texture_if_needed(material.gltfExtras.emissiveTexture, texture_coordinates_index, texture_transformation,
                                gltf_material.emissiveTexture);

    // Read any images referenced by extensions.
    material.gltfExtras.extensions = gltf_material.extensions;
    auto has_clear_coat = false;
    for (auto &[extension_name, extension] : material.gltfExtras.extensions) {
      if (extension.IsObject()) {
        for (auto &[key, value] : extension.Get<tinygltf::Value::Object>()) {
          if (material.gltfExtras.texture_from_specular_glossiness_diffuse && extension_name == specular_glossiness_extension &&
              key == "diffuseTexture") {
            // Skip diffuse texture if it's already been read as the base color texture.
            continue;
          } else if (extension_name == clear_coat_extension) {
            has_clear_coat = true;
            for (auto &[subkey, subvalue] : extension.Get<tinygltf::Value::Object>()) {
              if (subkey == clear_coat_extension_factor_key) {
                if (subvalue.IsNumber()) {
                  material.baseClearCoat = (float)subvalue.GetNumberAsDouble();
                }
              } else if (subkey == clear_coat_extension_roughness_factor_key) {
                if (subvalue.IsNumber()) {
                  material.baseClearCoatRoughness = (float)subvalue.GetNumberAsDouble();
                }
              }
            }
          }
          // Assume anything that has an "index" is a texture. This is true for all the material extensions in
          // https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos.
          if (value.Has("index")) {
            auto &index_value = value.Get<tinygltf::Value::Object>()["index"];
            auto extra_texture_info = tinygltf::TextureInfo();
            extra_texture_info.index = index_value.GetNumberAsInt();
            auto open3d_texture = std::optional<unsigned int>();
            reference_texture_if_needed(open3d_texture, texture_coordinates_index, texture_transformation, extra_texture_info);
            if (open3d_texture.has_value()) {
              index_value = tinygltf::Value((int)material.gltfExtras.extension_images.size());
              material.gltfExtras.extension_images.push_back(*open3d_texture);
            }
          }
        }
      }
    }
    if (has_clear_coat) {
      material.gltfExtras.extensions.erase(clear_coat_extension);
    }

    return MaterialWithAncillaries{std::move(material), texture_coordinates_index, texture_transformation};
  };

  std::vector<MaterialWithAncillaries> materials;
  std::unordered_map<int, size_t> gltf_material_to_id;

  mesh.Clear();
  geometry::TriangleMesh mesh_temp;
  for (const tinygltf::Node &gltf_node : model.nodes) {
    if (gltf_node.mesh != -1) {
      const tinygltf::Mesh &gltf_mesh = model.meshes[gltf_node.mesh];

      for (const tinygltf::Primitive &primitive : gltf_mesh.primitives) {
        mesh_temp.Clear();
        size_t material_id;
        if (auto it = gltf_material_to_id.find(primitive.material); it != gltf_material_to_id.end()) {
          material_id = it->second;
        } else {
          material_id = materials.size();
          materials.emplace_back(read_material(primitive));
          gltf_material_to_id[primitive.material] = material_id;
        }
        const auto &material_with_ancillaries = materials[material_id];
        const auto texture_coordinates_attribute =
            material_with_ancillaries.texture_coordinates_index_.has_value()
                ? std::make_optional("TEXCOORD_" + std::to_string(*material_with_ancillaries.texture_coordinates_index_))
                : std::optional<std::string>();
        for (const auto &attribute : primitive.attributes) {
          if (attribute.first == "POSITION") {
            tinygltf::Accessor &positions_accessor = model.accessors[attribute.second];
            tinygltf::BufferView &positions_view = model.bufferViews[positions_accessor.bufferView];
            const tinygltf::Buffer &positions_buffer = model.buffers[positions_view.buffer];
            const float *positions =
                reinterpret_cast<const float *>(&positions_buffer.data[positions_view.byteOffset + positions_accessor.byteOffset]);

            for (size_t i = 0; i < positions_accessor.count; ++i) {
              mesh_temp.vertices_.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
            }
          }

          if (attribute.first == "NORMAL") {
            tinygltf::Accessor &normals_accessor = model.accessors[attribute.second];
            tinygltf::BufferView &normals_view = model.bufferViews[normals_accessor.bufferView];
            const tinygltf::Buffer &normals_buffer = model.buffers[normals_view.buffer];
            const float *normals = reinterpret_cast<const float *>(&normals_buffer.data[normals_view.byteOffset + normals_accessor.byteOffset]);

            for (size_t i = 0; i < normals_accessor.count; ++i) {
              mesh_temp.vertex_normals_.emplace_back(normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]);
            }
          }

          if (attribute.first == "COLOR_0") {
            tinygltf::Accessor &colors_accessor = model.accessors[attribute.second];
            tinygltf::BufferView &colors_view = model.bufferViews[colors_accessor.bufferView];
            const tinygltf::Buffer &colors_buffer = model.buffers[colors_view.buffer];

            size_t byte_stride = colors_view.byteStride;
            if (byte_stride == 0) {
              // According to glTF 2.0 specs:
              // When byteStride==0, it means that accessor
              // elements are tightly packed.
              byte_stride = colors_accessor.type * tinygltf::GetComponentSizeInBytes(colors_accessor.componentType);
            }
            switch (colors_accessor.componentType) {
              case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                for (size_t i = 0; i < colors_accessor.count; ++i) {
                  const float *colors = reinterpret_cast<const float *>(colors_buffer.data.data() + colors_view.byteOffset + i * byte_stride);
                  mesh_temp.vertex_colors_.emplace_back(colors[0], colors[1], colors[2]);
                }
                break;
              }
              case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                double max_val = (double)std::numeric_limits<uint8_t>::max();
                for (size_t i = 0; i < colors_accessor.count; ++i) {
                  const uint8_t *colors = reinterpret_cast<const uint8_t *>(colors_buffer.data.data() + colors_view.byteOffset + i * byte_stride);
                  mesh_temp.vertex_colors_.emplace_back(colors[0] / max_val, colors[1] / max_val, colors[2] / max_val);
                }
                break;
              }
              case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                double max_val = (double)std::numeric_limits<uint16_t>::max();
                for (size_t i = 0; i < colors_accessor.count; ++i) {
                  const uint16_t *colors = reinterpret_cast<const uint16_t *>(colors_buffer.data.data() + colors_view.byteOffset + i * byte_stride);
                  mesh_temp.vertex_colors_.emplace_back(colors[0] / max_val, colors[1] / max_val, colors[2] / max_val);
                }
                break;
              }
              default: {
                utility::LogWarning(
                    "Unrecognized component type for "
                    "vertex colors");
                break;
              }
            }
          }

          if (attribute.first == texture_coordinates_attribute) {
            tinygltf::Accessor &positions_accessor = model.accessors[attribute.second];
            tinygltf::BufferView &positions_view = model.bufferViews[positions_accessor.bufferView];
            const tinygltf::Buffer &positions_buffer = model.buffers[positions_view.buffer];
            const float *positions =
                reinterpret_cast<const float *>(&positions_buffer.data[positions_view.byteOffset + positions_accessor.byteOffset]);

            for (size_t i = 0; i < positions_accessor.count; ++i) {
              mesh_temp.triangle_uvs_.emplace_back(positions[i * 2 + 0], positions[i * 2 + 1]);
            }
            if (material_with_ancillaries.texture_transformation_.has_value()) {
              for (auto &uv : mesh_temp.triangle_uvs_) {
                const auto result_uv = *material_with_ancillaries.texture_transformation_ * Eigen::Vector3d(uv(0u), uv(1u), 1.0);
                uv = Eigen::Vector2d(result_uv(0u), result_uv(1u));
              }
            }
          }
        }

        // Load triangles
        std::unique_ptr<IntArrayBase> indices_array_pointer = nullptr;
        {
          const tinygltf::Accessor &indices_accessor = model.accessors[primitive.indices];
          const tinygltf::BufferView &indices_view = model.bufferViews[indices_accessor.bufferView];
          const tinygltf::Buffer &indices_buffer = model.buffers[indices_view.buffer];
          const auto data_address = indices_buffer.data.data() + indices_view.byteOffset + indices_accessor.byteOffset;
          const auto byte_stride = indices_accessor.ByteStride(indices_view);
          const auto count = indices_accessor.count;

          // Allocate the index array in the pointer-to-base
          // declared in the parent scope
          switch (indices_accessor.componentType) {
            case TINYGLTF_COMPONENT_TYPE_BYTE:
              indices_array_pointer = std::unique_ptr<IntArray<char>>(new IntArray<char>(ArrayAdapter<char>(data_address, count, byte_stride)));
              break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
              indices_array_pointer = std::unique_ptr<IntArray<unsigned char>>(
                  new IntArray<unsigned char>(ArrayAdapter<unsigned char>(data_address, count, byte_stride)));
              break;
            case TINYGLTF_COMPONENT_TYPE_SHORT:
              indices_array_pointer = std::unique_ptr<IntArray<short>>(new IntArray<short>(ArrayAdapter<short>(data_address, count, byte_stride)));
              break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
              indices_array_pointer = std::unique_ptr<IntArray<unsigned short>>(
                  new IntArray<unsigned short>(ArrayAdapter<unsigned short>(data_address, count, byte_stride)));
              break;
            case TINYGLTF_COMPONENT_TYPE_INT:
              indices_array_pointer = std::unique_ptr<IntArray<int>>(new IntArray<int>(ArrayAdapter<int>(data_address, count, byte_stride)));
              break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
              indices_array_pointer =
                  std::unique_ptr<IntArray<unsigned int>>(new IntArray<unsigned int>(ArrayAdapter<unsigned int>(data_address, count, byte_stride)));
              break;
            default:
              break;
          }
          const auto &indices = *indices_array_pointer;

          switch (primitive.mode) {
            case TINYGLTF_MODE_TRIANGLES:
              for (size_t i = 0; i < indices_accessor.count; i += 3) {
                mesh_temp.triangles_.push_back(Eigen::Vector3i(indices[i], indices[i + 1], indices[i + 2]));
              }
              break;
            case TINYGLTF_MODE_TRIANGLE_STRIP:
              for (size_t i = 2; i < indices_accessor.count; ++i) {
                mesh_temp.triangles_.push_back(Eigen::Vector3i(indices[i - 2], indices[i - 1], indices[i]));
              }
              break;
            case TINYGLTF_MODE_TRIANGLE_FAN:
              for (size_t i = 2; i < indices_accessor.count; ++i) {
                mesh_temp.triangles_.push_back(Eigen::Vector3i(indices[0], indices[i - 1], indices[i]));
              }
              break;
          }
          mesh_temp.triangles_uvs_idx_ = std::vector<Eigen::Vector3i>(mesh_temp.triangles_.size(), Eigen::Vector3i::Constant(-1));
        }

        // handle invalid meshes with a texture but no texture coordinates by skipping texturing altogether
        if (materials[material_id].material_.IsTextured() &&
            (texture_coordinates_attribute.has_value() ? primitive.attributes.find(*texture_coordinates_attribute) == primitive.attributes.end()
                                                       : true)) {
          materials[material_id].material_.RemoveTextures();
        }

        // read textures
        mesh_temp.triangle_material_ids_.resize(mesh_temp.triangles_.size(), (int)material_id);
        if (materials[material_id].material_.IsTextured()) {
          std::vector<Eigen::Vector2d> triangle_uvs_;
          FOREACH(i, mesh_temp.triangles_) {
            const Eigen::Vector3i &face = mesh_temp.triangles_[i];
            for (int v = 0; v < 3; ++v) {
              mesh_temp.triangles_uvs_idx_[i](v) = triangle_uvs_.size();
              triangle_uvs_.emplace_back(mesh_temp.triangle_uvs_[face[v]]);
            }
          }
          mesh_temp.triangle_uvs_ = std::move(triangle_uvs_);
        } else {
          mesh_temp.triangle_uvs_.clear();
          mesh_temp.triangles_uvs_idx_.clear();
        }

        // apply node transforms
        const auto node_idx = &gltf_node - &model.nodes[0];
        if (node_transforms[node_idx] != Eigen::Matrix4d::Identity()) {
          mesh_temp.Transform(node_transforms[node_idx]);
        }

        mesh.Add(mesh_temp, false);
      }
    }
  }
  mesh.materials_.clear();
  mesh.materials_.reserve(materials.size());
  for (auto &material : materials) {
    mesh.materials_.push_back(std::move(material.material_));
  }
  mesh.textures_ = std::move(textures);

  return true;
}

bool ReadTriangleMeshFromGLTF(const std::string &filename, geometry::TriangleMesh &mesh, bool print_progress) {
  return ReadTriangleMeshFromGLTFWithOptions(filename, mesh, print_progress, TextureLoadMode::normal);
}

bool ReadTriangleMeshFromGLTFWithTexturePassThrough(const std::string &filename, geometry::TriangleMesh &mesh, bool print_progress) {
  return ReadTriangleMeshFromGLTFWithOptions(filename, mesh, print_progress, TextureLoadMode::pass_through);
}

bool ReadTriangleMeshFromGLTFWithIgnoringExternalTextures(const std::string &filename, geometry::TriangleMesh &mesh, bool print_progress) {
  return ReadTriangleMeshFromGLTFWithOptions(filename, mesh, print_progress, TextureLoadMode::ignore_external_files);
}

template <typename T>
void extendBuffer(const std::vector<T> &src, tinygltf::Buffer &dst, size_t &byte_offset, size_t &byte_length) {
  byte_offset = dst.data.size();
  byte_length = sizeof(T) * src.size();
  byte_length = ((byte_length + 3) / 4) * 4;
  dst.data.resize(byte_offset + byte_length);
  memcpy(&dst.data[byte_offset], &src[0], byte_length);
}

template <typename TOUT, typename TIN>
void extendBufferConvert(const std::vector<TIN> &src, tinygltf::Buffer &dst, size_t &byte_offset, size_t &byte_length) {
  byte_offset = dst.data.size();
  byte_length = sizeof(TOUT) * src.size();
  byte_length = ((byte_length + 3) / 4) * 4;
  dst.data.resize(byte_offset + byte_length);
  TOUT *const ptr_dst = reinterpret_cast<TOUT *>(dst.data.data() + byte_offset);
  FOREACH(i, src)
  ptr_dst[i] = src[i].template cast<typename TOUT::Scalar>();
}

static void InitializeGltfMaterial(tinygltf::Material &material, const geometry::TriangleMesh &mesh) {
  material.pbrMetallicRoughness.metallicFactor = 0.0;
  material.pbrMetallicRoughness.roughnessFactor = 1.0;
  if (mesh.materials_.size() == 1u) {
    const auto &open3d_material = mesh.materials_.front();
    const auto &color = open3d_material.baseColor;
    if (color != geometry::TriangleMesh::Material::MaterialParameter()) {
      material.pbrMetallicRoughness.baseColorFactor = {color.f4[0], color.f4[1], color.f4[2], color.f4[3]};
    }
    material.doubleSided = open3d_material.gltfExtras.doubleSided;
    material.alphaMode = open3d_material.gltfExtras.alphaMode;
    material.alphaCutoff = open3d_material.gltfExtras.alphaCutoff;
    if (open3d_material.gltfExtras.emissiveFactor.has_value()) {
      const auto &emissiveFactor = *open3d_material.gltfExtras.emissiveFactor;
      material.emissiveFactor = {emissiveFactor[0], emissiveFactor[1], emissiveFactor[2]};
    }
    material.pbrMetallicRoughness.metallicFactor = open3d_material.baseMetallic;
    material.pbrMetallicRoughness.roughnessFactor = open3d_material.baseRoughness;
  }
}

static std::optional<tinygltf::Image> TrySkippedExternalTexture(const geometry::Image &image, const std::filesystem::path &parent_directory) {
  if (image.pass_through_.has_value()) {
    const auto *absolute_path = std::get_if<std::filesystem::path>(&*image.pass_through_);
    if (absolute_path != nullptr) {
      const auto relative_path = std::filesystem::relative(*absolute_path, parent_directory);
      tinygltf::Image gltf_image;
      gltf_image.uri = relative_path.string();
      gltf_image.mimeType = utility::filesystem::GetMimeType(absolute_path->string());
      gltf_image.name = absolute_path->stem().string();
      return (gltf_image);
    }
  }
  return (std::optional<tinygltf::Image>());
}

// export the mesh as a GLTF file
bool SaveMeshGLTF(const std::string &fileName, const geometry::TriangleMesh &_mesh) {
  const std::string path = utility::filesystem::GetFileParentDirectory(fileName);
  if (!path.empty())
    utility::filesystem::MakeDirectoryHierarchy(path);
  std::string filename_ext = utility::filesystem::GetFileExtensionInLowerCase(fileName);
  const bool bBinary(filename_ext == "glb");

  const auto material_consolidation = GetMaterialConsolidation(_mesh);
  auto meshes = SeparateMeshByMaterial(_mesh, material_consolidation);
  for (auto &mesh : meshes) {
    assert(mesh.materials_.size() == 1u);
    if (mesh.materials_.front().IsTextured() && mesh.HasTriangleUvIndices()) {
      const auto texture_coordinates_consolidation = GetTextureCoordinatesConsolidation(mesh);
      ConsolidateTextureCoordinateIndicesWithVertices(mesh, texture_coordinates_consolidation);
    }
  }

  // create GLTF model
  tinygltf::Model gltfModel;
  tinygltf::Scene gltfScene;
  tinygltf::Mesh gltfMesh;
  tinygltf::Buffer gltfBuffer;

  if (_mesh.HasTextures()) {
    const auto file_path = std::filesystem::path(fileName);
    const auto parent_directory = file_path.parent_path();
    const auto assets_relative_directory = std::filesystem::path("assets");
    auto created_assets_directory = false;
    const auto texture_base_name = file_path.stem().string();
    gltfModel.images.reserve(_mesh.textures_.size());
    gltfModel.textures.reserve(_mesh.textures_.size());
    for (auto texture_index = 0u; texture_index < _mesh.textures_.size(); ++texture_index) {
      const auto &image = _mesh.textures_[texture_index];
      if (bBinary) {
        const auto encoded_data = EncodeImage(image, path + "Temp.jpg");
        tinygltf::Image gltf_image;
        gltf_image.mimeType = encoded_data.mime_type_;
        gltf_image.bufferView = gltfModel.bufferViews.size();
        gltf_image.as_is = true;
        tinygltf::BufferView imageBufferView;
        imageBufferView.buffer = gltfModel.buffers.size();
        extendBuffer(encoded_data.data_, gltfBuffer, imageBufferView.byteOffset, imageBufferView.byteLength);
        gltfModel.bufferViews.emplace_back(std::move(imageBufferView));
        gltfModel.images.emplace_back(std::move(gltf_image));
      } else {
        auto skipped_external_texture = TrySkippedExternalTexture(image, parent_directory);
        if (skipped_external_texture.has_value()) {
          gltfModel.images.push_back(std::move(*skipped_external_texture));
        } else {
          if (!created_assets_directory) {
            const auto assets_directory = parent_directory / assets_relative_directory;
            std::filesystem::create_directories(assets_directory);
            created_assets_directory = true;
          }
          const auto texture_name = texture_base_name + '_' + std::to_string(texture_index);
          tinygltf::Image gltf_image;
          gltf_image.name = texture_name;
          const auto *encoded_data = image.pass_through_.has_value() ? std::get_if<geometry::Image::EncodedData>(&*image.pass_through_)
                                                                     : (const geometry::Image::EncodedData *)nullptr;
          const auto mime_type = ((encoded_data != nullptr) ? encoded_data->mime_type_.c_str() : "image/jpeg");
          gltf_image.mimeType = mime_type;
          const auto relative_texture_file =
              assets_relative_directory / std::filesystem::path(texture_name + '.' + utility::filesystem::GetExtension(mime_type));
          const auto texture_file = parent_directory / relative_texture_file;
          gltf_image.uri = relative_texture_file.string();
          std::filesystem::remove(texture_file);
          if (encoded_data != nullptr) {
            WriteFileFromBuffer(texture_file.string(), encoded_data->data_);
          } else {
            io::WriteImage(texture_file.string(), image);
          }
          gltfModel.images.emplace_back(std::move(gltf_image));
        }
      }
      tinygltf::Texture gltf_texture;
      gltf_texture.source = gltfModel.textures.size();
      gltfModel.textures.emplace_back(std::move(gltf_texture));
    }
  }

  std::unordered_set<std::string> extensions_used;

  for (const geometry::TriangleMesh &mesh : meshes) {
    assert(mesh.materials_.size() == 1u);

    tinygltf::Primitive gltfPrimitive;

    // setup vertices
    {
      static_assert(3 * sizeof(double) == sizeof(Eigen::Vector3d), "vertices_ should be continuous");
      Eigen::AlignedBox3d bb;
      for (const Eigen::Vector3d &v : mesh.vertices_)
        bb.extend(v);
      gltfPrimitive.attributes["POSITION"] = gltfModel.accessors.size();
      tinygltf::Accessor vertexPositionAccessor;
      vertexPositionAccessor.bufferView = gltfModel.bufferViews.size();
      vertexPositionAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      vertexPositionAccessor.count = mesh.vertices_.size();
      vertexPositionAccessor.type = TINYGLTF_TYPE_VEC3;
      vertexPositionAccessor.minValues = {bb.min().x(), bb.min().y(), bb.min().z()};
      vertexPositionAccessor.maxValues = {bb.max().x(), bb.max().y(), bb.max().z()};
      gltfModel.accessors.emplace_back(std::move(vertexPositionAccessor));
      // setup vertices buffer
      tinygltf::BufferView vertexPositionBufferView;
      vertexPositionBufferView.buffer = gltfModel.buffers.size();
      extendBufferConvert<Eigen::Vector3f>(mesh.vertices_, gltfBuffer, vertexPositionBufferView.byteOffset, vertexPositionBufferView.byteLength);
      gltfModel.bufferViews.emplace_back(std::move(vertexPositionBufferView));
    }

    // setup vertex normals
    if (mesh.HasVertexNormals()) {
      static_assert(3 * sizeof(double) == sizeof(Eigen::Vector3d), "vertex_normals_ should be continuous");
      gltfPrimitive.attributes["NORMAL"] = gltfModel.accessors.size();
      tinygltf::Accessor vertexNormalAccessor;
      vertexNormalAccessor.bufferView = gltfModel.bufferViews.size();
      vertexNormalAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      vertexNormalAccessor.count = mesh.vertex_normals_.size();
      vertexNormalAccessor.type = TINYGLTF_TYPE_VEC3;
      gltfModel.accessors.emplace_back(std::move(vertexNormalAccessor));
      // setup vertex normals buffer
      tinygltf::BufferView vertexNormalBufferView;
      vertexNormalBufferView.buffer = gltfModel.buffers.size();
      extendBufferConvert<Eigen::Vector3f>(mesh.vertex_normals_, gltfBuffer, vertexNormalBufferView.byteOffset, vertexNormalBufferView.byteLength);
      gltfModel.bufferViews.emplace_back(std::move(vertexNormalBufferView));
    }

    // setup faces
    {
      static_assert(3 * sizeof(unsigned) == sizeof(Eigen::Vector3i), "triangles_ should be continuous");
      gltfPrimitive.indices = gltfModel.accessors.size();
      tinygltf::Accessor triangleAccessor;
      triangleAccessor.bufferView = gltfModel.bufferViews.size();
      triangleAccessor.type = TINYGLTF_TYPE_SCALAR;
      triangleAccessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
      triangleAccessor.count = mesh.triangles_.size() * 3;
      gltfModel.accessors.emplace_back(std::move(triangleAccessor));
      // setup triangles buffer
      tinygltf::BufferView triangleBufferView;
      triangleBufferView.buffer = gltfModel.buffers.size();
      extendBuffer(mesh.triangles_, gltfBuffer, triangleBufferView.byteOffset, triangleBufferView.byteLength);
      gltfModel.bufferViews.emplace_back(std::move(triangleBufferView));
      gltfPrimitive.mode = TINYGLTF_MODE_TRIANGLES;
    }

    // setup material
    const geometry::TriangleMesh::Material &material = mesh.materials_.front();
    tinygltf::Material gltfMaterial;
    InitializeGltfMaterial(gltfMaterial, mesh);
    gltfPrimitive.material = gltfModel.materials.size();
    gltfMaterial.extensions = material.gltfExtras.extensions;
    if (material.HasBaseClearCoat()) {
      auto object = tinygltf::Value::Object();
      object.insert(std::make_pair(clear_coat_extension_factor_key, tinygltf::Value((double)material.baseClearCoat)));
      object.insert(std::make_pair(clear_coat_extension_roughness_factor_key, tinygltf::Value((double)material.baseClearCoatRoughness)));
      gltfMaterial.extensions.insert(std::make_pair(clear_coat_extension, tinygltf::Value(std::move(object))));
    }
    for (const auto &[extension_name, extension] : gltfMaterial.extensions) {
      extensions_used.insert(extension_name);
    }
    if (material.IsTextured()) {
      // setup texture coordinates accessor
      gltfPrimitive.attributes["TEXCOORD_0"] = gltfModel.accessors.size();
      tinygltf::Accessor vertexTexcoordAccessor;
      vertexTexcoordAccessor.bufferView = gltfModel.bufferViews.size();
      vertexTexcoordAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      vertexTexcoordAccessor.count = mesh.triangle_uvs_.size();
      vertexTexcoordAccessor.type = TINYGLTF_TYPE_VEC2;
      gltfModel.accessors.emplace_back(std::move(vertexTexcoordAccessor));
      // setup texture coordinates
      static_assert(2 * sizeof(double) == sizeof(Eigen::Vector2d), "triangle_uvs_ should be continuous");
      assert(mesh.vertices_.size() == mesh.triangle_uvs_.size());
      tinygltf::BufferView vertexTexcoordBufferView;
      vertexTexcoordBufferView.buffer = gltfModel.buffers.size();
      extendBufferConvert<Eigen::Vector2f>(mesh.triangle_uvs_, gltfBuffer, vertexTexcoordBufferView.byteOffset, vertexTexcoordBufferView.byteLength);
      gltfModel.bufferViews.emplace_back(std::move(vertexTexcoordBufferView));
      // setup material
      auto setup_texture_if_needed = [&](const std::optional<unsigned int> &texture_index, auto &texture_info) {
        if (texture_index.has_value()) {
          texture_info.index = *texture_index;
          texture_info.texCoord = 0;
        }
      };

      setup_texture_if_needed(material.gltfExtras.texture_idx.has_value() ? material.gltfExtras.texture_idx : material.albedo,
                              gltfMaterial.pbrMetallicRoughness.baseColorTexture);
      setup_texture_if_needed(material.normalMap, gltfMaterial.normalTexture);
      setup_texture_if_needed(material.ambientOcclusion, gltfMaterial.occlusionTexture);
      setup_texture_if_needed(material.roughness, gltfMaterial.pbrMetallicRoughness.metallicRoughnessTexture);
      setup_texture_if_needed(material.gltfExtras.emissiveTexture, gltfMaterial.emissiveTexture);
      //! @note The following textures are ignored: metallic, reflectance, clearCoat, clearCoatRoughness and anisotropy because there is nothing
      //! that matches in tiny_gltf
      for (auto &[extension_name, extension] : gltfMaterial.extensions) {
        if (extension.IsObject()) {
          for (auto &[key, value] : extension.Get<tinygltf::Value::Object>()) {
            if (value.Has("index")) {
              auto &texture_info_obj = value.Get<tinygltf::Value::Object>();
              if (material.gltfExtras.texture_from_specular_glossiness_diffuse && extension_name == specular_glossiness_extension &&
                  key == "diffuseTexture") {
                // The texture that was assigned to the metallic-roughness base color texture was actually a
                // specular-glossiness diffuse texture. Move it from there to here.
                texture_info_obj["index"] = tinygltf::Value((int)gltfMaterial.pbrMetallicRoughness.baseColorTexture.index);
                gltfMaterial.pbrMetallicRoughness.baseColorTexture.index = -1;
                continue;
              }
              const auto idx = texture_info_obj["index"].GetNumberAsInt();
              if (idx >= 0 && idx < material.gltfExtras.extension_images.size()) {
                tinygltf::TextureInfo texture_info;
                setup_texture_if_needed(material.gltfExtras.extension_images[idx], texture_info);
                texture_info_obj["index"] = tinygltf::Value(texture_info.index);
                texture_info_obj["texCoord"] = tinygltf::Value(texture_info.texCoord);
              }
            }
          }
        }
      }
    }
    gltfModel.materials.emplace_back(std::move(gltfMaterial));

    gltfMesh.primitives.emplace_back(std::move(gltfPrimitive));
  }

  // setup scene node
  gltfScene.nodes.emplace_back(gltfModel.nodes.size());
  tinygltf::Node node;
  node.mesh = gltfModel.meshes.size();
  gltfModel.nodes.emplace_back(std::move(node));
  gltfModel.meshes.emplace_back(std::move(gltfMesh));
  gltfModel.scenes.emplace_back(std::move(gltfScene));
  gltfModel.buffers.emplace_back(std::move(gltfBuffer));
  gltfModel.asset.generator = "Polycam";
  gltfModel.asset.version = "2.0";
  gltfModel.defaultScene = 0;
  gltfModel.extensionsUsed.insert(gltfModel.extensionsUsed.begin(), extensions_used.begin(), extensions_used.end());

  // setup GLTF
  tinygltf::TinyGLTF gltf;
  const bool bEmbedImages(bBinary), bEmbedBuffers(bBinary), bPrettyPrint(!bBinary);
  return gltf.WriteGltfSceneToFile(&gltfModel, fileName, bEmbedImages, bEmbedBuffers, bPrettyPrint, bBinary);
}

/*
 * Add a vertex from an Open3D mesh to a set of vectors, and return the vertex
 * index in the destination mesh that the destination vectors describe. Intended
 * as a helper for preparing data for GLTF export.
 */
inline uint32_t addVertexToBuffers(const geometry::TriangleMesh &mesh, const size_t &src_tri_idx, const size_t &src_wedge_idx,
                                   std::vector<float> &vertices, std::vector<float> &normals, std::vector<float> &uvs, const bool &write_normals) {
  uint32_t src_vertex_idx = mesh.triangles_[src_tri_idx][src_wedge_idx];
  // There exists 3 UVs per triangle face, arranged sequentially.
  uint32_t src_uv_idx = src_tri_idx * 3 + src_wedge_idx;
  uint32_t dst_vertex_idx = vertices.size() / 3;
  vertices.push_back(mesh.vertices_[src_vertex_idx][0]);
  vertices.push_back(mesh.vertices_[src_vertex_idx][1]);
  vertices.push_back(mesh.vertices_[src_vertex_idx][2]);

  uvs.push_back(mesh.triangle_uvs_[src_uv_idx][0]);
  uvs.push_back(mesh.triangle_uvs_[src_uv_idx][1]);

  if (write_normals && mesh.HasVertexNormals()) {
    normals.push_back(mesh.vertex_normals_[src_vertex_idx][0]);
    normals.push_back(mesh.vertex_normals_[src_vertex_idx][1]);
    normals.push_back(mesh.vertex_normals_[src_vertex_idx][2]);
  }
  return dst_vertex_idx;
}

/*
 * The logic of writing a textured mesh to GLTF considerably diverges
 * from other cases. Open3D (and 3D model formats such as OBJ) favors
 * per-face texture coordinates. This makes sense because adjacent triangles
 * might reference texture patches that are not adjacent, and a vertex can
 * need a separate UV depending on which triangle is being rendered.
 *
 * GLTF, on the other hand, assumes that texture coordinates are per-vertex.
 * This mirrors more how one would write a shader around this scenario. This
 * function therefore takes the following approach:
 *
 * - Divides vertices per-material, effectively creating N sub-meshes.
 * - Iterates through the faces associated with each sub-mesh.
 * - Assigns a unique vertex per wedge.
 */
inline tinygltf::Model WriteTexturedTriangleMeshToGLTFModel(const std::string &filename, const geometry::TriangleMesh &mesh,
                                                            const bool &write_vertex_normals) {
  tinygltf::Model model;
  tinygltf::Scene gltf_scene;
  std::string parent_dir = utility::filesystem::GetFileParentDirectory(filename);

  for (size_t tex_idx = 0; tex_idx < mesh.textures_.size(); tex_idx++) {
    tinygltf::Primitive gltf_primitive;
    gltf_primitive.mode = TINYGLTF_MODE_TRIANGLES;

    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> uvs;
    std::vector<uint32_t> indices;
    Eigen::Vector3d min_vertex(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    Eigen::Vector3d max_vertex(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    for (size_t tri_idx = 0; tri_idx < mesh.triangles_.size(); tri_idx++) {
      if (mesh.triangle_material_ids_[tri_idx] != tex_idx) {
        continue;
      }
      for (size_t wedge_idx = 0; wedge_idx < 3; wedge_idx++) {
        uint32_t src_vertex_idx = mesh.triangles_[tri_idx][wedge_idx];
        uint32_t dst_vertex_idx = addVertexToBuffers(mesh, tri_idx, wedge_idx, vertices, normals, uvs, write_vertex_normals);
        indices.push_back(dst_vertex_idx);
        Eigen::Vector3d vertex = mesh.vertices_[src_vertex_idx];
        // Since we're doing this per-material, we have to calculate maxes and
        // mins ourself.
        for (size_t comp_idx = 0; comp_idx < 3; comp_idx++) {
          if (min_vertex[comp_idx] > vertex[comp_idx]) {
            min_vertex[comp_idx] = vertex[comp_idx];
          }
          if (max_vertex[comp_idx] < vertex[comp_idx]) {
            max_vertex[comp_idx] = vertex[comp_idx];
          }
        }
      }
    }
    size_t num_vertices = vertices.size() / 3;

    tinygltf::Buffer buffer;

    // indices
    {
      tinygltf::BufferView buffer_view;
      buffer_view.name = "buffer-" + std::to_string(tex_idx) + "-bufferview-indices-0";
      buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
      buffer_view.buffer = model.buffers.size();
      extendBuffer(indices, buffer, buffer_view.byteOffset, buffer_view.byteLength);
      model.bufferViews.push_back(buffer_view);

      tinygltf::Accessor accessor;
      accessor.name = "buffer-" + std::to_string(tex_idx) + "-accessor-indices-buffer-0-mesh-0";
      accessor.type = TINYGLTF_TYPE_SCALAR;
      accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
      accessor.count = indices.size();
      accessor.bufferView = model.bufferViews.size() - 1;
      accessor.minValues.push_back(0);
      accessor.maxValues.push_back(indices.size() - 1);

      model.accessors.push_back(accessor);

      gltf_primitive.indices = int(model.accessors.size()) - 1;
    }

    // vertices
    {
      tinygltf::BufferView buffer_view;
      buffer_view.name = "buffer-" + std::to_string(tex_idx) + "-bufferview-vertices-0";
      buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
      buffer_view.buffer = model.buffers.size();
      extendBuffer(vertices, buffer, buffer_view.byteOffset, buffer_view.byteLength);
      model.bufferViews.push_back(buffer_view);

      tinygltf::Accessor accessor;
      accessor.name = "buffer-" + std::to_string(tex_idx) + "-accessor-position-buffer-0-mesh-0";
      accessor.type = TINYGLTF_TYPE_VEC3;
      accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      accessor.count = num_vertices;
      accessor.bufferView = model.bufferViews.size() - 1;
      accessor.minValues.push_back(min_vertex[0]);
      accessor.minValues.push_back(min_vertex[1]);
      accessor.minValues.push_back(min_vertex[2]);
      accessor.maxValues.push_back(max_vertex[0]);
      accessor.maxValues.push_back(max_vertex[1]);
      accessor.maxValues.push_back(max_vertex[2]);

      model.accessors.push_back(accessor);

      gltf_primitive.attributes.insert(std::make_pair("POSITION", static_cast<int>(model.accessors.size()) - 1));
    }

    // uv
    {
      tinygltf::BufferView buffer_view;
      buffer_view.name = "buffer-" + std::to_string(tex_idx) + "-bufferview-uvs-0";
      buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
      buffer_view.buffer = model.buffers.size();
      extendBuffer(uvs, buffer, buffer_view.byteOffset, buffer_view.byteLength);
      model.bufferViews.push_back(buffer_view);

      tinygltf::Accessor accessor;
      accessor.name = "buffer-" + std::to_string(tex_idx) + "-accessor-uvs-buffer-0-mesh-0";
      accessor.type = TINYGLTF_TYPE_VEC2;
      accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      accessor.count = num_vertices;
      accessor.bufferView = model.bufferViews.size() - 1;
      model.accessors.push_back(accessor);

      gltf_primitive.attributes.insert(std::make_pair("TEXCOORD_0", static_cast<int>(model.accessors.size()) - 1));
      gltf_primitive.material = tex_idx;

      tinygltf::Image image;
      auto skipped_external_texture = TrySkippedExternalTexture(mesh.textures_[tex_idx], std::filesystem::path(parent_dir));
      if (skipped_external_texture.has_value()) {
        image = std::move(*skipped_external_texture);
      } else {
        // Save the bytes to the GLTF directly.
        auto encoded_image = EncodeImage(mesh.textures_[tex_idx], parent_dir + "texture" + std::to_string(tex_idx) + ".jpg");
        image.mimeType = encoded_image.mime_type_;
        tinygltf::BufferView image_buffer_view;
        image_buffer_view.name = "buffer-" + std::to_string(tex_idx) + "-bufferview-image-0";
        image_buffer_view.buffer = model.buffers.size();
        extendBuffer(encoded_image.data_, buffer, image_buffer_view.byteOffset, image_buffer_view.byteLength);
        model.bufferViews.push_back(image_buffer_view);
        image.bufferView = model.bufferViews.size() - 1;
        image.as_is = true;
      }

      tinygltf::Material gltf_mat;
      gltf_mat.pbrMetallicRoughness.metallicFactor = 0.0;
      gltf_mat.pbrMetallicRoughness.roughnessFactor = 1.0;
      gltf_mat.pbrMetallicRoughness.baseColorTexture.index = model.images.size();
      gltf_mat.pbrMetallicRoughness.baseColorTexture.texCoord = 0;
      gltf_mat.name = image.uri;

      tinygltf::Texture gltf_texture;
      gltf_texture.source = model.images.size();
      gltf_texture.name = image.uri;

      model.images.push_back(image);
      model.materials.push_back(gltf_mat);
      model.textures.push_back(gltf_texture);
    }

    // normals
    if (write_vertex_normals && mesh.HasVertexNormals()) {
      tinygltf::BufferView buffer_view;
      buffer_view.name = "buffer-0-bufferview-normals";
      buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
      buffer_view.buffer = model.buffers.size();
      extendBuffer(normals, buffer, buffer_view.byteOffset, buffer_view.byteLength);
      model.bufferViews.push_back(buffer_view);

      tinygltf::Accessor accessor;
      accessor.name = "buffer-0-accessor-normal-buffer-0-mesh-0";
      accessor.type = TINYGLTF_TYPE_VEC3;
      accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      accessor.count = num_vertices;
      accessor.bufferView = model.bufferViews.size() - 1;

      model.accessors.push_back(accessor);

      gltf_primitive.attributes.insert(std::make_pair("NORMAL", static_cast<int>(model.accessors.size()) - 1));
    }

    // Add new mesh
    tinygltf::Mesh gltf_mesh;
    gltf_mesh.primitives.push_back(gltf_primitive);
    model.meshes.push_back(gltf_mesh);

    // Add  node for mesh
    tinygltf::Node gltf_node;
    gltf_node.mesh = model.meshes.size() - 1;
    gltf_scene.nodes.push_back(model.nodes.size());
    model.nodes.push_back(gltf_node);

    // Add buffer for mesh
    model.buffers.push_back(buffer);
  }

  model.asset.generator = "Polycam";
  model.asset.version = "2.0";
  model.defaultScene = 0;
  model.scenes.push_back(gltf_scene);

  return model;
}

inline tinygltf::Model WriteTriangleMeshToGLTFModel(const std::string &filename, const geometry::TriangleMesh &mesh, const bool &write_vertex_normals,
                                                    const bool &write_vertex_colors) {
  tinygltf::Model model;
  model.asset.generator = "Polycam";
  model.asset.version = "2.0";
  model.defaultScene = 0;

  size_t byte_length;
  size_t num_of_vertices = mesh.vertices_.size();
  size_t num_of_triangles = mesh.triangles_.size();

  float float_temp;
  unsigned char *temp = NULL;

  tinygltf::BufferView indices_buffer_view_array;
  bool save_indices_as_uint32 = num_of_vertices > 65536;
  indices_buffer_view_array.name = save_indices_as_uint32 ? "buffer-0-bufferview-uint" : "buffer-0-bufferview-ushort";
  indices_buffer_view_array.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
  indices_buffer_view_array.buffer = 0;
  indices_buffer_view_array.byteLength = 0;
  model.bufferViews.push_back(indices_buffer_view_array);
  size_t indices_buffer_view_index = model.bufferViews.size() - 1;

  tinygltf::BufferView buffer_view_array;
  buffer_view_array.name = "buffer-0-bufferview-vec3", buffer_view_array.target = TINYGLTF_TARGET_ARRAY_BUFFER;
  buffer_view_array.buffer = 0;
  buffer_view_array.byteLength = 0;
  buffer_view_array.byteOffset = 0;
  buffer_view_array.byteStride = 12;
  model.bufferViews.push_back(buffer_view_array);
  size_t mesh_attributes_buffer_view_index = model.bufferViews.size() - 1;

  tinygltf::Scene gltf_scene;
  gltf_scene.nodes.push_back(0);
  model.scenes.push_back(gltf_scene);

  tinygltf::Node gltf_node;
  gltf_node.mesh = 0;
  model.nodes.push_back(gltf_node);

  tinygltf::Mesh gltf_mesh;
  tinygltf::Primitive gltf_primitive;

  tinygltf::Accessor indices_accessor;
  indices_accessor.name = "buffer-0-accessor-indices-buffer-0-mesh-0";
  indices_accessor.type = TINYGLTF_TYPE_SCALAR;
  indices_accessor.componentType = save_indices_as_uint32 ? TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT : TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT;
  indices_accessor.count = 3 * num_of_triangles;
  byte_length = 3 * num_of_triangles * (save_indices_as_uint32 ? sizeof(uint32_t) : sizeof(uint16_t));

  indices_accessor.bufferView = int(indices_buffer_view_index);
  indices_accessor.byteOffset = model.bufferViews[indices_buffer_view_index].byteLength;
  model.bufferViews[indices_buffer_view_index].byteLength += byte_length;

  std::vector<unsigned char> index_buffer;
  for (size_t tidx = 0; tidx < num_of_triangles; ++tidx) {
    const Eigen::Vector3i &triangle = mesh.triangles_[tidx];
    size_t uint_size = save_indices_as_uint32 ? sizeof(uint32_t) : sizeof(uint16_t);
    for (size_t i = 0; i < 3; ++i) {
      temp = (unsigned char *)&(triangle(i));
      for (size_t j = 0; j < uint_size; ++j) {
        index_buffer.push_back(temp[j]);
      }
    }
  }

  indices_accessor.minValues.push_back(0);
  indices_accessor.maxValues.push_back(3 * int(num_of_triangles) - 1);
  model.accessors.push_back(indices_accessor);
  gltf_primitive.indices = int(model.accessors.size()) - 1;

  tinygltf::Accessor positions_accessor;
  positions_accessor.name = "buffer-0-accessor-position-buffer-0-mesh-0";
  positions_accessor.type = TINYGLTF_TYPE_VEC3;
  positions_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  positions_accessor.count = num_of_vertices;
  byte_length = 3 * num_of_vertices * sizeof(float);
  positions_accessor.bufferView = int(mesh_attributes_buffer_view_index);
  positions_accessor.byteOffset = model.bufferViews[mesh_attributes_buffer_view_index].byteLength;
  model.bufferViews[mesh_attributes_buffer_view_index].byteLength += byte_length;

  std::vector<unsigned char> mesh_attribute_buffer;
  for (size_t vidx = 0; vidx < num_of_vertices; ++vidx) {
    const Eigen::Vector3d &vertex = mesh.vertices_[vidx];
    for (size_t i = 0; i < 3; ++i) {
      float_temp = (float)vertex(i);
      temp = (unsigned char *)&(float_temp);
      for (size_t j = 0; j < sizeof(float); ++j) {
        mesh_attribute_buffer.push_back(temp[j]);
      }
    }
  }

  Eigen::Vector3d min_bound = mesh.GetMinBound();
  positions_accessor.minValues.push_back(min_bound[0]);
  positions_accessor.minValues.push_back(min_bound[1]);
  positions_accessor.minValues.push_back(min_bound[2]);
  Eigen::Vector3d max_bound = mesh.GetMaxBound();
  positions_accessor.maxValues.push_back(max_bound[0]);
  positions_accessor.maxValues.push_back(max_bound[1]);
  positions_accessor.maxValues.push_back(max_bound[2]);
  model.accessors.push_back(positions_accessor);
  gltf_primitive.attributes.insert(std::make_pair("POSITION", static_cast<int>(model.accessors.size()) - 1));

  if (write_vertex_normals && mesh.HasVertexNormals()) {
    tinygltf::Accessor normals_accessor;
    normals_accessor.name = "buffer-0-accessor-normal-buffer-0-mesh-0";
    normals_accessor.type = TINYGLTF_TYPE_VEC3;
    normals_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    normals_accessor.count = mesh.vertices_.size();
    size_t byte_length = 3 * mesh.vertices_.size() * sizeof(float);
    normals_accessor.bufferView = int(mesh_attributes_buffer_view_index);
    normals_accessor.byteOffset = model.bufferViews[mesh_attributes_buffer_view_index].byteLength;
    model.bufferViews[mesh_attributes_buffer_view_index].byteLength += byte_length;

    for (size_t vidx = 0; vidx < num_of_vertices; ++vidx) {
      const Eigen::Vector3d &normal = mesh.vertex_normals_[vidx];
      for (size_t i = 0; i < 3; ++i) {
        float_temp = (float)normal(i);
        temp = (unsigned char *)&(float_temp);
        for (size_t j = 0; j < sizeof(float); ++j) {
          mesh_attribute_buffer.push_back(temp[j]);
        }
      }
    }

    model.accessors.push_back(normals_accessor);
    gltf_primitive.attributes.insert(std::make_pair("NORMAL", static_cast<int>(model.accessors.size()) - 1));
  }

  if (write_vertex_colors && mesh.HasVertexColors()) {
    tinygltf::Accessor colors_accessor;
    colors_accessor.name = "buffer-0-accessor-color-buffer-0-mesh-0";
    colors_accessor.type = TINYGLTF_TYPE_VEC3;
    colors_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    colors_accessor.count = mesh.vertices_.size();
    size_t byte_length = 3 * mesh.vertices_.size() * sizeof(float);
    colors_accessor.bufferView = int(mesh_attributes_buffer_view_index);
    colors_accessor.byteOffset = model.bufferViews[mesh_attributes_buffer_view_index].byteLength;
    model.bufferViews[mesh_attributes_buffer_view_index].byteLength += byte_length;

    for (size_t vidx = 0; vidx < num_of_vertices; ++vidx) {
      const Eigen::Vector3d &color = mesh.vertex_colors_[vidx];
      for (size_t i = 0; i < 3; ++i) {
        float_temp = (float)color(i);
        temp = (unsigned char *)&(float_temp);
        for (size_t j = 0; j < sizeof(float); ++j) {
          mesh_attribute_buffer.push_back(temp[j]);
        }
      }
    }

    model.accessors.push_back(colors_accessor);
    gltf_primitive.attributes.insert(std::make_pair("COLOR_0", static_cast<int>(model.accessors.size()) - 1));
  }

  gltf_primitive.mode = TINYGLTF_MODE_TRIANGLES;
  gltf_mesh.primitives.push_back(gltf_primitive);
  model.meshes.push_back(gltf_mesh);

  model.bufferViews[0].byteOffset = 0;
  model.bufferViews[1].byteOffset = index_buffer.size();

  tinygltf::Buffer buffer;
  buffer.uri = "geometry.bin";
  buffer.data.resize(index_buffer.size() + mesh_attribute_buffer.size());
  memcpy(buffer.data.data(), index_buffer.data(), index_buffer.size());
  memcpy(buffer.data.data() + index_buffer.size(), mesh_attribute_buffer.data(), mesh_attribute_buffer.size());
  model.buffers.push_back(buffer);
  return model;
}

bool WriteTriangleMeshToGLTF(const std::string &filename, const geometry::TriangleMesh &mesh, bool write_ascii /* = false*/,
                             bool compressed /* = false*/, bool write_vertex_normals /* = true*/, bool write_vertex_colors /* = true*/,
                             bool write_triangle_uvs /* = true*/, bool print_progress) {
#if 1
  return SaveMeshGLTF(filename, mesh);
#else
  tinygltf::Model model;
  if (write_triangle_uvs && mesh.HasTriangleUvs()) {
    model = WriteTexturedTriangleMeshToGLTFModel(filename, mesh, write_vertex_normals);
  } else {
    model = WriteTriangleMeshToGLTFModel(filename, mesh, write_vertex_normals, write_vertex_colors);
  }

  tinygltf::TinyGLTF loader;
  std::string filename_ext = utility::filesystem::GetFileExtensionInLowerCase(filename);
  if (filename_ext == "glb") {
    if (!loader.WriteGltfSceneToFile(&model, filename, false, true, true, true)) {
      utility::LogWarning("Write GLTF failed.");
      return false;
    }
  } else {
    if (!loader.WriteGltfSceneToFile(&model, filename, false, true, true, false)) {
      utility::LogWarning("Write GLTF failed.");
      return false;
    }
  }

  return true;
#endif
}

}  // namespace io
}  // namespace open3d

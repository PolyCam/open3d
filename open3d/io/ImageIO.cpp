// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
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

#include "open3d/io/ImageIO.h"

#include <filesystem>
#include <fstream>
#include <unordered_map>

#include "open3d/utility/Console.h"
#include "open3d/utility/FileSystem.h"

namespace open3d {

namespace {
using namespace io;

static const std::unordered_map<std::string, std::function<bool(const std::string &, geometry::Image &)> > file_extension_to_image_read_function{
    {"png", ReadImageFromPNG},
    {"jpg", ReadImageFromJPG},
    {"jpeg", ReadImageFromJPG},
};

static const std::unordered_map<std::string, std::function<bool(const std::string &, const geometry::Image &, int)> >
    file_extension_to_image_write_function{
        {"png", WriteImageToPNG},
        {"jpg", WriteImageToJPG},
        {"jpeg", WriteImageToJPG},
    };

}  // unnamed namespace

namespace io {

std::vector<uint8_t> ReadFileIntoBuffer(const std::string &path) {
  auto stream = std::ifstream(path, std::ios::in | std::ios::binary);
  stream.seekg(0, std::ios::end);
  auto size = stream.tellg();
  stream.seekg(0);
  std::vector<uint8_t> buffer(size);
  stream.read((char *)buffer.data(), size);
  stream.close();
  return (buffer);
}

std::shared_ptr<geometry::Image> CreateImageFromFile(const std::string &filename, TextureLoadMode texture_load_mode) {
  auto image = std::make_shared<geometry::Image>();
  ReadImage(filename, *image, texture_load_mode);
  return image;
}

bool ReadImage(const std::string &filename, geometry::Image &image, TextureLoadMode texture_load_mode) {
  if (texture_load_mode == TextureLoadMode::pass_through || texture_load_mode == TextureLoadMode::ignore_external_files) {
    if (texture_load_mode == TextureLoadMode::pass_through) {
      image.pass_through_ = geometry::Image::EncodedData{ReadFileIntoBuffer(filename), utility::filesystem::GetMimeType(filename)};
    } else {
      image.pass_through_ = std::filesystem::path(filename);
    }
    // Make a fake 1x1 RGB image just in case somewhere else in Open3D the image integrity is verified.
    image.Prepare(1, 1, 3, 1);
    image.data_ = std::vector<uint8_t>(3, uint8_t(0));
    return true;
  }
  std::string filename_ext = utility::filesystem::GetFileExtensionInLowerCase(filename);
  if (filename_ext.empty()) {
    utility::LogWarning("Read geometry::Image failed: missing file extension.");
    return false;
  }
  auto map_itr = file_extension_to_image_read_function.find(filename_ext);
  if (map_itr == file_extension_to_image_read_function.end()) {
    utility::LogWarning("Read geometry::Image failed: file extension {} unknown", filename_ext);
    return false;
  }
  return map_itr->second(filename, image);
}

bool WriteImage(const std::string &filename, const geometry::Image &image, int quality /* = 90*/) {
  if (image.pass_through_.has_value()) {
    return (std::visit(
        [&](const auto &pass_through) {
          using PassThroughType = typename std::decay<decltype(pass_through)>::type;
          if constexpr (std::is_same<PassThroughType, geometry::Image::EncodedData>::value) {
            std::ofstream file_out(filename, std::ios::out | std::ios::binary);
            if (!file_out.is_open()) {
              return false;
            }
            file_out.write(reinterpret_cast<const char *>(pass_through.data_.data()), pass_through.data_.size());
            file_out.close();
            return true;
          } else if constexpr (std::is_same<PassThroughType, std::filesystem::path>::value) {
            if (pass_through == std::filesystem::path(filename)) {
              return true;
            }
            return std::filesystem::copy_file(pass_through, filename);
          }
        },
        *image.pass_through_));
  } else {
    std::string filename_ext = utility::filesystem::GetFileExtensionInLowerCase(filename);
    if (filename_ext.empty()) {
      utility::LogWarning("Write geometry::Image failed: unknown file extension.");
      return false;
    }
    auto map_itr = file_extension_to_image_write_function.find(filename_ext);
    if (map_itr == file_extension_to_image_write_function.end()) {
      utility::LogWarning("Write geometry::Image failed: unknown file extension.");
      return false;
    }
    return map_itr->second(filename, image, quality);
  }
}

}  // namespace io
}  // namespace open3d

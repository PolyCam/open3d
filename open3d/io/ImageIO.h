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

#pragma once

#include <string>

#include "open3d/geometry/Image.h"

namespace open3d {
namespace io {

enum class TextureLoadMode {
  normal,
  pass_through,          // Textures are not decoded on loading, nor encoded on writing. Only available with GLB, GLTF, and OBJ files.
  ignore_external_files  // External texture files are neither read or written on writing. Only available with GLTF files. Resorts to
                         // pass through on GLB and OBJ files, embedded textures on GLTF files and is ignored on other formats.
};

std::vector<uint8_t> ReadFileIntoBuffer(const std::string &path);

/// Factory function to create an image from a file (ImageFactory.cpp)
/// Return an empty image if fail to read the file.
std::shared_ptr<geometry::Image> CreateImageFromFile(const std::string &filename, TextureLoadMode texture_load_mode = TextureLoadMode::normal);

/// The general entrance for reading an Image from a file
/// The function calls read functions based on the extension name of filename.
/// \return return true if the read function is successful, false otherwise.
bool ReadImage(const std::string &filename, geometry::Image &image, TextureLoadMode texture_load_mode = TextureLoadMode::normal);

/// The general entrance for writing an Image to a file
/// The function calls write functions based on the extension name of filename.
/// If the write function supports quality, the parameter will be used.
/// Otherwise it will be ignored.
/// \return return true if the write function is successful, false otherwise.
bool WriteImage(const std::string &filename, const geometry::Image &image, int quality = 90);

bool ReadImageFromMemoryPNG(const uint8_t *data, size_t size, geometry::Image &image);
bool ReadImageFromPNG(const std::string &filename, geometry::Image &image);

bool WriteImageToPNG(const std::string &filename, const geometry::Image &image, int quality);

bool ReadImageFromMemoryJPG(const uint8_t *data, size_t size, geometry::Image &image);
bool ReadImageFromJPG(const std::string &filename, geometry::Image &image);

bool WriteImageToJPG(const std::string &filename, const geometry::Image &image, int quality = 90);

}  // namespace io
}  // namespace open3d

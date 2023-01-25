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

#include <png.h>

#include "open3d/io/ImageIO.h"
#include "open3d/utility/Console.h"

namespace open3d {

namespace {
using namespace io;

void SetPNGImageFromImage(const geometry::Image &image, png_image &pngimage) {
  pngimage.width = image.width_;
  pngimage.height = image.height_;
  pngimage.format = 0;
  if (image.bytes_per_channel_ == 2) {
    pngimage.format |= PNG_FORMAT_FLAG_LINEAR;
  }
  if (image.num_of_channels_ == 3) {
    pngimage.format |= PNG_FORMAT_FLAG_COLOR;
  }
}

}  // unnamed namespace

namespace io {

struct MemStream {
  const uint8_t *data;
  size_t size;
  size_t pos;
};

void custom_png_read_file(png_structp png_ptr, png_bytep buffer, png_size_t size) {
  png_voidp const ptrIO = png_get_io_ptr(png_ptr);
  MemStream &stream = *static_cast<MemStream *>(ptrIO);
  if (stream.pos >= stream.size)
    return;
  if (stream.pos + size >= stream.size)
    size = stream.size - stream.pos;
  memcpy(buffer, stream.data + stream.pos, size);
  stream.pos += size;
}

bool ReadImageFromMemoryPNG(const uint8_t *data, size_t size, geometry::Image &image) {
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr)
    return false;

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
    return false;

  if (setjmp(png_jmpbuf(png_ptr)))
    return false;

  MemStream stream;
  stream.data = data;
  stream.size = size;
  stream.pos = 0;
  png_set_read_fn(png_ptr, &stream, custom_png_read_file);

  png_read_info(png_ptr, info_ptr);

  png_uint_32 width, height;
  int bitdepth, colortype;
  png_get_IHDR(png_ptr, info_ptr, &width, &height, &bitdepth, &colortype, NULL, NULL, NULL);

  if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
    png_set_tRNS_to_alpha(png_ptr);
    colortype = PNG_COLOR_TYPE_RGB_ALPHA;
  }

#if 0
	// if it doesn't have a file gamma, don't do any correction ("do no harm")
	if (png_get_valid(png_ptr, info_ptr, PNG_INFO_gAMA)) {
		double gamma = 0;
		const double screen_gamma = 2.2;
		if (png_get_gAMA(png_ptr, info_ptr, &gamma))
			png_set_gamma(png_ptr, screen_gamma, gamma);
	}
#endif

  int num_of_channels = 3;
  int bytes_per_channel = 1;
  switch (colortype) {
    case PNG_COLOR_TYPE_GRAY:
      if (bitdepth < 8)
        png_set_expand_gray_1_2_4_to_8(png_ptr);
      num_of_channels = 1;
      break;
    case PNG_COLOR_TYPE_PALETTE:
      // png_set_palette_to_rgb(png_ptr);
      num_of_channels = 1;
      break;
    case PNG_COLOR_TYPE_GRAY_ALPHA:
      // png_set_gray_to_rgb(png_ptr);
      num_of_channels = 1;
      break;
    case PNG_COLOR_TYPE_RGB:
      break;
    default:
      assert("error: unsupported PNG image" == NULL);
      return false;
  }
  if (bitdepth == 16)
    png_set_strip_16(png_ptr);
  const size_t line_bytes = width * num_of_channels * bytes_per_channel;
  const size_t row_stride = png_get_rowbytes(png_ptr, info_ptr);

  png_set_interlace_handling(png_ptr);
  png_read_update_info(png_ptr, info_ptr);

  // read image directly to the data buffer
  image.Prepare(width, height, num_of_channels, bytes_per_channel);
  uint8_t *pdata = image.data_.data();
  for (size_t j = 0; j < height; ++j, pdata += line_bytes)
    png_read_row(png_ptr, (png_bytep)pdata, NULL);

  png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  return true;
}

bool ReadImageFromPNG(const std::string &filename, geometry::Image &image) {
  png_image pngimage;
  memset(&pngimage, 0, sizeof(pngimage));
  pngimage.version = PNG_IMAGE_VERSION;
  if (png_image_begin_read_from_file(&pngimage, filename.c_str()) == 0) {
    utility::LogWarning("Read PNG failed: unable to parse header.");
    return false;
  }

  // Clear colormap flag if necessary to ensure libpng expands the colo
  // indexed pixels to full color
  if (pngimage.format & PNG_FORMAT_FLAG_COLORMAP) {
    pngimage.format &= ~PNG_FORMAT_FLAG_COLORMAP;
  }

  image.Prepare(pngimage.width, pngimage.height, PNG_IMAGE_SAMPLE_CHANNELS(pngimage.format), PNG_IMAGE_SAMPLE_COMPONENT_SIZE(pngimage.format));

  if (png_image_finish_read(&pngimage, NULL, image.data_.data(), 0, NULL) == 0) {
    utility::LogWarning("Read PNG failed: unable to read file: {}", filename);
    utility::LogWarning("PNG error: {}", pngimage.message);
    return false;
  }
  return true;
}

bool WriteImageToPNG(const std::string &filename, const geometry::Image &image, int quality) {
  if (!image.HasData()) {
    utility::LogWarning("Write PNG failed: image has no data: {}", filename);
    return false;
  }
  png_image pngimage;
  memset(&pngimage, 0, sizeof(pngimage));
  pngimage.version = PNG_IMAGE_VERSION;
  SetPNGImageFromImage(image, pngimage);
  if (png_image_write_to_file(&pngimage, filename.c_str(), 0, image.data_.data(), 0, NULL) == 0) {
    utility::LogWarning("Write PNG failed: unable to write file: {}", filename);
    return false;
  }
  return true;
}

}  // namespace io
}  // namespace open3d

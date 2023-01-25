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

// clang-format off
#include <cstddef>
#include <cstdio>

#include "open3d/io/ImageIO.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/FileSystem.h"
#include <jpeglib.h>  // Include after cstddef to define size_t
// clang-format on

#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>

namespace open3d {
namespace io {

#define JPG_BUFFER_SIZE (16 * 1024)

struct MemStream {
  const uint8_t *data;
  size_t size;
  size_t pos;
};

struct JpegErrorMgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

struct JpegSource {
  struct jpeg_source_mgr pub;
  MemStream stream;
};

// F U N C T I O N S ///////////////////////////////////////////////

METHODDEF(void)
stub(j_decompress_ptr cinfo) {
  JpegSource *source = (JpegSource *)cinfo->src;
  source->stream.pos = 0;
}

METHODDEF(boolean)
fill_input_buffer(j_decompress_ptr cinfo) {
  JpegSource *source = (JpegSource *)cinfo->src;
  if (source->stream.pos >= source->stream.size)
    return FALSE;
  size_t size = JPG_BUFFER_SIZE;
  if (source->stream.pos + size >= source->stream.size)
    size = source->stream.size - source->stream.pos;
  source->pub.next_input_byte = source->stream.data + source->stream.pos;
  source->pub.bytes_in_buffer = size;
  source->stream.pos += size;
  return TRUE;
}

METHODDEF(void)
skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
  JpegSource *source = (JpegSource *)cinfo->src;
  if (num_bytes > (long)source->pub.bytes_in_buffer) {
    // We need to skip more data than we have in the buffer.
    source->stream.pos += num_bytes - source->pub.bytes_in_buffer;
    source->pub.next_input_byte += source->pub.bytes_in_buffer;
    source->pub.bytes_in_buffer = 0;
  } else {
    // Skip portion of the buffer
    source->pub.bytes_in_buffer -= num_bytes;
    source->pub.next_input_byte += num_bytes;
  }
}

METHODDEF(void)
error_exit(j_common_ptr cinfo) {
  JpegErrorMgr *err_mgr = (JpegErrorMgr *)(cinfo->err);

  /* Return control to the setjmp point */
  longjmp(err_mgr->setjmp_buffer, 1);
}

bool ReadImageFromMemoryJPG(const uint8_t *data, size_t size, geometry::Image &image) {
  jpeg_decompress_struct cinfo;  // IJG JPEG codec structure
  JpegErrorMgr jerr;             // error processing manager state
  JpegSource source;             // memory buffer source
  JSAMPARRAY buffer;             // line memory buffer
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = error_exit;

  if (setjmp(jerr.setjmp_buffer) != 0)
    return false;
  jpeg_create_decompress(&cinfo);

  // Prepare for suspending reader
  source.pub.init_source = stub;
  source.pub.fill_input_buffer = fill_input_buffer;
  source.pub.skip_input_data = skip_input_data;
  source.pub.resync_to_restart = jpeg_resync_to_restart;
  source.pub.term_source = stub;
  source.pub.bytes_in_buffer = 0;  // forces fill_input_buffer on first read
  source.pub.next_input_byte = NULL;
  source.stream.data = data;
  source.stream.size = size;
  cinfo.src = &source.pub;

  jpeg_read_header(&cinfo, TRUE);

  // We only support two channel types: gray, and RGB.
  int num_of_channels = 3;
  int bytes_per_channel = 1;
  switch (cinfo.jpeg_color_space) {
    case JCS_RGB:
    case JCS_YCbCr:
      cinfo.out_color_space = JCS_RGB;
      cinfo.out_color_components = 3;
      num_of_channels = 3;
      break;
    case JCS_GRAYSCALE:
      cinfo.jpeg_color_space = JCS_GRAYSCALE;
      cinfo.out_color_components = 1;
      num_of_channels = 1;
      break;
    case JCS_CMYK:
    case JCS_YCCK:
    default:
      utility::LogWarning("Read JPG failed: color space not supported.");
      jpeg_destroy_decompress(&cinfo);
      return false;
  }
  jpeg_start_decompress(&cinfo);
  image.Prepare(cinfo.output_width, cinfo.output_height, num_of_channels, bytes_per_channel);
  int row_stride = cinfo.output_width * cinfo.output_components;
  buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, row_stride, 1);
  uint8_t *pdata = image.data_.data();
  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, buffer, 1);
    memcpy(pdata, buffer[0], row_stride);
    pdata += row_stride;
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return true;
}

bool ReadImageFromJPG(const std::string &filename, geometry::Image &image) {
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file_in;
  JSAMPARRAY buffer;

  if ((file_in = utility::filesystem::FOpen(filename, "rb")) == NULL) {
    utility::LogWarning("Read JPG failed: unable to open file: {}", filename);
    return false;
  }

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, file_in);
  jpeg_read_header(&cinfo, TRUE);

  // We only support two channel types: gray, and RGB.
  int num_of_channels = 3;
  int bytes_per_channel = 1;
  switch (cinfo.jpeg_color_space) {
    case JCS_RGB:
    case JCS_YCbCr:
      cinfo.out_color_space = JCS_RGB;
      cinfo.out_color_components = 3;
      num_of_channels = 3;
      break;
    case JCS_GRAYSCALE:
      cinfo.jpeg_color_space = JCS_GRAYSCALE;
      cinfo.out_color_components = 1;
      num_of_channels = 1;
      break;
    case JCS_CMYK:
    case JCS_YCCK:
    default:
      utility::LogWarning("Read JPG failed: color space not supported.");
      jpeg_destroy_decompress(&cinfo);
      fclose(file_in);
      return false;
  }
  jpeg_start_decompress(&cinfo);
  image.Prepare(cinfo.output_width, cinfo.output_height, num_of_channels, bytes_per_channel);
  int row_stride = cinfo.output_width * cinfo.output_components;
  buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, row_stride, 1);
  uint8_t *pdata = image.data_.data();
  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, buffer, 1);
    memcpy(pdata, buffer[0], row_stride);
    pdata += row_stride;
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(file_in);
  return true;
}

bool WriteImageToJPG(const std::string &filename, const geometry::Image &image, int quality /* = 90*/) {
  if (!image.HasData()) {
    utility::LogWarning("Write JPG failed: image has no data.");
    return false;
  }
  if (image.bytes_per_channel_ != 1 || (image.num_of_channels_ != 1 && image.num_of_channels_ != 3)) {
    utility::LogWarning("Write JPG failed: unsupported image data.");
    return false;
  }
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file_out;
  JSAMPROW row_pointer[1];

  if ((file_out = utility::filesystem::FOpen(filename, "wb")) == NULL) {
    utility::LogWarning("Write JPG failed: unable to open file: {}", filename);
    return false;
  }

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  jpeg_stdio_dest(&cinfo, file_out);
  cinfo.image_width = image.width_;
  cinfo.image_height = image.height_;
  cinfo.input_components = image.num_of_channels_;
  cinfo.in_color_space = (cinfo.input_components == 1 ? JCS_GRAYSCALE : JCS_RGB);
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);
  int row_stride = image.width_ * image.num_of_channels_;
  const uint8_t *pdata = image.data_.data();
  std::vector<uint8_t> buffer(row_stride);
  while (cinfo.next_scanline < cinfo.image_height) {
    memcpy(buffer.data(), pdata, row_stride);
    row_pointer[0] = buffer.data();
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
    pdata += row_stride;
  }
  jpeg_finish_compress(&cinfo);
  fclose(file_out);
  jpeg_destroy_compress(&cinfo);
  return true;
}

}  // namespace io
}  // namespace open3d

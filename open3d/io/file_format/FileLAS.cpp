#include "laswriter.hpp"
#include "open3d/io/PointCloudIO.h"
#include "open3d/utility/FileSystem.h"
#include "open3d/utility/ProgressReporters.h"

namespace open3d {
namespace io {

bool WritePointCloudToLAS(const std::string &filename, const geometry::PointCloud &pointcloud, const WritePointCloudOption &params) {
  try {
    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(filename.c_str());

    // default to LAZ format instead of using bool(params.compressed)
    laswriteopener.set_format("LAZ");

    if (!laswriteopener.active()) {
      // create name from input name
      laswriteopener.make_file_name(filename.c_str(), -2);
    }
    LASheader hdr;
    hdr.version_major = 1;
    hdr.version_minor = 2;
    memset(hdr.generating_software, 0, 32);
    strncpy(hdr.generating_software, "Polycam", 32);
    hdr.generating_software[31] = '\0';

    if (pointcloud.HasColors()) {
      hdr.point_data_format = 2;          // position and color
      hdr.point_data_record_length = 26;  // fixed length for point_data_format == 2
    } else {
      hdr.point_data_format = 0;          // just XYZ position
      hdr.point_data_record_length = 20;  // fixed length for point_data_format == 0
    }

    // normalize all axis
    Eigen::Vector3d min = {INFINITY, INFINITY, INFINITY};
    Eigen::Vector3d max = {-INFINITY, -INFINITY, -INFINITY};
    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
      const Eigen::Vector3d &point = pointcloud.points_[i];
      for (int j = 0; j < 3; ++j) {
        if (min[j] > point[j])
          min[j] = point[j];
        if (max[j] < point[j])
          max[j] = point[j];
      }
    }

    hdr.min_x = min[0];
    hdr.min_y = min[1];
    hdr.min_x = min[2];
    hdr.max_x = max[0];
    hdr.max_y = max[1];
    hdr.max_x = max[2];

    // calculate a scale small enough that we can safely switch XYZ coordonates from double to int
    // by divinding 0.9 to the total number of points in the pointcloud we ensure that, even if
    // all points were on the same line, there is enough theoretical space to accomodate all points
    // after conversion to intiger
    // TODO: check if we could replace the 0.9 constant with
    //      minimum value of the difference (max-min) on each axis
    const double dScale = 0.9 / pointcloud.points_.size();
    hdr.x_scale_factor = dScale;
    hdr.y_scale_factor = dScale;
    hdr.z_scale_factor = dScale;

    // set record count smaller than number of points as some points are
    // identical and overwrite eachother
    hdr.number_of_point_records = 0.95 * pointcloud.points_.size();

    // init point
    LASpoint laspoint;
    laspoint.init(&hdr, hdr.point_data_format, hdr.point_data_record_length, 0);

    LASwriter *laswriter = laswriteopener.open(&hdr);
    if (laswriter == 0) {
      utility::LogWarning("Write LAS failed: unable to open file: {}", filename);
      return false;
    }

    const Eigen::Vector3d delta = {max[0] - min[0], max[1] - min[1], max[2] - min[2]};

    utility::CountingProgressReporter reporter(params.update_progress);
    reporter.SetTotal(pointcloud.points_.size());

    if (hdr.point_data_format == 2) {
      for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        const Eigen::Vector3d &point = pointcloud.points_[i];
        const Eigen::Vector3d &color = pointcloud.colors_[i];

        laspoint.set_X(point(0) / dScale);
        laspoint.set_Y(point(1) / dScale);
        laspoint.set_Z(point(2) / dScale);
        const U16 cR = color[0] * 255;
        const U16 cG = color[1] * 255;
        const U16 cB = color[2] * 255;
        const U16 RGB[3] = {cR, cG, cB};
        laspoint.set_RGB(RGB);

        if (!laswriter->write_point(&laspoint)) {
          utility::LogWarning("Write LAS failed: unable to write file: {}", filename);
          return false;  // error happened during writing.
        }

        if (i % 1000 == 0) {
          reporter.Update(i);
        }
      }
    } else {
      for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        const Eigen::Vector3d &point = pointcloud.points_[i];

        laspoint.set_X(point(0) / dScale);
        laspoint.set_Y(point(1) / dScale);
        laspoint.set_Z(point(2) / dScale);

        if (!laswriter->write_point(&laspoint)) {
          utility::LogWarning("Write LAS failed: unable to write file: {}", filename);
          return false;  // error happened during writing.
        }

        if (i % 1000 == 0) {
          reporter.Update(i);
        }
      }
    }
    std::cout << "LAS export success to : " << filename;

    // update the header
    laswriter->update_header(&hdr, TRUE);

    // close the writer
    I64 total_bytes = laswriter->close();

    reporter.Finish();
    return true;
  } catch (const std::exception &e) {
    utility::LogWarning("Write LAS failed with exception: {}", e.what());
    return false;
  } catch (...) {
    utility::LogWarning("Write LAS failed with unknown error");
    return false;
  }
}

}  // namespace io
}  // namespace open3d

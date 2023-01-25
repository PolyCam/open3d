//
//  FileDXF.cpp
//  Polycam
//
//  Created by Chris Heinrich on 11/21/20.
//  Copyright © 2020 Chris Heinrich. All rights reserved.
//

/*
 This file contains a method to write point cloud to DXF file. It is loosely based off the FilePTS.cpp file that was in open3D
 The official Autodesk file spec for .dxf is here https://images.autodesk.com/adsk/files/autocad_2012_pdf_dxf-reference_enu.pdf
 Refer to the POINT group for the reference on writing points.

 We don't include a reader because we do not need to read point clouds in this format (they would be very uncommin anyway)

 We can potentially extend our writer to include more information like the color of the point cloud

 */

#include <Eigen/Dense>
#include <cstdio>
#include <map>

#include "open3d/io/FileFormatIO.h"
#include "open3d/io/PointCloudIO.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/FileSystem.h"
#include "open3d/utility/Helper.h"
#include "open3d/utility/ProgressReporters.h"

namespace open3d {
namespace io {

typedef Eigen::Vector3i Color;

// Autocad ACI format is an ad-hoc encoding of the color space into an 8 bit integer.
// The best thing we can do is a brute force lookup that attempts to match a given (r,g,b) to its closest
// counterpart in the ACI spectrum
std::vector<Color> getACIColorMap() {
  std::vector<Color> aci_color_map(256);
  aci_color_map[0] = Color(0, 0, 0);
  aci_color_map[1] = Color(255, 0, 0);
  aci_color_map[2] = Color(255, 255, 0);
  aci_color_map[3] = Color(0, 255, 0);
  aci_color_map[4] = Color(0, 255, 255);
  aci_color_map[5] = Color(0, 0, 255);
  aci_color_map[6] = Color(255, 0, 255);
  aci_color_map[7] = Color(255, 255, 255);
  aci_color_map[8] = Color(128, 128, 128);
  aci_color_map[9] = Color(192, 192, 192);
  aci_color_map[10] = Color(255, 0, 0);
  aci_color_map[11] = Color(255, 127, 127);
  aci_color_map[12] = Color(165, 0, 0);
  aci_color_map[13] = Color(165, 82, 82);
  aci_color_map[14] = Color(127, 0, 0);
  aci_color_map[15] = Color(127, 63, 63);
  aci_color_map[16] = Color(76, 0, 0);
  aci_color_map[17] = Color(76, 38, 38);
  aci_color_map[18] = Color(38, 0, 0);
  aci_color_map[19] = Color(38, 19, 19);
  aci_color_map[20] = Color(255, 63, 0);
  aci_color_map[21] = Color(255, 159, 127);
  aci_color_map[22] = Color(165, 41, 0);
  aci_color_map[23] = Color(165, 103, 82);
  aci_color_map[24] = Color(127, 31, 0);
  aci_color_map[25] = Color(127, 79, 63);
  aci_color_map[26] = Color(76, 19, 0);
  aci_color_map[27] = Color(76, 47, 38);
  aci_color_map[28] = Color(38, 9, 0);
  aci_color_map[29] = Color(38, 28, 19);
  aci_color_map[30] = Color(255, 127, 0);
  aci_color_map[31] = Color(255, 191, 127);
  aci_color_map[32] = Color(165, 82, 0);
  aci_color_map[33] = Color(165, 124, 82);
  aci_color_map[34] = Color(127, 63, 0);
  aci_color_map[35] = Color(127, 95, 63);
  aci_color_map[36] = Color(76, 38, 0);
  aci_color_map[37] = Color(76, 57, 38);
  aci_color_map[38] = Color(38, 19, 0);
  aci_color_map[39] = Color(38, 28, 19);
  aci_color_map[40] = Color(255, 191, 0);
  aci_color_map[41] = Color(255, 223, 127);
  aci_color_map[42] = Color(165, 124, 0);
  aci_color_map[43] = Color(165, 145, 82);
  aci_color_map[44] = Color(127, 95, 0);
  aci_color_map[45] = Color(127, 111, 63);
  aci_color_map[46] = Color(76, 57, 0);
  aci_color_map[47] = Color(76, 66, 38);
  aci_color_map[48] = Color(38, 28, 0);
  aci_color_map[49] = Color(38, 33, 19);
  aci_color_map[50] = Color(255, 255, 0);
  aci_color_map[51] = Color(255, 255, 127);
  aci_color_map[52] = Color(165, 165, 0);
  aci_color_map[53] = Color(165, 165, 82);
  aci_color_map[54] = Color(127, 127, 0);
  aci_color_map[55] = Color(127, 127, 63);
  aci_color_map[56] = Color(76, 76, 0);
  aci_color_map[57] = Color(76, 76, 38);
  aci_color_map[58] = Color(38, 38, 0);
  aci_color_map[59] = Color(38, 38, 19);
  aci_color_map[60] = Color(191, 255, 0);
  aci_color_map[61] = Color(223, 255, 127);
  aci_color_map[62] = Color(124, 165, 0);
  aci_color_map[63] = Color(145, 165, 82);
  aci_color_map[64] = Color(95, 127, 0);
  aci_color_map[65] = Color(111, 127, 63);
  aci_color_map[66] = Color(57, 76, 0);
  aci_color_map[67] = Color(66, 76, 38);
  aci_color_map[68] = Color(28, 38, 0);
  aci_color_map[69] = Color(33, 38, 19);
  aci_color_map[70] = Color(127, 255, 0);
  aci_color_map[71] = Color(191, 255, 127);
  aci_color_map[72] = Color(82, 165, 0);
  aci_color_map[73] = Color(124, 165, 82);
  aci_color_map[74] = Color(63, 127, 0);
  aci_color_map[75] = Color(95, 127, 63);
  aci_color_map[76] = Color(38, 76, 0);
  aci_color_map[77] = Color(57, 76, 38);
  aci_color_map[78] = Color(19, 38, 0);
  aci_color_map[79] = Color(28, 38, 19);
  aci_color_map[80] = Color(63, 255, 0);
  aci_color_map[81] = Color(159, 255, 127);
  aci_color_map[82] = Color(41, 165, 0);
  aci_color_map[83] = Color(103, 165, 82);
  aci_color_map[84] = Color(31, 127, 0);
  aci_color_map[85] = Color(79, 127, 63);
  aci_color_map[86] = Color(19, 76, 0);
  aci_color_map[87] = Color(47, 76, 38);
  aci_color_map[88] = Color(9, 38, 0);
  aci_color_map[89] = Color(23, 38, 19);
  aci_color_map[90] = Color(0, 255, 0);
  aci_color_map[91] = Color(125, 255, 127);
  aci_color_map[92] = Color(0, 165, 0);
  aci_color_map[93] = Color(82, 165, 82);
  aci_color_map[94] = Color(0, 127, 0);
  aci_color_map[95] = Color(63, 127, 63);
  aci_color_map[96] = Color(0, 76, 0);
  aci_color_map[97] = Color(38, 76, 38);
  aci_color_map[98] = Color(0, 38, 0);
  aci_color_map[99] = Color(19, 38, 19);
  aci_color_map[100] = Color(0, 255, 63);
  aci_color_map[101] = Color(127, 255, 159);
  aci_color_map[102] = Color(0, 165, 41);
  aci_color_map[103] = Color(82, 165, 103);
  aci_color_map[104] = Color(0, 127, 31);
  aci_color_map[105] = Color(63, 127, 79);
  aci_color_map[106] = Color(0, 76, 19);
  aci_color_map[107] = Color(38, 76, 47);
  aci_color_map[108] = Color(0, 38, 9);
  aci_color_map[109] = Color(19, 88, 23);
  aci_color_map[110] = Color(0, 255, 127);
  aci_color_map[111] = Color(127, 255, 191);
  aci_color_map[112] = Color(0, 165, 82);
  aci_color_map[113] = Color(82, 165, 124);
  aci_color_map[114] = Color(0, 127, 63);
  aci_color_map[115] = Color(63, 127, 95);
  aci_color_map[116] = Color(0, 76, 38);
  aci_color_map[117] = Color(38, 76, 57);
  aci_color_map[118] = Color(0, 38, 19);
  aci_color_map[119] = Color(19, 88, 28);
  aci_color_map[120] = Color(0, 255, 191);
  aci_color_map[121] = Color(127, 255, 223);
  aci_color_map[122] = Color(0, 165, 124);
  aci_color_map[123] = Color(82, 165, 145);
  aci_color_map[124] = Color(0, 127, 95);
  aci_color_map[125] = Color(63, 127, 111);
  aci_color_map[126] = Color(0, 76, 57);
  aci_color_map[127] = Color(38, 76, 66);
  aci_color_map[128] = Color(0, 38, 28);
  aci_color_map[129] = Color(19, 88, 88);
  aci_color_map[130] = Color(0, 255, 255);
  aci_color_map[131] = Color(127, 255, 255);
  aci_color_map[132] = Color(0, 165, 165);
  aci_color_map[133] = Color(82, 165, 165);
  aci_color_map[134] = Color(0, 127, 127);
  aci_color_map[135] = Color(63, 127, 127);
  aci_color_map[136] = Color(0, 76, 76);
  aci_color_map[137] = Color(38, 76, 76);
  aci_color_map[138] = Color(0, 38, 38);
  aci_color_map[139] = Color(19, 88, 88);
  aci_color_map[140] = Color(0, 191, 255);
  aci_color_map[141] = Color(127, 223, 255);
  aci_color_map[142] = Color(0, 124, 165);
  aci_color_map[143] = Color(82, 145, 165);
  aci_color_map[144] = Color(0, 95, 127);
  aci_color_map[145] = Color(63, 111, 217);
  aci_color_map[146] = Color(0, 57, 76);
  aci_color_map[147] = Color(38, 66, 126);
  aci_color_map[148] = Color(0, 28, 38);
  aci_color_map[149] = Color(19, 88, 88);
  aci_color_map[150] = Color(0, 127, 255);
  aci_color_map[151] = Color(127, 191, 255);
  aci_color_map[152] = Color(0, 82, 165);
  aci_color_map[153] = Color(82, 124, 165);
  aci_color_map[154] = Color(0, 63, 127);
  aci_color_map[155] = Color(63, 95, 127);
  aci_color_map[156] = Color(0, 38, 76);
  aci_color_map[157] = Color(38, 57, 126);
  aci_color_map[158] = Color(0, 19, 38);
  aci_color_map[159] = Color(19, 28, 88);
  aci_color_map[160] = Color(0, 63, 255);
  aci_color_map[161] = Color(127, 159, 255);
  aci_color_map[162] = Color(0, 41, 165);
  aci_color_map[163] = Color(82, 103, 165);
  aci_color_map[164] = Color(0, 31, 127);
  aci_color_map[165] = Color(63, 79, 127);
  aci_color_map[166] = Color(0, 19, 76);
  aci_color_map[167] = Color(38, 47, 126);
  aci_color_map[168] = Color(0, 9, 38);
  aci_color_map[169] = Color(19, 23, 88);
  aci_color_map[170] = Color(0, 0, 255);
  aci_color_map[171] = Color(127, 127, 255);
  aci_color_map[172] = Color(0, 0, 165);
  aci_color_map[173] = Color(82, 82, 165);
  aci_color_map[174] = Color(0, 0, 127);
  aci_color_map[175] = Color(63, 63, 127);
  aci_color_map[176] = Color(0, 0, 76);
  aci_color_map[177] = Color(38, 38, 126);
  aci_color_map[178] = Color(0, 0, 38);
  aci_color_map[179] = Color(19, 19, 88);
  aci_color_map[180] = Color(63, 0, 255);
  aci_color_map[181] = Color(159, 127, 255);
  aci_color_map[182] = Color(41, 0, 165);
  aci_color_map[183] = Color(103, 82, 165);
  aci_color_map[184] = Color(31, 0, 127);
  aci_color_map[185] = Color(79, 63, 127);
  aci_color_map[186] = Color(19, 0, 76);
  aci_color_map[187] = Color(47, 38, 126);
  aci_color_map[188] = Color(9, 0, 38);
  aci_color_map[189] = Color(23, 19, 88);
  aci_color_map[190] = Color(127, 0, 255);
  aci_color_map[191] = Color(191, 127, 255);
  aci_color_map[192] = Color(165, 0, 82);
  aci_color_map[193] = Color(124, 82, 165);
  aci_color_map[194] = Color(63, 0, 127);
  aci_color_map[195] = Color(95, 63, 127);
  aci_color_map[196] = Color(38, 0, 76);
  aci_color_map[197] = Color(57, 38, 126);
  aci_color_map[198] = Color(19, 0, 38);
  aci_color_map[199] = Color(28, 19, 88);
  aci_color_map[200] = Color(191, 0, 255);
  aci_color_map[201] = Color(223, 127, 255);
  aci_color_map[202] = Color(124, 0, 165);
  aci_color_map[203] = Color(142, 82, 165);
  aci_color_map[204] = Color(95, 0, 127);
  aci_color_map[205] = Color(111, 63, 127);
  aci_color_map[206] = Color(57, 0, 76);
  aci_color_map[207] = Color(66, 38, 76);
  aci_color_map[208] = Color(28, 0, 38);
  aci_color_map[209] = Color(88, 19, 88);
  aci_color_map[210] = Color(255, 0, 255);
  aci_color_map[211] = Color(255, 127, 255);
  aci_color_map[212] = Color(165, 0, 165);
  aci_color_map[213] = Color(165, 82, 165);
  aci_color_map[214] = Color(127, 0, 127);
  aci_color_map[215] = Color(127, 63, 127);
  aci_color_map[216] = Color(76, 0, 76);
  aci_color_map[217] = Color(76, 38, 76);
  aci_color_map[218] = Color(38, 0, 38);
  aci_color_map[219] = Color(88, 19, 88);
  aci_color_map[220] = Color(255, 0, 191);
  aci_color_map[221] = Color(255, 127, 223);
  aci_color_map[222] = Color(165, 0, 124);
  aci_color_map[223] = Color(165, 82, 145);
  aci_color_map[224] = Color(127, 0, 95);
  aci_color_map[225] = Color(127, 63, 111);
  aci_color_map[226] = Color(76, 0, 57);
  aci_color_map[227] = Color(76, 38, 66);
  aci_color_map[228] = Color(38, 0, 28);
  aci_color_map[229] = Color(88, 19, 88);
  aci_color_map[230] = Color(255, 0, 127);
  aci_color_map[231] = Color(255, 127, 191);
  aci_color_map[232] = Color(165, 0, 82);
  aci_color_map[233] = Color(165, 82, 124);
  aci_color_map[234] = Color(127, 0, 63);
  aci_color_map[235] = Color(127, 63, 95);
  aci_color_map[236] = Color(76, 0, 38);
  aci_color_map[237] = Color(76, 38, 57);
  aci_color_map[238] = Color(38, 0, 19);
  aci_color_map[239] = Color(88, 19, 28);
  aci_color_map[240] = Color(255, 0, 63);
  aci_color_map[241] = Color(255, 127, 159);
  aci_color_map[242] = Color(165, 0, 41);
  aci_color_map[243] = Color(165, 82, 103);
  aci_color_map[244] = Color(127, 0, 31);
  aci_color_map[245] = Color(127, 63, 79);
  aci_color_map[246] = Color(76, 0, 19);
  aci_color_map[247] = Color(76, 38, 47);
  aci_color_map[248] = Color(38, 0, 9);
  aci_color_map[249] = Color(88, 19, 23);
  aci_color_map[250] = Color(0, 0, 0);
  aci_color_map[251] = Color(101, 101, 101);
  aci_color_map[252] = Color(102, 102, 102);
  aci_color_map[253] = Color(153, 153, 153);
  aci_color_map[254] = Color(204, 204, 204);
  aci_color_map[255] = Color(255, 255, 255);
  return aci_color_map;
}

// We define this in the main namespace so we only have to build this dictionary once
std::vector<Color> ColorMap = getACIColorMap();

// maps an rgb color to an autocad color key
int rgbToAutocadColor(const Color &color) {
  int closest_match = 0;
  int distance = 10000;
  for (int i = 0; i < 256; i++) {
    const Color nextColor = ColorMap[i];
    const int c1 = color[0] - nextColor(0);
    const int c2 = color[1] - nextColor(1);
    const int c3 = color[2] - nextColor(2);
    const int nextDistance = c1 * c1 + c2 * c2 + c3 * c3;
    if (nextDistance < distance) {
      closest_match = i;
      distance = nextDistance;
    }
  }
  return closest_match;
};

int rgbToAutocadColor(const uint8_t &red, const uint8_t &green, const uint8_t &blue) { return rgbToAutocadColor(Color(red, green, blue)); }

bool WritePointCloudToDXF(const std::string &filename, const geometry::PointCloud &pointcloud, const WritePointCloudOption &params) {
  try {
    utility::filesystem::CFile file;
    if (!file.Open(filename, "w")) {
      utility::LogWarning("Write DXF failed: unable to open file: {}", filename);
      return false;
    }

    // Note CH: ARKit uses the y-axis for gravity, but most point cloud readers (including CAD programs) expect z-axis for gravity
    bool swap_y_and_z = false;
    if (swap_y_and_z) {
      std::cout << "Swapping y and z axis for DXF file" << std::endl;
    }

    utility::CountingProgressReporter reporter(params.update_progress);
    reporter.SetTotal(pointcloud.points_.size());

    // Write file header
    if (fprintf(file.GetFILE(), "999\nCreated by Polycam\n  0\nSECTION\n  2\nENTITIES\n  0\n") < 0) {
      utility::LogWarning("Write DXF failed: unable to write file: {}", filename);
      return false;
    }
    // Write points as POINT entities
    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
      const auto &point = pointcloud.points_[i];
      double x = point(0);
      double y = point(1);
      double z = point(2);
      if (swap_y_and_z) {
        z = point(1);
        y = -point(2);  // a minus sign is needed to keep this a right-handed coord system
      }

      if (!pointcloud.HasColors()) {
        if (fprintf(file.GetFILE(), "POINT\n  8\nCLOUD_001\n  6\nBYLAYER\n  10\n%.10f\n  20\n%.10f\n  30\n%.10f\n  0\n", x, y, z) < 0) {
          utility::LogWarning("Write DXF failed: unable to write file: {}", filename);
          return false;
        }
      } else {
        const auto EigenColor = utility::ColorToUint8(pointcloud.colors_[i]);
        const Color color(EigenColor[0], EigenColor[1], EigenColor[2]);
        int aci_color_code = rgbToAutocadColor(color);
        if (fprintf(file.GetFILE(), "POINT\n  8\nCLOUD_001\n  6\nBYLAYER\n  10\n%.10f\n  20\n%.10f\n  30\n%.10f\n  62\n%d\n  0\n", x, y, z,
                    aci_color_code) < 0) {
          utility::LogWarning("Write DXF failed: unable to write file: {}", filename);
          return false;
        }
      }
      if (i % 1000 == 0) {
        reporter.Update(i);
      }
    }
    // Write file footer
    if (fprintf(file.GetFILE(), "ENDSEC\n  0\nEOF") < 0) {
      utility::LogWarning("Write DXF failed: unable to write file: {}", filename);
      return false;
    }
    reporter.Finish();
    return true;
  } catch (const std::exception &e) {
    utility::LogWarning("Write PTS failed with exception: {}", e.what());
    return false;
  }
}

}  // namespace io
}  // namespace open3d

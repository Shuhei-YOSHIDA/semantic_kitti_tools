/**
 * @file param.hpp
 */


#ifndef INCLUDE_SEMANTIC_KITTI_TOOLS_PARAM_H
#define INCLUDE_SEMANTIC_KITTI_TOOLS_PARAM_H

#include <cstdint>
#include <map>
#include <array>

namespace semantic_kitti_tools
{
std::map<uint16_t, std::array<int, 3>> color_map{
//  0 : "unlabeled"
//  1 : "outlier"
//  10: "car"
//  11: "bicycle"
//  13: "bus"
//  15: "motorcycle"
//  16: "on-rails"
//  18: "truck"
//  20: "other-vehicle"
//  30: "person"
//  31: "bicyclist"
//  32: "motorcyclist"
//  40: "road"
//  44: "parking"
//  48: "sidewalk"
//  49: "other-ground"
//  50: "building"
//  51: "fence"
//  52: "other-structure"
//  60: "lane-marking"
//  70: "vegetation"
//  71: "trunk"
//  72: "terrain"
//  80: "pole"
//  81: "traffic-sign"
//  99: "other-object"
//  252: "moving-car"
//  253: "moving-bicyclist"
//  254: "moving-person"
//  255: "moving-motorcyclist"
//  256: "moving-on-rails"
//  257: "moving-bus"
//  258: "moving-truck"
//  259: "moving-other-vehicle"
  {0 , {0, 0, 0}},
  {1 , {0, 0, 255}},
  {10, {245, 150, 100}},
  {11, {245, 230, 100}},
  {13, {250, 80, 100}},
  {15, {150, 60, 30}},
  {16, {255, 0, 0}},
  {18, {180, 30, 80}},
  {20, {255, 0, 0}},
  {30, {30, 30, 255}},
  {31, {200, 40, 255}},
  {32, {90, 30, 150}},
  {40, {255, 0, 255}},
  {44, {255, 150, 255}},
  {48, {75, 0, 75}},
  {49, {75, 0, 175}},
  {50, {0, 200, 255}},
  {51, {50, 120, 255}},
  {52, {0, 150, 255}},
  {60, {170, 255, 150}},
  {70, {0, 175, 0}},
  {71, {0, 60, 135}},
  {72, {80, 240, 150}},
  {80, {150, 240, 255}},
  {81, {0, 0, 255}},
  {99, {255, 255, 50}},
  {252, {245, 150, 100}},
  {256, {255, 0, 0}},
  {253, {200, 40, 255}},
  {254, {30, 30, 255}},
  {255, {90, 30, 150}},
  {257, {250, 80, 100}},
  {258, {180, 30, 80}},
  {259, {255, 0, 0}}
};



} // namespace semantic_kitti_tools

#endif

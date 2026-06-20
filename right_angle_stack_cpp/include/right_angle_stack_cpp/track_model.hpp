#ifndef RIGHT_ANGLE_STACK_CPP__TRACK_MODEL_HPP_
#define RIGHT_ANGLE_STACK_CPP__TRACK_MODEL_HPP_

#include <string>
#include <vector>
#include <tuple>

// 锥桶信息：颜色 + 世界坐标
struct ConeInfo
{
  std::string color;
  double x, y, z;
};

// 从 SDF 文件解析锥桶列表（提取 <include> 标签中的 uri/name/pose）
std::vector<ConeInfo> load_cones_from_sdf(const std::string & path);

#endif  // RIGHT_ANGLE_STACK_CPP__TRACK_MODEL_HPP_
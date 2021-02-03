#ifndef _RS_MAP_MANAGER_HPP_
#define _RS_MAP_MANAGER_HPP_

#include <ryusei/common/defs.hpp>

namespace project_ryusei
{

class MapManager
{
public:
  MapManager();
  ~MapManager();
  bool init(const std::string &conf_path);
  bool loadMap(const std::string &path);
  bool saveMap(const std::string &path, const LocalMap &map);
  void getLocalMap(const Pose2D &pose, LocalMap &map);
  void updateMap(const Pose2D &pose, const cv::Point3f &points, LocalMap &map);
  void downsamplePoints(const Pose2D &pose, const std::vector<cv::Point3f> &points, LocalMap &map_out, 
                        const double &culling_distance);
  std::vector<cv::Point3f> pcl_points_, points_abs_, points_rel_;
  std::vector<std::vector<std::vector<cv::Point3f>>> block_;
  float points_rel_x_, points_rel_y_, points_rel_z_;
  double MAP_RANGE_;
private:
  cv::Point3f cvtAbsToRel(const Pose2D &pose, const double cos_val, const double sin_val,
                const cv::Point3f &points);
  cv::Point3f cvtRelToAbs(const Pose2D &pose, const double cos_val, const double sin_val,
                const cv::Point3f &points);
};
}

#endif
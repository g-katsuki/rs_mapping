#define _USE_MATH_DEFINES

#include <ryusei/mapping/map_manager.hpp>
#include <boost/timer.hpp>

using namespace project_ryusei;
using namespace std;

int main(int argc, char **argv)
{
  if(argc < 5)
  {
    cout << "Usage is... " << argv[0] << " [config file] [path to map] [path to save folder] [x] [y] [yaw]" << endl;
    return -1;
  }

  MapManager manager;
  if(!manager.init(argv[1]))
  {
    cout << "Failed to load config file" << endl;
    return -1;
  }

  if(!manager.loadMap(argv[2]))
  {
    cout << "Failed to load map" << endl;
    return -1;
  }

  LocalMap map, downsample_map;
  manager.getLocalMap(Pose2D(stod(argv[4]), stod(argv[5]), stod(argv[6]) * M_PI / 180.0), map);
  /*** 時間計測の開始 ***/
  boost::timer t;
  manager.downsamplePoints(map.pose, map.points, downsample_map, 1.0);
  /*** 経過時間を表示 ***/
  cout << "Elapsed time : " << t.elapsed() << "[s]" << endl;

  if(!manager.saveMap(argv[3], downsample_map))
  {
    cout << "Failed to save map" << endl;
    return -1;
  }

  return 0;
}
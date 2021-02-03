#include <ryusei/mapping/map_manager.hpp>
#include <iostream>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

using namespace project_ryusei;
using namespace std;
namespace prop = boost::property_tree;

/***************************************
* Public methods
***************************************/

MapManager::MapManager()
{

}

MapManager::~MapManager()
{

}

/**************************************
* iniファイルの読み込みを行うメソッド
***************************************/
bool MapManager::init(const std::string &conf_path)
{
  prop::ptree pt;
  prop::read_ini(conf_path, pt);
  /*** iniファイルからパラメータを読み込み ***/
  if(auto v = pt.get_optional<double>("MapManager.Range")) MAP_RANGE_ = v.get();

  return true;
}

/******************************************************************
* PCLのPCDファイルを読み込みクラス変数として格納するためのメソッド.
* 決められた区間(block)ごとに分けて点群を格納することで点群にアクセス
* する際の処理時間を削減する.
*******************************************************************/
bool MapManager::loadMap(const string &path)
{
  ifstream ifs(path);
  if(ifs.fail()){
    cerr << "Failed to load " << path << endl;
    return false;
  }

  /*** 読み込んだ座標をpcl_points_に格納 ***/
  string line;
  vector<string> items(3);
  while(getline(ifs, line)){
    if(isdigit(line[0]) == 1 || line[0] == '-'){
      stringstream ss;
      ss << line;
      ss >> items[0] >> items[1] >> items[2];
      pcl_points_.push_back(cv::Point3f(stof(items[0]), stof(items[1]), stof(items[2])));
    }
  }

  /*** 3次元のvectorを用意 ***/
  block_.resize(10);
  for(int i = 0; i < 9; i++){
    block_[i].resize(10);
  }

  /*** 適当な座標で区切ったvectorに点群を格納 ***/
  int ref_x, ref_y;
  for(int i = 0, size = pcl_points_.size(); i < size; i++){
      int ref_x = (pcl_points_[i].x + (MAP_RANGE_ * 0.8)) / (MAP_RANGE_ * 0.2);
    int ref_y = (pcl_points_[i].y + (MAP_RANGE_ * 0.8)) / (MAP_RANGE_ * 0.2);
    block_[ref_x][ref_y].push_back(cv::Point3f(pcl_points_[i]));
  }

  ifs.close();

  return true;
}

/****************************************************************
* 現在管理しているマップを引数に与えられたファイルに保存するメソッド
*****************************************************************/
bool MapManager::saveMap(const std::string &path, const LocalMap &map)
{
  ofstream ofs(path);

  if(ofs.fail()){
    cerr << "Failed to load save location" << path << endl;
    return false;
  }

  /*** 1行ずつファイルに書き込む ***/
  for(int i = 0, size = map.points.size(); i < size; i++){
    ofs << map.points[i].x << " " << map.points[i].y << " " << map.points[i].z << endl;
  }

	ofs.close();

  return true;
}

/************************************************
* 引数で与えられた位置情報の周囲100mの
* 点群情報(LocalMap型)を読み込み済みの
* マップから抽出し引数のmapへ格納するメソッド
*************************************************/
void MapManager::getLocalMap(const Pose2D &pose, LocalMap &map)
{
  map.pose = pose;
  double cos_val = cos(pose.yaw);
  double sin_val = sin(pose.yaw);
  /*** poseとの位置関係から,距離を測りたいblockだけに絞る ***/
  int ref_pose_x = (pose.x + (MAP_RANGE_ * 0.8)) / (MAP_RANGE_ * 0.2);
  int ref_pose_y = (pose.y + (MAP_RANGE_ * 0.8)) / (MAP_RANGE_ * 0.2);
  int points_size = 0, points_cnt = 0;

  for(int i = 0; i < 9; i++){
    for(int j = 0; j < 9; j++){    // map.pointsのサイズを事前に決めるためにblock_のサイズを合計する
      if(i <= ref_pose_x+2  && i >= ref_pose_x-2 && j <= ref_pose_y+2 && j >= ref_pose_y-2){
        points_size += block_[i][j].size();
      }
    }
  }

  map.points.resize(points_size);
  for(int i = 0; i < 9; i++){
    for(int j = 0; j < 9; j++){
      if(i <= ref_pose_x+2 && i >= ref_pose_x-2 && j <= ref_pose_y+2 && j >= ref_pose_y-2){
        for(int k = 0, size = block_[i][j].size(); k < size; k++){
          /*** 絶対座標から相対座標に変換しつつ代入 ***/
          map.points[points_cnt] = MapManager::cvtAbsToRel(pose, cos_val, sin_val, block_[i][j][k]);
          points_cnt++;
        }
      }
    }
  }

}

/************************************************************
* ローカル座標(LiDARから取得されたままのデータ)で渡された点群を
* 現在管理しているマップに追加するメソッド
*************************************************************/
void MapManager::updateMap(const Pose2D &pose, const cv::Point3f &points, LocalMap &map)
{
  double cos_val = cos(pose.yaw);
  double sin_val = sin(pose.yaw);
  map.points.push_back(MapManager::cvtRelToAbs(pose, cos_val, sin_val, points));
}

/********************************************************
* マップ全体の点群をダウンサンプリングするメソッド.
* culling_distanceで決めた距離以下の点同士を重ねて1つの点に
* することで点群数を減らす.
*********************************************************/
void MapManager::downsamplePoints(const Pose2D &pose, const vector<cv::Point3f> &points, LocalMap &map_out,
                                  const double &culling_distance)
{
  map_out.pose = Pose2D(pose.x, pose.y, pose.yaw);
  int x, y, points_cnt = 0;
  map_out.points.resize(points.size());
  float x_out, y_out;
  
  /*** 既に書き込まれた座標を被らないようにMat型で管理(要素が多すぎると配列が使えない為) ***/
  int double_ups_size = (int)(2*MAP_RANGE_ / culling_distance);
  cv::Mat double_ups(double_ups_size, double_ups_size, CV_8UC1, cv::Scalar(0));

  /*** roundを使って近い座標同士をまとめあげて点群数を減らす ***/
  for(int i = 0, size = points.size(); i < size ; i++){
    x = (double_ups_size/2) + round(points[i].x / culling_distance);
    y = (double_ups_size/2) + round(points[i].y / culling_distance);
    double_ups.at<uchar>(x,y) = 200;
  }

  for(int x = 0, size = double_ups_size; x < size; x++){
    for(int y = 0, size = double_ups_size; y < size; y++){
      if(double_ups.at<char>(x,y)){
        x_out = (x - double_ups_size/2) * culling_distance;
        y_out = (y - double_ups_size/2) * culling_distance;
        map_out.points[points_cnt] = cv::Point3f(x_out, y_out, 0);
        points_cnt++;
      }
    }
  }

  /*** 余った領域を削除 ***/
  map_out.points.erase(map_out.points.begin() + points_cnt, map_out.points.end());
  cout << "points_size: " << map_out.points.size() << endl;
}

/***************************************
* Private methods
***************************************/

/*****************************************
* 絶対座標から相対座標へ変換するメソッド
******************************************/
cv::Point3f MapManager::cvtAbsToRel(const Pose2D &pose, const double cos_val,
                          const double sin_val, const cv::Point3f &points)
{
  cv::Point3f rel;
  rel.x = (points.x - pose.x) * cos_val + (points.y - pose.y) * sin_val;
  rel.y = (-1) * (points.x - pose.x) * sin_val + (points.y - pose.y) * cos_val;
  rel.z = points.z;

  return rel;
}

/*****************************************
* 相対座標から絶対座標に変換するメソッド
******************************************/
cv::Point3f MapManager::cvtRelToAbs(const Pose2D &pose, const double cos_val,
                          const double sin_val, const cv::Point3f &points)
{
  cv::Point3f abs;
  abs.x = (points.x * cos_val) - (points.y * sin_val) + pose.x;
  abs.y = (points.y * cos_val) + (points.x * sin_val) + pose.y;
  abs.z = points.z;
  
  return abs;
}

#ifndef MULTI_SENSORS_OBJECTDETECTION_DATAFUSION_H
#define MULTI_SENSORS_OBJECTDETECTION_DATAFUSION_H
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <ros/ros.h>

using namespace cv;
using namespace std;

const static float TIME_GAP = 0.1;  // second

enum class CameraFLAG
{
  CAMERA_LEFT = 0,
  CAMERA_RIGHT = 1,
  CAMERA_BOTH
};

class DataFusion
{
public:
  DataFusion();

  virtual ~DataFusion();

  //bool Init(const ros::NodeHandle &nh_param);

  void ProjectLidarPoints(const double lidar_timestamp, const pcl::PointCloud<pcl::PointXYZ> &lidar, CameraFLAG flag = CameraFLAG::CAMERA_LEFT);

  // 通过图像区域ROI 返回该区域内对应的Lidar三维坐标
  bool GetLidarPointsByImageROI(const double time_stamp,
                                const cv::Rect &roi,
                                CameraFLAG flag,
                                std::vector<cv::Point3d> &lidar_points,
                                std::vector<cv::Point2i> &uv_points);

  //bool GetLidarPointsByImageROI(const double time_stamp, const std::vector<cv::Rect> &rois);

  // 返回点云图像融合结果
  void GetFusionDebug(const cv::Mat& src_img, cv::Mat &fusion_img, CameraFLAG flag = CameraFLAG::CAMERA_LEFT);

private:
  // 过滤相机视野外的点云
  void LidarPointFillter(const pcl::PointCloud<pcl::PointXYZ> &lidar, std::vector<size_t> &reserved_pt_id);

  // loacal data
private:
  double lidar_time_;
  std::vector<cv::Point2d> uv_points_l_; // 激光点投影到左目后对应的像素坐标（注：肯定存在超出图像范围的点，注意判断）
  std::vector<cv::Point2d> uv_points_r_;

  std::vector<cv::Point3d> lidar_points_; // 原始激光点

  // init param
private:
  bool b_init_ = false;

  int image_width_;
  int image_height_;

  cv::Mat rvec_left_;
  cv::Mat tvec_left_;
  cv::Mat camera_matrix_left_;

  cv::Mat rvec_right_;
  cv::Mat tvec_right_;
  cv::Mat camera_matrix_right_;
};

#endif
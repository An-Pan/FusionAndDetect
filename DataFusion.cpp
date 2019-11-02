#include "DataFusion.h"
//
//bool DataFusion::Init(const ros::NodeHandle &nh_param)
//{
//      bool b_res = true;
//
//      nh_param.param("image_width", this->image_width_, 4096);
//      nh_param.param("image_height", this->image_height_, 3000);
//
//      double fx_l, fy_l, cx_l, cy_l;
//      double fx_r, fy_r, cx_r, cy_r;
//
//      // 解析左相机内参数
//      if (!nh_param.hasParam("fx_l"))
//      {
//            cout << "can not find param fx_l" << endl;
//            return false;
//      }
//      nh_param.getParam("fx_l", fx_l);
//
//      if (!nh_param.hasParam("fy_l"))
//      {
//            cout << "can not find param fy_l" << endl;
//            return false;
//      }
//      nh_param.getParam("fy_l", fy_l);
//
//      if (!nh_param.hasParam("cx_l"))
//      {
//            cout << "can not find param cx_l" << endl;
//            return false;
//      }
//      nh_param.getParam("cx_l", cx_l);
//
//      if (!nh_param.hasParam("cy_l"))
//      {
//            cout << "can not find param cy_l" << endl;
//            return false;
//      }
//      nh_param.getParam("cy_l", cy_l);
//
//      ////////////////////////////////////////////////////////
//      // 解析右相机内参数
//      if (!nh_param.hasParam("fx_r"))
//      {
//            cout << "can not find param fx_r" << endl;
//            return false;
//      }
//      nh_param.getParam("fx_r", fx_r);
//
//      if (!nh_param.hasParam("fy_r"))
//      {
//            cout << "can not find param fy_r" << endl;
//            return false;
//      }
//      nh_param.getParam("fy_r", fy_r);
//
//      if (!nh_param.hasParam("cx_r"))
//      {
//            cout << "can not find param cx_r" << endl;
//            return false;
//      }
//      nh_param.getParam("cx_r", cx_r);
//
//      if (!nh_param.hasParam("cy_r"))
//      {
//            cout << "can not find param cy_r" << endl;
//            return false;
//      }
//      nh_param.getParam("cy_r", cy_r);
//
//      this->camera_matrix_left_ = (cv::Mat_<double>(3, 3) << fx_l, 0, cx_l, 0.0, fy_l, cy_l, 0.0, 0.0, 1.0);
//
//      this->camera_matrix_right_ = (cv::Mat_<double>(3, 3) << fx_r, 0, cx_r, 0.0, fy_r, cy_r, 0.0, 0.0, 1.0);
//
//      /////////////////////////////////////////////////////////////////////////
//      double rvecx_l, rvecy_l, rvecz_l;
//
//      double rvecx_r, rvecy_r, rvecz_r;
//
//      // 解析左相机旋转向量
//      if (!nh_param.hasParam("rvecx_l"))
//      {
//            cout << "can not find param rvecx_l" << endl;
//            return false;
//      }
//      nh_param.getParam("rvecx_l", rvecx_l);
//
//      if (!nh_param.hasParam("rvecy_l"))
//      {
//            cout << "can not find param rvecy_l" << endl;
//            return false;
//      }
//      nh_param.getParam("rvecy_l", rvecy_l);
//
//      if (!nh_param.hasParam("rvecz_l"))
//      {
//            cout << "can not find param rvecz_l" << endl;
//            return false;
//      }
//      nh_param.getParam("rvecz_l", rvecz_l);
//
//      this->rvec_left_ = (cv::Mat_<double>(3, 1) << rvecx_l, rvecy_l, rvecz_l);
//
//      // 解析右相机旋转向量
//      if (!nh_param.hasParam("rvecx_r"))
//      {
//            cout << "can not find param rvecx_r" << endl;
//            return false;
//      }
//      nh_param.getParam("rvecx_r", rvecx_r);
//
//      if (!nh_param.hasParam("rvecy_r"))
//      {
//            cout << "can not find param rvecy_r" << endl;
//            return false;
//      }
//      nh_param.getParam("rvecy_r", rvecy_r);
//
//      if (!nh_param.hasParam("rvecz_r"))
//      {
//            cout << "can not find param rvecz_r" << endl;
//            return false;
//      }
//      nh_param.getParam("rvecz_r", rvecz_r);
//
//      this->rvec_right_ = (cv::Mat_<double>(3, 1) << rvecx_r, rvecy_r, rvecz_r);
//
//      ///////////////////////////////////////////////////////////////////////////
//      double tvecx_l, tvecy_l, tvecz_l;
//
//      double tvecx_r, tvecy_r, tvecz_r;
//
//      // 解析左相机平移向量
//      if (!nh_param.hasParam("tvecx_l"))
//      {
//            cout << "can not find param tvecx_l" << endl;
//            return false;
//      }
//      nh_param.getParam("tvecx_l", tvecx_l);
//
//      if (!nh_param.hasParam("tvecy_l"))
//      {
//            cout << "can not find param tvecy_l" << endl;
//            return false;
//      }
//      nh_param.getParam("tvecy_l", tvecy_l);
//
//      if (!nh_param.hasParam("tvecz_l"))
//      {
//            cout << "can not find param tvecz_l" << endl;
//            return false;
//      }
//      nh_param.getParam("tvecz_l", tvecz_l);
//
//      this->tvec_left_ = (cv::Mat_<double>(3, 1) << tvecx_l, tvecy_l, tvecz_l);
//
//      // 解析右相机平移向量
//      if (!nh_param.hasParam("tvecx_r"))
//      {
//            cout << "can not find param tvecx_r" << endl;
//            return false;
//      }
//      nh_param.getParam("tvecx_r", tvecx_r);
//
//      if (!nh_param.hasParam("tvecy_r"))
//      {
//            cout << "can not find param tvecy_r" << endl;
//            return false;
//      }
//      nh_param.getParam("tvecy_r", tvecy_r);
//
//      if (!nh_param.hasParam("tvecz_r"))
//      {
//            cout << "can not find param tvecz_r" << endl;
//            return false;
//      }
//      nh_param.getParam("tvecz_r", tvecz_r);
//
//      this->tvec_right_ = (cv::Mat_<double>(3, 1) << tvecx_r, tvecy_r, tvecz_r);
//
//      this->b_init_ = true;
//
//      return true;
//}

void DataFusion::LidarPointFillter(const pcl::PointCloud<pcl::PointXYZ> &lidar, std::vector<size_t> &reserved_pt_id)
{
      reserved_pt_id.reserve(lidar.size() / 2);

      for (size_t i = 0; i < lidar.size(); i++)
      {

            if (lidar[i].y < 0 || lidar[i].y > 0)
            {
                  continue;
            }

            if (abs(lidar[i].x) > 200)
            {
                  continue;
            }

            if (abs(lidar[i].z) > 200)
            {
                  continue;
            }

            reserved_pt_id.emplace_back(i);
      }
}

void DataFusion::ProjectLidarPoints(const double lidar_timestamp, const pcl::PointCloud<pcl::PointXYZ> &lidar, CameraFLAG flag)
{
      this->lidar_time_ = lidar_timestamp;

      std::vector<size_t> reserved_pt_id;

      LidarPointFillter(lidar, reserved_pt_id);

      lidar_points_.clear();
      lidar_points_.reserve(reserved_pt_id.size());

      for (size_t i = 0; i < reserved_pt_id.size(); i++)
      {
            size_t index = reserved_pt_id[i];
            lidar_points_.emplace_back(cv::Point3d(lidar[index].x, lidar[index].y, lidar[index].z));
      }

      switch (flag)
      {
      case CameraFLAG::CAMERA_LEFT:
      {
            cv::projectPoints(lidar_points_, this->rvec_left_, this->tvec_left_, this->camera_matrix_left_, cv::Mat(), this->uv_points_l_);
      }
      break;
      case CameraFLAG::CAMERA_RIGHT:
      {
            cv::projectPoints(lidar_points_, this->rvec_right_, this->tvec_right_, this->camera_matrix_right_, cv::Mat(), this->uv_points_r_);
      }
      break;
      case CameraFLAG::CAMERA_BOTH:
      {
            cv::projectPoints(lidar_points_, this->rvec_left_, this->tvec_left_, this->camera_matrix_left_, cv::Mat(), this->uv_points_l_);
            cv::projectPoints(lidar_points_, this->rvec_right_, this->tvec_right_, this->camera_matrix_right_, cv::Mat(), this->uv_points_r_);
      }
      break;

      default:
            break;
      }
}

bool DataFusion::GetLidarPointsByImageROI(const double time_stamp,
                                          const cv::Rect &roi,
                                          CameraFLAG flag,
                                          std::vector<cv::Point3d> &lidar_points,
                                          std::vector<cv::Point2i> &uv_points)
{
      if (flag == CameraFLAG::CAMERA_BOTH)
      {
            cout << "Do not support CAMERA_BOTH option" << endl;
            return false;
      }

      if (abs(lidar_time_ - time_stamp) > TIME_GAP)
      {
            cout << "Lidar-time and Image-time unmatch" << endl;
            return false;
      }

      if (flag == CameraFLAG::CAMERA_LEFT)
      {
            if (uv_points_l_.size() != lidar_points_.size())
            {
                  cout << "points size unmatch" << endl;
                  return false;
            }

            uv_points.reserve(uv_points_l_.size());
            lidar_points.reserve(uv_points_l_.size());
            for (int i = 0; i < uv_points_l_.size(); i++)
            {
                  int u = uv_points_l_[i].x;
                  int v = uv_points_l_[i].y;
                  if (u < roi.x || u > (roi.x + roi.width - 1))
                  {
                        continue;
                  }
                  if (v < roi.y || v > (roi.y + roi.height - 1))
                  {
                        continue;
                  }
                  uv_points.emplace_back(cv::Point2i(u, v));
                  lidar_points.emplace_back(lidar_points_[i]);
            }
      }

      if (flag == CameraFLAG::CAMERA_RIGHT)
      {
            if (uv_points_r_.size() != lidar_points_.size())
            {
                  cout << "points size unmatch" << endl;
                  return false;
            }

            uv_points.reserve(uv_points_r_.size());
            lidar_points.reserve(uv_points_r_.size());
            for (int i = 0; i < uv_points_r_.size(); i++)
            {
                  int u = uv_points_r_[i].x;
                  int v = uv_points_r_[i].y;
                  if (u < roi.x || u > (roi.x + roi.width - 1))
                  {
                        continue;
                  }
                  if (v < roi.y || v > (roi.y + roi.height - 1))
                  {
                        continue;
                  }
                  uv_points.emplace_back(cv::Point2i(u, v));
                  lidar_points.emplace_back(lidar_points_[i]);
            }
      }

      if (uv_points.empty() || lidar_points.empty())
      {
            cout << "Results empty()" << endl;
            return false;
      }

      return true;
}

void DataFusion::GetFusionDebug(const cv::Mat &src_img, cv::Mat &fusion_img, CameraFLAG flag)
{
      src_img.copyTo(fusion_img);
      int width = image_width_ / 5.0;
      int height = image_height_ / 5.0;
      cv::resize(fusion_img, fusion_img, cv::Size(width, height));

      if (flag == CameraFLAG::CAMERA_LEFT)
      {
            for (int i = 0; i < uv_points_l_.size(); i++)
            {
                  int u = uv_points_l_[i].x / 5.0;
                  int v = uv_points_l_[i].y / 5.0;

                  if (u < 0 || u > width - 1 || v < 0 || v > height - 1)
                  {
                        continue;
                  }
                  cv::circle(fusion_img, cv::Point(u, v), 3, cv::Scalar(255, 0, 0), 2);
            }
      }
      else
      {
            for (int i = 0; i < uv_points_r_.size(); i++)
            {
                  int u = uv_points_r_[i].x / 5.0;
                  int v = uv_points_r_[i].y / 5.0;

                  if (u < 0 || u > width - 1 || v < 0 || v > height - 1)
                  {
                        continue;
                  }
                  cv::circle(fusion_img, cv::Point(u, v), 3, cv::Scalar(255, 0, 0), 2);
            }
      }
}
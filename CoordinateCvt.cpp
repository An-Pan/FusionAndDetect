//
// Created by ibd02 on 19-11-16.
//

#include "CoordinateCvt.h"



void CoordinateCvt::CvtIMU2World(const Eigen::Vector3d &src_pt, Eigen::Vector3d &dst_pt, const Pos &pos) {

    double roll = -pos.roll;
    double pitch = -pos.pitch;
    double yaw = pos.heading;

    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
            0, cos(pitch*PI / 180), sin(pitch*PI / 180),
            0, -sin(pitch*PI / 180), cos(pitch*PI / 180);

    Eigen::Matrix3d Ry;
    Ry << cos(roll*PI / 180), 0, -sin(roll*PI / 180),
            0, 1, 0,
            sin(roll*PI / 180), 0, cos(roll*PI / 180);

    Eigen::Matrix3d Rz;
    Rz << cos(yaw*PI / 180), sin(yaw*PI / 180), 0,
            -sin(yaw*PI / 180), cos(yaw*PI / 180), 0,
            0, 0, 1;

    Eigen::Vector3d T;
    T <<pos.t_x,pos.t_y,pos.t_z;


    dst_pt = Rz*Rx*Ry * src_pt;
    dst_pt += T;
}


void CoordinateCvt::CvtIMU2World(const cv::Point3d& src_pt,cv::Point3d& dst_pt,const Pos& pos){

    double roll = -pos.roll;
    double pitch = -pos.pitch;
    double yaw = pos.heading;

// pitch
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, cos(pitch*PI / 180), sin(pitch*PI / 180),
            0, -sin(pitch*PI / 180), cos(pitch*PI / 180));

    // row
    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(roll*PI / 180), 0, -sin(roll*PI / 180),
            0, 1, 0,
            sin(roll*PI / 180), 0, cos(roll*PI / 180));

    // yaw
    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw*PI / 180), sin(yaw*PI / 180), 0,
            -sin(yaw*PI / 180), cos(yaw*PI / 180), 0,
            0, 0, 1
    );

    cv::Mat T = (cv::Mat_<double>(3, 1) << pos.t_x,
            pos.t_y,
            pos.t_z);


    cv::Mat srcPoint = (cv::Mat_<double>(3, 1) << src_pt.x, src_pt.y, src_pt.z);
    cv::Mat dstPoint = Rz*Rx*Ry*srcPoint;

    dstPoint = dstPoint + T;

    dst_pt.x = dstPoint.at<double>(0, 0);
    dst_pt.y = dstPoint.at<double>(1, 0);
    dst_pt.z = dstPoint.at<double>(2, 0);
}

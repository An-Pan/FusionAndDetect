//
// Created by ibd02 on 19-11-16.
//

#ifndef OBJECTDETECT_COORDINATECVT_H
#define OBJECTDETECT_COORDINATECVT_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>


#define PI 3.1415926

using namespace cv;
using namespace std;

struct Pos
{
    double t_x;
    double t_y;
    double t_z;

    double pitch;
    double roll;
    double heading;

    Pos(double x,double y,double z,double p,double r,double h)
            :t_x(x),t_y(y),t_z(z),pitch(p),roll(r),heading(h){};
    Pos(){};


};

class CoordinateCvt {

public:
    CoordinateCvt(){};

    void CvtIMU2World(const Eigen::Vector3d& src_pt,Eigen::Vector3d& dst_pt,const Pos& pos);

    void CvtIMU2World(const cv::Point3d& src_pt,cv::Point3d& dst_pt,const Pos& pos);


};


#endif //OBJECTDETECT_COORDINATECVT_H

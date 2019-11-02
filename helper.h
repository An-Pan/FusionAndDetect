//
// Created by ibd02 on 19-10-7.
//

#ifndef OBJECTDETECT_HELPER_H
#define OBJECTDETECT_HELPER_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <shapefil.h>

using namespace std;

extern void CreateShpFile(const std::string file_path, const std::vector<cv::Point3d>& points);
extern int GetTrace(string path_to_dataset, vector<cv::Point3d> &pose);

#endif //OBJECTDETECT_HELPER_H

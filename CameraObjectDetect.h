//
// Created by ibd02 on 19-9-30.
//

#ifndef OBJECTDETECT_CAMERAOBJECTDETECT_H
#define OBJECTDETECT_CAMERAOBJECTDETECT_H

#include <string>
#include <opencv2/opencv.hpp>


#include "TrtNet.h"

using namespace std;
using namespace cv;

struct Bbox
{
    int classId;
    int left;
    int right;
    int top;
    int bot;
    float score;
};

class CameraObjectDetect {
public:
    CameraObjectDetect()  {};

    virtual ~CameraObjectDetect() {};

    bool Init(const std::string config_file);

    void DetectImage(const cv::Mat& frame);

    void ShowDebug(cv::Mat& img);
    cv::Rect GetDetectResult();

private:
    vector<float> PrepareImage(const cv::Mat& img);

    vector<Bbox> PostProcessImg(const cv::Mat& img,vector<Yolo::Detection>& detections,int classes);

    void DoNms(vector<Yolo::Detection>& detections,int classes ,float nmsThresh);

private:
    std::unique_ptr<Tn::trtNet> net_;


    vector<Bbox> bboxs_;
    //int class_num_ = 80;
    int class_num_ = 6;         // boat modle
    int w_ = 608;
    int h_ = 608;
    int c_ = 3;
    float nms_ = 0.45;
    float confidence_ = 0.7;

};


#endif //OBJECTDETECT_CAMERAOBJECTDETECT_H

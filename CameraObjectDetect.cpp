//
// Created by ibd02 on 19-9-30.
//

#include "CameraObjectDetect.h"
bool CameraObjectDetect::Init(const std::string engine_file)
{
    if(engine_file.empty())
        return false;

    net_.reset(new Tn::trtNet(engine_file));

    return true;
}

void CameraObjectDetect::DetectImage(const cv::Mat& frame)
{
  if(frame.empty()){
        return;
    }
    vector<float> inputData;
    inputData.reserve(h_*w_*c_*2);
    vector<float> cur_input = PrepareImage(frame);

    int outputCount = net_->getOutputSize()/sizeof(float);
    unique_ptr<float[]> outputData(new float[outputCount]);
    inputData.insert(inputData.end(), cur_input.begin(), cur_input.end());

    // batchCount == 1
    net_->doInference(inputData.data(), outputData.get(),1);

    auto output = outputData.get();
    auto outputSize = net_->getOutputSize()/ sizeof(float) / 1;

    int detCount = output[0];
    //later detect result
    vector<Yolo::Detection> result;
    result.resize(detCount);
    memcpy(result.data(), &output[1], detCount*sizeof(Yolo::Detection));

    vector<Yolo::Detection> result_filter;
    for(auto it:result){
        if(it.prob < confidence_)
            continue;
        result_filter.push_back(it);
    }

    this->bboxs_ = PostProcessImg(frame,result_filter,class_num_);
}

void CameraObjectDetect::ShowDebug(cv::Mat& img)
{
    for(const auto& item : this->bboxs_)
    {
        cv::rectangle(img,cv::Point(item.left,item.top),cv::Point(item.right,item.bot),cv::Scalar(0,0,255),3,8,0);
        //cout << "class=" << item.classId << " prob=" << item.score*100 << endl;
        //cout << "left=" << item.left << " right=" << item.right << " top=" << item.top << " bot=" << item.bot << endl;
    }

    cv::imwrite("result.jpg",img);
    //cv::waitKey(0);
}

cv::Rect CameraObjectDetect::GetDetectResult()
{
    for(const auto& item : this->bboxs_)
    {
        return cv::Rect(cv::Point(item.left,item.top),cv::Point(item.right,item.bot));
    }

}

vector<float> CameraObjectDetect::PrepareImage(const cv::Mat& img)
{
    using namespace cv;

    int c = c_;
    int h = h_;   //net h
    int w = w_;   //net w

    float scale = min(float(w)/img.cols,float(h)/img.rows);
    auto scaleSize = cv::Size(img.cols * scale,img.rows * scale);

    cv::Mat rgb ;
    cv::cvtColor(img, rgb, CV_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized,scaleSize,0,0,INTER_CUBIC);

    cv::Mat cropped(h, w,CV_8UC3, 127);
    Rect rect((w- scaleSize.width)/2, (h-scaleSize.height)/2, scaleSize.width,scaleSize.height);
    resized.copyTo(cropped(rect));

    cv::Mat img_float;
    if (c == 3)
        cropped.convertTo(img_float, CV_32FC3, 1/255.0);
    else
        cropped.convertTo(img_float, CV_32FC1 ,1/255.0);

    //HWC TO CHW
    vector<Mat> input_channels(c);
    cv::split(img_float, input_channels);

    vector<float> result(h*w*c);
    auto data = result.data();
    int channelLength = h * w;
    for (int i = 0; i < c; ++i) {
        memcpy(data,input_channels[i].data,channelLength*sizeof(float));
        data += channelLength;
    }

    return result;
}



vector<Bbox> CameraObjectDetect::PostProcessImg(const cv::Mat& img,vector<Yolo::Detection>& detections,int classes)
{
    using namespace cv;

    int h = h_;   //net h
    int w = w_;   //net w

    //scale bbox to img
    int width = img.cols;
    int height = img.rows;
    float scale = min(float(w)/width,float(h)/height);
    float scaleSize[] = {width * scale,height * scale};

    //correct box
    for (auto& item : detections)
    {
        auto& bbox = item.bbox;
        bbox[0] = (bbox[0] * w - (w - scaleSize[0])/2.f) / scaleSize[0];
        bbox[1] = (bbox[1] * h - (h - scaleSize[1])/2.f) / scaleSize[1];
        bbox[2] /= scaleSize[0];
        bbox[3] /= scaleSize[1];
    }

    //nms
    float nmsThresh = nms_;
    if(nmsThresh > 0)
        DoNms(detections,classes,nmsThresh);

    vector<Bbox> boxes;
    for(const auto& item : detections)
    {
        auto& b = item.bbox;
        Bbox bbox =
                {
                        item.classId,   //classId
                        max(int((b[0]-b[2]/2.)*width),0), //left
                        min(int((b[0]+b[2]/2.)*width),width), //right
                        max(int((b[1]-b[3]/2.)*height),0), //top
                        min(int((b[1]+b[3]/2.)*height),height), //bot
                        item.prob       //score
                };
        boxes.push_back(bbox);
    }

    return boxes;
}


void CameraObjectDetect::DoNms(vector<Yolo::Detection>& detections,int classes ,float nmsThresh)
{
    //auto t_start = chrono::high_resolution_clock::now();

    vector<vector<Yolo::Detection>> resClass;
    resClass.resize(classes);

    for (const auto& item : detections)
        resClass[item.classId].push_back(item);

    auto iouCompute = [](float * lbox, float* rbox)
    {
        float interBox[] = {
                max(lbox[0] - lbox[2]/2.f , rbox[0] - rbox[2]/2.f), //left
                min(lbox[0] + lbox[2]/2.f , rbox[0] + rbox[2]/2.f), //right
                max(lbox[1] - lbox[3]/2.f , rbox[1] - rbox[3]/2.f), //top
                min(lbox[1] + lbox[3]/2.f , rbox[1] + rbox[3]/2.f), //bottom
        };

        if(interBox[2] > interBox[3] || interBox[0] > interBox[1])
            return 0.0f;

        float interBoxS =(interBox[1]-interBox[0])*(interBox[3]-interBox[2]);
        return interBoxS/(lbox[2]*lbox[3] + rbox[2]*rbox[3] -interBoxS);
    };

    vector<Yolo::Detection> result;
    for (int i = 0;i<classes;++i)
    {
        auto& dets =resClass[i];
        if(dets.size() == 0)
            continue;

        sort(dets.begin(),dets.end(),[=](const Yolo::Detection& left,const Yolo::Detection& right){
            return left.prob > right.prob;
        });

        for (unsigned int m = 0;m < dets.size() ; ++m)
        {
            auto& item = dets[m];
            result.push_back(item);
            for(unsigned int n = m + 1;n < dets.size() ; ++n)
            {
                if (iouCompute(item.bbox,dets[n].bbox) > nmsThresh)
                {
                    dets.erase(dets.begin()+n);
                    --n;
                }
            }
        }
    }

    //swap(detections,result);
    detections = move(result);

    //auto t_end = chrono::high_resolution_clock::now();
    //float total = chrono::duration<float, milli>(t_end - t_start).count();
    //cout << "Time taken for nms is " << total << " ms." << endl;
}

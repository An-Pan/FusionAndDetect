#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "args_parser.h"
#include "configs.h"
#include "CameraObjectDetect.h"
#include "CoordinateCvt.h"

#include "Cluster.h"
#include "utils.h"

#include "helper.h"

using namespace std;

void readPCDfile(const std::string finname, std::vector<cv::Point3d>& points);

bool readGPS(const std::string filename, cv::Point3d& imu_pos);
bool readGPS(const std::string filename, Pos& imu_pos);

void PointQucikSort(const std::vector<DbscanPoint>& db_points, std::vector<size_t>& sort_index);

void PointQucikSort(const std::vector<cv::Point3d>& src_points, std::vector<size_t>& sort_index);

inline double sigmoid(double x)
{
    return (1.0 / (1.0 + exp(-x)));
}


void LidarObjectFilter(const cv::Rect& target_roi, const std::vector<cv::Point3d>& src_points,const std::vector<cv::Point2d>& uv_points, double& ave_depth)
{
    std::vector<size_t> sort_index;
    // 小于 10点 , 排序取中值
    if(src_points.size() < 10){

        PointQucikSort(src_points, sort_index);
        int mid_index = sort_index[sort_index.size() / 2 ];
        ave_depth = src_points[mid_index].y;

        return;
    }

    vector<DbscanPoint> db_points;

    for(int i = 0;i<src_points.size();i++){
        db_points.push_back(DbscanPoint(src_points[i].y,0,i));
    }

    dbscan(db_points, 1, 5);


    sort_index.reserve(db_points.size());
    // 按深度递增排序
    PointQucikSort(db_points, sort_index);
    // 按照距离聚类后 使用以下指标进行 目标过滤
    /*
     * 1.距离方差
     * 2.类内点数
     * 3.对应像素坐标的范围(overlape)
     */
    std::vector<float> score_vec;
    float max_score = 0.0;
    int good_cluster_index = sort_index[0];
    for(int i=0;i<std::min((int)sort_index.size(),3);i++){
        double x_min = INT_MAX;
        double z_min = INT_MAX;
        double x_max = INT_MIN;
        double z_max = INT_MIN;
        int point_num = 0;
        float variance = 0.0;
        float over_lape_rate = 0.0;
        double dis_max = 0.0;
        double dis_min = 200.0;

        double temp_ave_depth = 0.0;

        int u_min = 4096;
        int u_max = 0;
        int v_min = 3000;
        int v_max = 0;

        //std::ofstream file_db("class_id="+std::to_string(i)+".txt");
        for (auto point_iter : db_points) {
            if (point_iter.cluster == sort_index[i]) {
                temp_ave_depth += point_iter.x;
                point_num++;
                int u = uv_points[point_iter.index].x;
                int v = uv_points[point_iter.index].y;
                if (point_iter.x < dis_min) {
                    dis_min = point_iter.x;
                }
                if (point_iter.x > dis_max) {
                    dis_max = point_iter.x;
                }

                if(u>u_max){
                    u_max = u;
                }
                if(u<u_min){
                    u_min = u;
                }
                if(v>v_max){
                    v_max = v;
                }
                if(v<v_min){
                    v_min = v;
                }
                //file_db << -src_points[point_iter.index].x << "," << -src_points[point_iter.index].y << ","<<src_points[point_iter.index].z<<endl;
            }

        }   // end of for (auto point_iter : db_points)
        variance = 1.0 / (float)(dis_max - dis_min + 1.0) ;
        float overlap = NYNU::math::calc_overlap(target_roi,cv::Rect(u_min,v_min,u_max-u_min,v_max-v_min));
        over_lape_rate = overlap / (float)(target_roi.width * target_roi.height);

        float score = 3.0*sigmoid(over_lape_rate) + 2.0*sigmoid(float(point_num)/float(src_points.size())) + sigmoid(variance) ;
        if(score > max_score){
            max_score = score;
            good_cluster_index = sort_index[i];
            ave_depth = temp_ave_depth/(double)point_num;
        }
    }




}

void LidarObjectFilter(const cv::Rect& target_roi, const std::vector<cv::Point3d>& src_points,const std::vector<cv::Point2d>& uv_points, std::vector<cv::Point3d>& dst_points,std::vector<cv::Point2d>& dst_uv_points)
{
    std::vector<size_t> sort_index;
    // 小于 10点 , 排序取中值
    if(src_points.size() < 10){

        PointQucikSort(src_points, sort_index);
        int mid_index = sort_index[sort_index.size() / 2 ];
        dst_points.push_back(src_points[mid_index]);
        dst_uv_points.push_back(uv_points[mid_index]);

        return;
    }

    vector<DbscanPoint> db_points;

    for(int i = 0;i<src_points.size();i++){
        db_points.push_back(DbscanPoint(src_points[i].y,0,i));
    }

    dbscan(db_points, 1, 5);


    sort_index.reserve(db_points.size());
    // 按深度递增排序
    PointQucikSort(db_points, sort_index);
    // 按照距离聚类后 使用以下指标进行 目标过滤
    /*
     * 1.距离方差
     * 2.类内点数
     * 3.对应像素坐标的范围(overlape)
     */
    std::vector<float> score_vec;
    float max_score = 0.0;
    int good_cluster_index = sort_index[0];
    for(int i=0;i<std::min((int)sort_index.size(),3);i++){
        double x_min = INT_MAX;
        double z_min = INT_MAX;
        double x_max = INT_MIN;
        double z_max = INT_MIN;
        int point_num = 0;
        float variance = 0.0;
        float over_lape_rate = 0.0;
        double dis_max = 0.0;
        double dis_min = 200.0;

        int u_min = 4096;
        int u_max = 0;
        int v_min = 3000;
        int v_max = 0;

        //std::ofstream file_db("class_id="+std::to_string(i)+".txt");
        for (auto point_iter : db_points) {
            if (point_iter.cluster == sort_index[i]) {
                point_num++;
                int u = uv_points[point_iter.index].x;
                int v = uv_points[point_iter.index].y;
                if (point_iter.x < dis_min) {
                    dis_min = point_iter.x;
                }
                if (point_iter.x > dis_max) {
                    dis_max = point_iter.x;
                }

                if(u>u_max){
                    u_max = u;
                }
                if(u<u_min){
                    u_min = u;
                }
                if(v>v_max){
                    v_max = v;
                }
                if(v<v_min){
                    v_min = v;
                }
                //file_db << -src_points[point_iter.index].x << "," << -src_points[point_iter.index].y << ","<<src_points[point_iter.index].z<<endl;
            }

        }   // end of for (auto point_iter : db_points)
        variance = 1.0 / (float)(dis_max - dis_min + 1.0) ;
        float overlap = NYNU::math::calc_overlap(target_roi,cv::Rect(u_min,v_min,u_max-u_min,v_max-v_min));
        over_lape_rate = overlap / (float)(target_roi.width * target_roi.height);

        float score = 3.0*sigmoid(over_lape_rate) + 2.0*sigmoid(float(point_num)/float(src_points.size())) + sigmoid(variance) ;
        if(score > max_score){
            max_score = score;
            good_cluster_index = sort_index[i];
        }
    }

    for (auto point_iter : db_points) {
        if (point_iter.cluster == good_cluster_index) {
            dst_points.push_back(src_points[point_iter.index]);
            dst_uv_points.push_back(uv_points[point_iter.index]);
        }
    }


}


vector<float> PrepareImage(cv::Mat& img)
{
    using namespace cv;

    int c = Parser::getIntValue("C");
    int h = Parser::getIntValue("H");   //net h
    int w = Parser::getIntValue("W");   //net w

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


int maintest()
{
    float A[10][3] =
            {
                    10,    50,     15.6,
                    12,    49,     16,
                    11,    52,     15.8,
                    13,    52.2,   15.8,
                    12.9,  50,     17,
                    14,    48,     16.6,
                    13.7,  49,     16.5,
                    13.6,  47.8,   16.4,
                    12.3,  46,     15.9,
                    13.1,  45,     16.2
            };

    const int stateNum=3;
    const int measureNum=3;
    KalmanFilter KF(stateNum, measureNum, 0);
    KF.transitionMatrix = (Mat_<float>(3, 3) <<1,0,0,0,1,0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);

    //初始状态值
    KF.statePost = (Mat_<float>(3, 1) <<A[0][0],A[0][1],A[0][2]);
    cout<<"state0="<<KF.statePost<<endl;

    for(int i=1;i<=9;i++)
    {
        //预测
        Mat prediction = KF.predict();
        //计算测量值
        measurement.at<float>(0) = (float)A[i][0];
        measurement.at<float>(1) = (float)A[i][1];
        measurement.at<float>(2) = (float)A[i][2];
        //更新
        KF.correct(measurement);
        //输出结果
        cout<<"predict ="<<"\t"<<prediction.at<float>(0)<<"\t"<<prediction.at<float>(1)<<"\t"<<prediction.at<float>(2)<<endl;
        cout<<"measurement="<<"\t"<<measurement.at<float>(0)<<"\t"<<measurement.at<float>(1)<<"\t"<<measurement.at<float>(2)<<endl;
        cout<<"correct ="<<"\t"<<KF.statePost.at<float>(0)<<"\t"<<KF.statePost.at<float>(1)<<"\t"<<KF.statePost.at<float>(2)<<endl;
    }

}

int main_test()
{
    std::shared_ptr<CameraObjectDetect> detect_ptr = std::make_shared<CameraObjectDetect>();

    detect_ptr->Init("/home/ibd02/Project/yunzhou/Yolo/TensorRT-Yolov3-caffe/yolov3_fp16.engine");

    cv::Mat image = cv::imread("/home/ibd02/Desktop/1028/1571474838113/image/25_L.jpg");
    detect_ptr->DetectImage(image);

    //cv::Rect rt = detect_ptr->GetDetectResult();
    //detect_ptr->ShowDebug(image);
    string file_out = "/home/ibd02/Desktop/1028/1571474838113/res/recorded3.mp4";

    VideoWriter recVid(file_out,CV_FOURCC('M','J', 'P', 'G'), 5, Size(image.cols, image.rows));
    string base_path = "/home/ibd02/Desktop/1028/1571474838113/ins/";

    string image_base_path = "/home/ibd02/Desktop/1028/1571474838113/image/";

    ifstream file_data("/home/ibd02/Desktop/1028/objectTrace.txt");
    char buf[1024] = {0};
    std::vector<cv::Point3d> corrd;
    while(file_data.getline(buf,1024)){
        string strbuf = buf;
        vector<string> items;
        boost::split(items,strbuf,boost::is_any_of(";"));
        double x = boost::lexical_cast<double>(items[0]);
        double y = boost::lexical_cast<double>(items[1]);
        double z = boost::lexical_cast<double>(items[2]);
        corrd.push_back(cv::Point3d(x,y,z));
    }
    int index = 0;
    for(int i=21;i<55;i+=2){
        string img_name = std::to_string(i);
        img_name += "_L.jpg";
        cv::Mat frame = cv::imread(image_base_path + img_name);

        detect_ptr->DetectImage(frame);

        cv::Rect rt = detect_ptr->GetDetectResult();
        /////////////////////////////////////////////////
        string file_name = std::to_string(i);
        file_name += "ex.txt";
        file_name = base_path + file_name;

        fstream file_in(file_name);
        char buf[1024]={0};
        double x = corrd[index].x;
        double y = corrd[index].y;
        double z = corrd[index].z;
        index++;

        string coord = std::to_string(x) + "," + std::to_string(y) + ","+std::to_string(z);
        cv::rectangle(frame,rt,cv::Scalar(0,0,255),3,8,0);
        cv::putText(frame,coord,cv::Point(rt.x-50,rt.y-50),1,2.6,cv::Scalar(0,0,0),3);
        recVid << frame;
        cv::imwrite("/home/ibd02/Desktop/1028/1571474838113/res/"+img_name,frame);
    }

    recVid.release();
    return 0;
}

int main()
{
    std::shared_ptr<CoordinateCvt> coord_cvt_ptr = std::make_shared<CoordinateCvt>();

    std::shared_ptr<CameraObjectDetect> detect_ptr = std::make_shared<CameraObjectDetect>();

    string baseic_path = "/media/ibd02/Record-Pan/01Yunzhou/10.19数据/下午有目标GPS的数据/camera1/1571474955837/";

    detect_ptr->Init("/home/ibd02/Project/yunzhou/Yolo/TensorRT-Yolov3-caffe/yolov3_fp16.engine");

    string image_path = baseic_path + "image/";

    string lidar_path = baseic_path +  "lidar/";

    string ins_path = baseic_path +  "ins/";

    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 7.3270e+03, 0, 2.0541e+03, 0.0 , 7.3401e+03, 1.5295e+03, 0.0, 0.0, 1.0);
//
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.566765129374211, -0.1306678992695552, 0.1540506105047432);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.6971699562683624,-0.4139272957278441,0.1764838994031326);

    boost::filesystem::create_directory(baseic_path+"res_new/");  //文件夹不存在。创建
    std::vector<cv::Point2d> image_points_pro;
    std::ofstream file_trace(baseic_path + "1571474955837traceEx.txt");

    std::vector<cv::Point3d> trace_points;
    //int total_num = ;
    for(int i =0;i<73;i++){
        std::cout << "processing index = "<<i << endl;



        image_points_pro.clear();
        string name = std::to_string(i);
        /*
        cv::Point3d imu_pos;
        if(!readGPS(ins_path+name+"mo.txt",imu_pos)){
            continue;
        }

        */

        Pos imu_pos;
        if(!readGPS(ins_path+name+"mo.txt",imu_pos)){
            continue;
        }

        name += "_L.jpg";

        name = image_path + name;

        cv::Mat image = cv::imread(name);
        detect_ptr->DetectImage(image);

        cv::Rect rt = detect_ptr->GetDetectResult();
        detect_ptr->ShowDebug(image);

        std::vector<cv::Point3d> lidar_poitns;
        readPCDfile(lidar_path+std::to_string(i)+".pcd",lidar_poitns);

        cv::projectPoints(lidar_poitns,rvec,tvec,camera_matrix,cv::Mat(),image_points_pro);
        std::vector<cv::Point3d> object_3d;
        std::vector<cv::Point2d> object_2d;
        for(int j=0;j<image_points_pro.size();j++){

            int u = image_points_pro[j].x;
            int v = image_points_pro[j].y;

            if(u < 0 || u > image.cols-1 || v < 0 || v > image.rows-1){
                continue;
            }

            cv::circle(image,cv::Point(u,v),10,cv::Scalar(255,0,0),3);

        }


        for(int j=0;j<image_points_pro.size();j++){

            int u = image_points_pro[j].x;
            int v = image_points_pro[j].y;


            if(u < rt.x || u > rt.x + rt.width || v < rt.y || v > rt.y + (float)rt.height * 1.2){
                continue;
            }
            object_3d.push_back(lidar_poitns[j]);
            object_2d.push_back(cv::Point2d(u,v));
            cv::circle(image,cv::Point(u,v),10,cv::Scalar(0,0,255),3);

        }

        cv::imwrite(baseic_path + "res_new/"+std::to_string(i)+".jpg",image);
        std::vector<cv::Point3d> res_3d;
        std::vector<cv::Point2d> res_uv;
        double ave_depth = 0.0;
        if(object_3d.empty()){
            continue;
        }
        //LidarObjectFilter(rt,object_3d,object_2d, res_3d,res_uv);
        //LidarObjectFilter(rt,object_3d,object_2d, ave_depth);

        //cout << "ave_dpth = " << ave_depth <<endl;

        std::ofstream file_pcd("map03.txt");
        for(auto it : lidar_poitns){
            cv::Point3d src_pt(it.x,it.y,it.z);
            cv::Point3d dst_pt;
            coord_cvt_ptr->CvtIMU2World(src_pt,dst_pt,imu_pos);

            file_pcd << std::setprecision(16)<< dst_pt.x << ","<<std::setprecision(16)<< dst_pt.y<< ","<<dst_pt.z<<endl;

        }
        //continue;
 //       file_pcd.close();
        double ave_x = 0.0;
        double ave_y = 0.0;
        double ave_z = 0.0;

        for(auto it:object_3d){
            ave_x+=it.x;
            ave_y+=it.y;
            ave_z+=it.z;
        }

        ave_x/=(double)(object_3d.size());
        ave_y/=(double)(object_3d.size());
        ave_z/=(double)(object_3d.size());

//        Eigen::Vector3d src_pt,dst_pt;
//        src_pt << ave_x,ave_y,ave_z;

        cv::Point3d src_pt(ave_x,ave_y,ave_z);
        cv::Point3d dst_pt;
        coord_cvt_ptr->CvtIMU2World(src_pt,dst_pt,imu_pos);
        file_trace << std::setprecision(16)<< dst_pt.x << ","<<std::setprecision(16)<< dst_pt.y<< ","<<dst_pt.z<<endl;
        //file_trace <<i <<","<<std::setprecision(16)<< ave_x + imu_pos.x<<","<<std::setprecision(16)<<ave_y + imu_pos.y<<","<<std::setprecision(16)<<ave_z + imu_pos.z<<endl;
        //trace_points.push_back(cv::Point3d(-ave_x,-ave_y,ave_z));




        //cv::waitKey(1);
    }

    file_trace.close();
}

/*
 * 10.1 离线测试代码
 */
int main_bak(int argc, char* argv[]) {


    std::shared_ptr<CameraObjectDetect> detect_ptr = std::make_shared<CameraObjectDetect>();

    detect_ptr->Init("/home/ibd02/Project/yunzhou/Yolo/src_git/TensorRT-Yolov3/cmake-build-debug/yolov3_fp16.engine");

    std::string image_path = "/media/ibd02/panan/101/multi_data/1570350340435/image/";
    image_path = "/media/ibd02/panan/101/data_1007/1570436290790/image/";
    string lidar_path = "/media/ibd02/panan/101/multi_data/1570350340435/lidar/";
    lidar_path = "/media/ibd02/panan/101/data_1007/1570436290790/lidar/";

    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 7.3270e+03, 0, 2.0541e+03, 0.0 , 7.3401e+03, 1.5295e+03, 0.0, 0.0, 1.0);
//
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.566765129374211, -0.1306678992695552, 0.1540506105047432);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.6971699562683624,-0.4139272957278441,0.1764838994031326);

    std::vector<cv::Point2d> image_points_pro;
    std::ofstream file_trace("trace.txt");
    std::ofstream file_trace_kalman("trace_kalman.txt");
    std::vector<cv::Point3d> trace_points;


    ///////////////////////////////////////////////////////////////////////////
    const int stateNum=4;                                      //状态值4×1向量(x,y,△x,△y)
    const int measureNum=2;                                    //测量值2×1向量(x,y)
    KalmanFilter KF(stateNum, measureNum, 0);
    //RNG rng;
    KF.transitionMatrix = (Mat_<float>(4, 4) <<1,0,1,0,
            0,1,0,1,
            0,0,1,0,
            0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    //rng.fill(KF.statePost,RNG::UNIFORM,0,540);   //初始状态值x(0)
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    std::vector<cv::Point3d> trace_kalman;
    ///////////////////////////////////////////////////////////////////////////
    //cout << KF.statePost << endl;
    for(int i =0;i<141;i++){
        std::cout << "processing index = "<<i << endl;
        image_points_pro.clear();
       string name = std::to_string(i);
        name += "_L.jpg";
        name = "1570436290790_L.jpg";
        name = image_path + name;

        cv::Mat image = cv::imread(name);
        detect_ptr->DetectImage(image);

        cv::Rect rt = detect_ptr->GetDetectResult();
        detect_ptr->ShowDebug(image);

        std::vector<cv::Point3d> lidar_poitns;
        //readPCDfile(lidar_path+std::to_string(i)+".pcd",lidar_poitns);
        readPCDfile(lidar_path+"1570436290790.pcd",lidar_poitns);
        cv::projectPoints(lidar_poitns,rvec,tvec,camera_matrix,cv::Mat(),image_points_pro);
        std::vector<cv::Point3d> object_3d;
        std::vector<cv::Point2d> object_2d;
        for(int i=0;i<image_points_pro.size();i++){

            int u = image_points_pro[i].x;
            int v = image_points_pro[i].y;

            if(u < 0 || u > image.cols-1 || v < 0 || v > image.rows-1){
                continue;
            }

            cv::circle(image,cv::Point(u,v),10,cv::Scalar(255,0,0),3);

        }

        std::ofstream file_object(std::to_string(i)+ "-newobject3d.txt");
        for(int j=0;j<image_points_pro.size();j++){

            int u = image_points_pro[j].x;
            int v = image_points_pro[j].y;


            if(u < rt.x || u > rt.x + rt.width || v < rt.y || v > rt.y + (float)rt.height * 0.75){
                continue;
            }
            object_3d.push_back(lidar_poitns[j]);
            object_2d.push_back(cv::Point2d(u,v));
            cv::circle(image,cv::Point(u,v),10,cv::Scalar(0,0,255),3);
            file_object<<-lidar_poitns[j].x << ","<<-lidar_poitns[j].y<<","<<lidar_poitns[j].z<<endl;
        }
        std::vector<cv::Point3d> res_3d;
        std::vector<cv::Point2d> res_uv;
        double ave_depth = 0.0;
        if(object_3d.empty()){
            continue;
        }
        LidarObjectFilter(rt,object_3d,object_2d, res_3d,res_uv);
        //LidarObjectFilter(rt,object_3d,object_2d, ave_depth);

        //cout << "ave_dpth = " << ave_depth <<endl;
        file_object.close();

        //continue;
        double ave_x = 0.0;
        double ave_y = 0.0;
        double ave_z = 0.0;

        for(auto it:res_3d){
            ave_x+=it.x;
            ave_y+=it.y;
            ave_z+=it.z;
        }

        ave_x/=(double)(res_3d.size());
        ave_y/=(double)(res_3d.size());
        ave_z/=(double)(res_3d.size());

        file_trace <<i <<","<< -ave_x<<","<<-ave_y<<","<<0<<endl;
        trace_points.push_back(cv::Point3d(-ave_x,-ave_y,ave_z));
        if(i==0){

            KF.statePost = (Mat_<float>(4, 1) <<-ave_x,-ave_y,0,0);
            trace_kalman.push_back(cv::Point3f(-ave_x,-ave_y,0));
            file_trace_kalman<<-ave_x<<","<<-ave_y<<","<<0<<endl;

        }else{

            Mat prediction =  KF.predict();
            Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1) );   ///预测值(x',y')
            ////3.update measurement
            measurement.at<float>(0) = -ave_x;
            measurement.at<float>(1) = -ave_y;
            //4.update
            KF.correct(measurement);

            trace_kalman.push_back(cv::Point3d(KF.statePost.at<float>(0),KF.statePost.at<float>(1),0));
            file_trace_kalman<<KF.statePost.at<float>(0)<<","<<KF.statePost.at<float>(1)<<","<<0<<endl;
        }

        cv::imwrite(std::to_string(i)+"new.jpg",image);

        //cv::waitKey(1);
    }
    file_trace.close();
    file_trace_kalman.close();
    CreateShpFile("trace.shp",trace_points);

    CreateShpFile("trace_kalman.shp",trace_kalman);

    std::cout << "All done." << std::endl;

    return 0;
}


void readPCDfile(const std::string finname, std::vector<cv::Point3d>& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (finname, *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        //PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
        return ;
    }

    //for (size_t i = 0; i < cloud->points.size (); ++i) //显示所有的点
    for (size_t i = 0; i < cloud->points.size (); ++i){



//        if(cloud->points[i].y < 0)
//            continue;
        points.push_back(cv::Point3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z));
    }



    return;
}



void PointQucikSort(const std::vector<DbscanPoint>& db_points, std::vector<size_t>& sort_index)
{
    vector<float> point_depth;
    set<int> visited_cluster;

    for (auto it : db_points)
    {
        if (visited_cluster.find(it.cluster) != visited_cluster.end()) {
            continue;
        }
        else {
            visited_cluster.insert(it.cluster);
            point_depth.push_back(it.x);
            sort_index.push_back(it.cluster);
        }
    }

    std::vector<size_t> temp_index = NYNU::tools::sort_indexes(point_depth,true);

    for (auto& it : temp_index) {
        it = sort_index[it];
    }
    sort_index.swap(temp_index);
}

void PointQucikSort(const std::vector<cv::Point3d>& src_points, std::vector<size_t>& sort_index)
{
    vector<float> point_depth;

    for(int i=0;i<src_points.size();i++){
        point_depth.push_back(src_points[i].y);
        sort_index.push_back(i);
    }


    std::vector<size_t> temp_index = NYNU::tools::sort_indexes(point_depth,true);

    for (auto& it : temp_index) {
        it = sort_index[it];
    }
    sort_index.swap(temp_index);
}


bool readGPS(const std::string filename, cv::Point3d& imu_pos)
{
    ifstream file_in(filename);
    char buf[1024] = {0};
    while(file_in.getline(buf,1024)){
        string strbuf = buf;
        vector<string> items;
        boost::algorithm::trim(strbuf);

        boost::split(items,strbuf,boost::is_any_of(" "));

        if(items.size() < 20)
            return false;

        double x = boost::lexical_cast<double>(items[17]);
        double y = boost::lexical_cast<double>(items[18]);
        double z = boost::lexical_cast<double>(items[19]);

        imu_pos.x = x;
        imu_pos.y = y;
        imu_pos.z = z;
        break;
    }

    return true;
}


bool readGPS(const std::string filename, Pos& imu_pos)
{
    ifstream file_in(filename);
    char buf[1024] = {0};
    while(file_in.getline(buf,1024)){
        string strbuf = buf;
        vector<string> items;
        boost::algorithm::trim(strbuf);

        boost::split(items,strbuf,boost::is_any_of(" "));

        if(items.size() < 20)
            return false;

        double x = boost::lexical_cast<double>(items[17]);
        double y = boost::lexical_cast<double>(items[18]);
        double z = boost::lexical_cast<double>(items[19]);

        double heading = boost::lexical_cast<double>(items[4]);
        double pitch = boost::lexical_cast<double>(items[5]);
        double roll = boost::lexical_cast<double>(items[6]);


        imu_pos.t_x = x;
        imu_pos.t_y = y;
        imu_pos.t_z = z;
        imu_pos.heading = heading;
        imu_pos.pitch = roll;
        imu_pos.roll = pitch;
        break;
    }

    return true;
}

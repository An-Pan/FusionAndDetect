//
// Created by ibd02 on 19-10-7.
//

#include "helper.h"


#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

void CreateShpFile(const std::string file_path, const std::vector<cv::Point3d>& points)
{
    DBFHandle dbf_h;
    SHPHandle shp_h;



    shp_h = SHPCreate(file_path.c_str(), SHPT_ARC);
    if( shp_h == NULL )
    {
        printf( "Unable to create:%s\n", file_path );
        exit( 3 );
    }
    dbf_h = DBFCreate(file_path.c_str());
    int fieldIdx=DBFAddField(dbf_h, "Shape", FTDouble, 12, 0);
    double* x_coord = new double[points.size()];
    double* y_coord = new double[points.size()];
    double* z_coord = new double[points.size()];

    SHPObject *psShape;
    for(int i=0;i<points.size();i++){
        x_coord[i] = points[i].x;
        y_coord[i] = points[i].y;
        z_coord[i] = points[i].z;

    }

    psShape = SHPCreateSimpleObject(SHPT_ARC,  points.size(), x_coord, y_coord, z_coord);
    int ishape=SHPWriteObject(shp_h, -1, psShape);
    SHPDestroyObject(psShape);
    DBFWriteIntegerAttribute(dbf_h, ishape, 0, ishape);
    SHPClose( shp_h );
    DBFClose(dbf_h);
}


// for local data test
int GetTrace(string path_to_dataset, vector<cv::Point3d> &pose)
{
    //检测文件是否存在
    ifstream fin(path_to_dataset);
    if (!fin)
    {
        cerr << "I cann't find txt!" << endl;
        return 1;
    }
    char buf[512]={0};

    while(fin.getline(buf,1024)){
        std::string strbuf = buf;
        std::vector<string> items;
        boost::split(items,strbuf,boost::is_any_of(","));

        cv::Point3d pt;
        pt.x = boost::lexical_cast<double>(items[1]);
        pt.y = boost::lexical_cast<double>(items[2]);
        pt.z = boost::lexical_cast<double>(items[3]);
        pose.push_back(pt);

    }

    return 1;
}
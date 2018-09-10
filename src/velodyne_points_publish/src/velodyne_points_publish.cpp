/* 	Created	:	Linhui
 *	Date	:	2016-11-08
 *	Usage	:	从无人车保存的.bin格式二进制文件中读取点云数据，进行坐标转换，然后存入PCL的点云对象中，
 *				再转成ROS专用的sensor_msgs::PointCloud2格式，然后以固定频率publish
*/
// 标准C++头文件
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
// linux读取目录文件的头文件
#include <unistd.h>
#include <dirent.h>
// ros相关头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// pcl相关头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>	
					// 将 pcl::PointCloud<PointCloudT> 转成 pcl::PCLPointCloud2格式的函数toPCLPointCloud2()的头文件
#include <pcl_conversions/pcl_conversions.h>
					// 将 pcl::PCLPointCloud2 转成 sensor_msgs::PointCloud2格式的函数pcl_conversions::fromPCL()的头文件
// OpenCV headers
#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>

// 自定义头文件
#include "velodyne_points_publish/velodyne_points_parser.h"
#include "velodyne_points_publish/parameterReader.h"

#define Laser32
//#define Laser64_MODE
//#define SAVE_PTS_FILE

using namespace std;
const int MAX_POINT_SIZE64 = 120000;
const int MAX_POINT_SIZE32 = 60000;	// 最大的点云数量
const int MAX_POINT_SIZE16 = 20000;
// 用于退出循环的函数，关联 ctrl+'C'
bool loop_done = false;  // flag indicating main loop is to terminate
extern "C" void PointsParseLoop_quit(int sig);
void PointsParseLoop_quit(int sig)
{
    loop_done = true;
}

#ifndef Laser64_MODE
int main(int argc, char **argv)
{
  // ROS初始化
  cout<<"ROS initializing...";
  ros::init (argc, argv, "VelodynePointPublisher");
  cout<<"done."<<endl;
  ros::NodeHandle nh;
  ros::Publisher pub32 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 2);	// 声明一下将要在points这个topic上发布消息
  ros::Publisher pub16_L = nh.advertise<sensor_msgs::PointCloud2> ("/lidar16_L_points", 2);	// 声明一下将要在points这个topic上发布消息
  ros::Publisher pub16_R = nh.advertise<sensor_msgs::PointCloud2> ("/lidar16_R_points", 2);	// 声明一下将要在points这个topic上发布消息

  // parameter reader
  PARAMETER_READER parReader("/root/catkin_SLAM/src/velodyne_points_publish/parameters/velodyne_points_publish_para.txt");
  const int startFrameNo = (int)atof(parReader.getValue("startFrameNo").c_str());
  const int endFrameNo = (int)atof(parReader.getValue("endFrameNo").c_str());
  const int frameStep = (int)atof(parReader.getValue("frameStep").c_str());
  const string base_dir = parReader.getValue("base_dir");
  const string folder = parReader.getValue("folder");
  const double frequency = (double)atof(parReader.getValue("frequency").c_str());
  ros::Rate loop_rate(frequency);	// 循环频率
  // 新建一个VELODYNE_PARSER实例
  VELODYNE_PARSER *pvelodyneParser = new VELODYNE_PARSER;
  pvelodyneParser->para_table.print_base_dir();
  if(!pvelodyneParser->init_para())
  {
    cerr<<"***Error: Can't init parameter table!"<<endl;
    return 0;
  }


  // 读取所有文件序号
  string HDL32Dir = base_dir+"/LIDAR32_DATA/"+folder;
  DIR *dir;
  struct dirent *ptr;
  vector<string> frameNo;
  if ((dir=opendir(HDL32Dir.c_str())) == NULL)
  {
    cerr<<"Open dir error..."<<endl;
    exit(1);
  }
  while ((ptr=readdir(dir)) != NULL)
  {
    if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    //current dir OR parrent dir
      continue;
    else if(ptr->d_type == 8)   //file
    {
      string name(ptr->d_name);
      name.erase(0,6);        // erase the prefix "Lidar_"
      int pos = name.find(".bin");
      name.erase(pos, 4);     // erase the subfix ".bin"
      frameNo.push_back(name);
    }
    else
      continue;
  }
  closedir(dir);
  sort(frameNo.begin(), frameNo.end());   // 升序排列，表示时间顺序

  // 主循环
  cv::Mat colorimg;
  int i = startFrameNo;
  while(ros::ok() && !loop_done && i < endFrameNo)
  {
    cout<<"Frame No."<<i << ",\t"<<frameNo[i]<<endl;

    pvelodyneParser->clear_points();    // DO NOT FORGET TO CLEANUP !!!

    // parse data
    stringstream ss;
    string lidar32filename;
    string lidar16filename_L;
    string lidar16filename_R;
    string imgfliename;

    ss.str("");
    ss.clear();
    ss << base_dir << "/LIDAR32_DATA/" << folder << "/Lidar_" << frameNo[i] <<".bin";
    ss >> lidar32filename;
    ss.str("");
    ss.clear();

    ss << base_dir << "/LIDAR16_DATA/" << folder << "/" << frameNo[i] <<"-L.bin";
    ss >> lidar16filename_L;
    ss.str("");
    ss.clear();

    ss << base_dir << "/LIDAR16_DATA/" << folder << "/" << frameNo[i] <<"-R.bin";
    ss >> lidar16filename_R;
    ss.str("");
    ss.clear();

    ss << base_dir << "/IMG_DATA/" << folder << "/ImgColor_" << frameNo[i] <<".jpg";
    ss >> imgfliename;
    ss.str("");
    ss.clear();

    pvelodyneParser->parse_lidar32_data(lidar32filename);
    pvelodyneParser->parse_lidar16_data(lidar16filename_L, lidar16filename_R);

    // show color image
    colorimg = cv::imread(imgfliename);
    if(!colorimg.empty())
    {
      cv::imshow("color", colorimg);
      cv::waitKey(2);
    }
    else
    {
      cout << "No such image " << imgfliename << endl;
    }

#ifdef SAVE_PTS_FILE
    string lidar32asc;
    ss.str("");
    ss.clear();
    ss << base_dir << "/ASC/" << frameNo[i] <<".asc";
    ss >> lidar32asc;
    pvelodyneParser->save_lidar32_txt(lidar32asc);
    string lidar16asc_L, lidar16asc_R;
    ss.str("");
    ss.clear();
    ss << base_dir << "/ASC/" << frameNo[i] <<"_L.asc";
    ss >> lidar16asc_L;
    ss.str("");
    ss.clear();
    ss << base_dir << "/ASC/" << frameNo[i] <<"_R.asc";
    ss >> lidar16asc_R;
    pvelodyneParser->save_lidar16_txt(lidar16asc_L, lidar16asc_R);
#endif

    // 将点云存入pcl数据结构
    pcl::PointCloud<pcl::PointXYZI> cloud32, cloud16_L, cloud16_R;
    cloud32.width = MAX_POINT_SIZE32;			// 预设大一点的空间
    cloud32.height = 1;
    cloud32.is_dense = true;
    cloud32.resize(cloud32.width*cloud32.height);

    int num = 0;
    bool PtNumExceed = false;
    // lidar32
#ifdef Laser32
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
      if(PtNumExceed)
        break;
      for(int cnt = 0; cnt < HDL32_BEAM_POINTSIZE; cnt++)
      {
        if(pvelodyneParser->lidar32_pointcloud[beam][cnt].valid)
        {
          cloud32.points[num].x = pvelodyneParser->lidar32_pointcloud[beam][cnt].x/100.0;
          cloud32.points[num].y = pvelodyneParser->lidar32_pointcloud[beam][cnt].y/100.0;
          cloud32.points[num].z = pvelodyneParser->lidar32_pointcloud[beam][cnt].z/100.0;
          cloud32.points[num].intensity = pvelodyneParser->lidar32_pointcloud[beam][cnt].intensity
                                        - (int)pvelodyneParser->lidar32_pointcloud[beam][cnt].intensity
                                        + float(beam);
          // 重新分配beam数值，即替换掉intensity整数部分
          num++;
          if(num >= MAX_POINT_SIZE32)
          {
            PtNumExceed = true;
            break;
          }
        }
      }
    }
#else
    const int beamSel[16] = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30};	// {0,2,3,5,6,8,9,11,12,14,15,17,18,20,21,23}
    for(int k = 0; k < 16; k++)
    {
      int beam = beamSel[k];
      if(PtNumExceed)
        break;
      for(int cnt = 0; cnt < HDL32_BEAM_POINTSIZE; cnt++)
      {
        if(pvelodyneParser->lidar32_pointcloud[beam][cnt].valid)
        {
          cloud32.points[num].x = pvelodyneParser->lidar32_pointcloud[beam][cnt].x/100.0;
          cloud32.points[num].y = pvelodyneParser->lidar32_pointcloud[beam][cnt].y/100.0;
          cloud32.points[num].z = pvelodyneParser->lidar32_pointcloud[beam][cnt].z/100.0;
          cloud32.points[num].intensity = pvelodyneParser->lidar32_pointcloud[beam][cnt].intensity
                                        - (int)pvelodyneParser->lidar32_pointcloud[beam][cnt].intensity
                                        + float(k);
          // 重新分配beam数值，即替换掉intensity整数部分
          num++;
          if(num >= MAX_POINT_SIZE32)
          {
            PtNumExceed = true;
            break;
          }
        }
      }
    }
#endif
    cloud32.width = num;
    cloud32.height = 1;
    cloud32.resize(cloud32.width*cloud32.height);	// 重新调整点云尺寸至真实值
    cout<<"cloud32 points size: "<<cloud32.points.size()<<endl;

    // lidar16-L
    cloud16_L.width = MAX_POINT_SIZE16;			// 预设大一点的空间
    cloud16_L.height = 1;
    cloud16_L.is_dense = true;
    cloud16_L.resize(cloud16_L.width*cloud16_L.height);

    num = 0;
    PtNumExceed = false;
    for(int beam = 0; beam < VLP16_BEAM_NUM; beam++)
    {
      if(PtNumExceed)
        break;
      for(int cnt = 0; cnt < VLP16_BEAM_POINTSIZE; cnt++)
      {
        if(pvelodyneParser->lidar16_pointcloud_L[beam][cnt].valid)
        {
          cloud16_L.points[num].x = pvelodyneParser->lidar16_pointcloud_L[beam][cnt].x/100.0;
          cloud16_L.points[num].y = pvelodyneParser->lidar16_pointcloud_L[beam][cnt].y/100.0;
          cloud16_L.points[num].z = pvelodyneParser->lidar16_pointcloud_L[beam][cnt].z/100.0;
          cloud16_L.points[num].intensity = pvelodyneParser->lidar16_pointcloud_L[beam][cnt].intensity; // 解析时已经更新过intensity
          num++;
          if(num >= MAX_POINT_SIZE16)
          {
            PtNumExceed = true;
            break;
          }
        }
      }
    }
    cloud16_L.width = num;
    cloud16_L.height = 1;
    cloud16_L.resize(cloud16_L.width*cloud16_L.height);	// 重新调整点云尺寸至真实值
    cout<<"cloud16_L points size: "<<cloud16_L.points.size()<<endl;

    // lidar16-R
    cloud16_R.width = MAX_POINT_SIZE16;			// 预设大一点的空间
    cloud16_R.height = 1;
    cloud16_R.is_dense = true;
    cloud16_R.resize(cloud16_R.width*cloud16_R.height);

    num = 0;
    PtNumExceed = false;
    for(int beam = 0; beam < VLP16_BEAM_NUM; beam++)
    {
      if(PtNumExceed)
        break;
      for(int cnt = 0; cnt < VLP16_BEAM_POINTSIZE; cnt++)
      {
        if(pvelodyneParser->lidar16_pointcloud_R[beam][cnt].valid)
        {
          cloud16_R.points[num].x = pvelodyneParser->lidar16_pointcloud_R[beam][cnt].x/100.0;
          cloud16_R.points[num].y = pvelodyneParser->lidar16_pointcloud_R[beam][cnt].y/100.0;
          cloud16_R.points[num].z = pvelodyneParser->lidar16_pointcloud_R[beam][cnt].z/100.0;
          cloud16_R.points[num].intensity = pvelodyneParser->lidar16_pointcloud_R[beam][cnt].intensity; // 解析时已经更新过intensity
          num++;
          if(num >= MAX_POINT_SIZE16)
          {
            PtNumExceed = true;
            break;
          }
        }
      }
    }
    cloud16_R.width = num;
    cloud16_R.height = 1;
    cloud16_R.resize(cloud16_R.width*cloud16_R.height);	// 重新调整点云尺寸至真实值
    cout<<"cloud16_R points size: "<<cloud16_R.points.size()<<endl;

    // 将pcl::PointCloud<pcl::PointCloudXYZI>格式转换成pcl::PCLPointCLoud2格式
    pcl::PCLPointCloud2 tmp_cloud32, tmp_cloud16_L, tmp_cloud16_R;
    pcl::toPCLPointCloud2(cloud32, tmp_cloud32);
    pcl::toPCLPointCloud2(cloud16_L, tmp_cloud16_L);
    pcl::toPCLPointCloud2(cloud16_R, tmp_cloud16_R);
    // 将pcl::PCLPointCLoud2格式转换成sensor_msgs::PointCloud2格式
    sensor_msgs::PointCloud2 output32, output16_L, output16_R;
    pcl_conversions::fromPCL(tmp_cloud32, output32);
    pcl_conversions::fromPCL(tmp_cloud16_L, output16_L);
    pcl_conversions::fromPCL(tmp_cloud16_R, output16_R);
    // 发布消息
    output32.header.frame_id = "/camera";
    output16_L.header.frame_id = "/camera";
    output16_R.header.frame_id = "/camera";
    pub32.publish(output32);
    pub16_L.publish(output16_L);
    pub16_R.publish(output16_R);

    // spin & sleep
    i+=frameStep;
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete pvelodyneParser;
  return 0;
}

#else
int main(int argc, char **argv)
{
  // ROS初始化
  ros::init (argc, argv, "VelodynePointPublisher");
  ros::NodeHandle nh;
  ros::Publisher pub64 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 2);	// 声明一下将要在points这个topic上发布消息

  // parameter reader
  PARAMETER_READER parReader("/root/catkin_SLAM/src/velodyne_points_publish/parameters/velodyne_points_publish_para.txt");
  const int startFrameNo = (int)atof(parReader.getValue("startFrameNo").c_str());
  const int endFrameNo = (int)atof(parReader.getValue("endFrameNo").c_str());
  const int frameStep = (int)atof(parReader.getValue("frameStep").c_str());
  string base_dir = parReader.getValue("base_dir");
  const string folder = parReader.getValue("folder");
  const double frequency = (double)atof(parReader.getValue("frequency").c_str());

  ros::Rate loop_rate(frequency);	// 循环频率

  // 新建一个VELODYNE_PARSER实例
  VELODYNE_PARSER *pvelodyneParser = new VELODYNE_PARSER;
  pvelodyneParser->para_table.print_base_dir();
  if(!pvelodyneParser->init_para())
  {
    cerr<<"***Error: Can't init parameter table!"<<endl;
    return 0;
  }

  // 读取所有文件序号
  string HDL64Dir = "/root/LOAM_DataBag/laser64_Obs/Ladar64";
  DIR *dir;
  struct dirent *ptr;
  vector<string> frameNo;
  if ((dir=opendir(HDL64Dir.c_str())) == NULL)
  {
    cerr<<"Open dir error..."<<endl;
    exit(1);
  }
  while ((ptr=readdir(dir)) != NULL)
  {
    if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    //current dir OR parrent dir
      continue;
    else if(ptr->d_type == 8)   //file
    {
      string name(ptr->d_name);
//      name.erase(0, 8); // erase prefix
      int pos = name.find(".bin");
      name.erase(pos, 4);     // erase the subfix ".bin"
      frameNo.push_back(name);
    }
    else
      continue;
  }
  closedir(dir);
  sort(frameNo.begin(), frameNo.end());   // 升序排列，表示时间顺序


  // 主循环
  int i = startFrameNo;
  while(ros::ok() && !loop_done && i < endFrameNo)
  {
    cout<<"Frame No."<<i << ",\t"<<frameNo[i]<<endl;
    pvelodyneParser->clear_points();    // DO NOT FORGET TO CLEANUP !!!

    // parse data
    stringstream ss;
    string lidar64filename;

    ss.str("");
    ss.clear();
//    ss << HDL64Dir << "/Ladar64-" << frameNo[i] <<".bin";
    ss << HDL64Dir << "/" << frameNo[i] <<".bin";
    ss >> lidar64filename;
    ss.str("");
    ss.clear();

    pvelodyneParser->parse_lidar64_data(lidar64filename);

#ifdef SAVE_PTS_FILE
    string lidar64asc;
    ss.str("");
    ss.clear();
    ss << base_dir << "/ASC/Lidar64-" << frameNo[i] <<".asc";
    ss >> lidar64asc;
    pvelodyneParser->save_lidar64_txt(lidar64asc);
#endif

    // 将点云存入pcl数据结构
    pcl::PointCloud<pcl::PointXYZI> cloud64;
    cloud64.width = MAX_POINT_SIZE64;			// 预设大一点的空间
    cloud64.height = 1;
    cloud64.is_dense = true;
    cloud64.resize(cloud64.width*cloud64.height);

    int num = 0;
    bool PtNumExceed = false;

//    for(int beam = 0; beam < HDL64_BEAM_NUM; beam++)
//    {
//      if(PtNumExceed)
//        break;
//      for(int cnt = 0; cnt < HDL64_BEAM_POINTSIZE; cnt++)
//      {
//        if(pvelodyneParser->lidar64_pointcloud[beam][cnt].valid)
//        {
//          cloud64.points[num].x = pvelodyneParser->lidar64_pointcloud[beam][cnt].x/100.0;
//          cloud64.points[num].y = pvelodyneParser->lidar64_pointcloud[beam][cnt].y/100.0;
//          cloud64.points[num].z = pvelodyneParser->lidar64_pointcloud[beam][cnt].z/100.0;
//          cloud64.points[num].intensity = pvelodyneParser->lidar64_pointcloud[beam][cnt].intensity
//                                        - (int)pvelodyneParser->lidar64_pointcloud[beam][cnt].intensity
//                                        + float(beam);
//          // 重新分配beam数值，即替换掉intensity整数部分
//          num++;
//          if(num >= MAX_POINT_SIZE64)
//          {
//            PtNumExceed = true;
//            break;
//          }
//        }
//      }
//    }

    const int beamSel[32] = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62};	// {0,2,3,5,6,8,9,11,12,14,15,17,18,20,21,23}
    for(int k = 0; k < 32; k++)
    {
      int beam = beamSel[k];
      if(PtNumExceed)
        break;
      for(int cnt = 0; cnt < HDL64_BEAM_POINTSIZE; cnt++)
      {
        if(pvelodyneParser->lidar64_pointcloud[beam][cnt].valid)
        {
          cloud64.points[num].x = pvelodyneParser->lidar64_pointcloud[beam][cnt].x/100.0;
          cloud64.points[num].y = pvelodyneParser->lidar64_pointcloud[beam][cnt].y/100.0;
          cloud64.points[num].z = pvelodyneParser->lidar64_pointcloud[beam][cnt].z/100.0;
          cloud64.points[num].intensity = pvelodyneParser->lidar64_pointcloud[beam][cnt].intensity
                                        - (int)pvelodyneParser->lidar64_pointcloud[beam][cnt].intensity
                                        + float(k);
          // 重新分配beam数值，即替换掉intensity整数部分
          num++;
          if(num >= MAX_POINT_SIZE32)
          {
            PtNumExceed = true;
            break;
          }
        }
      }
    }

    // save data
//    string lidar64asc;
//    ss.str("");
//    ss.clear();
//    ss << base_dir << "/ASC/Lidar64-" << frameNo[i] <<".asc";
//    ss >> lidar64asc;
//    fstream fileout;
//    fileout.open(lidar64asc, ios::out);
//    if(!fileout)
//    {
//        cerr<<"***Error: Can't open file \""<<lidar64asc<<"\""<<endl;
//    }
//    for(int i = 0; i < cloud64.points.size(); i++)
//    {

//        fileout << cloud64.points[i].x*100.0 << "\t\t"<<
//                   cloud64.points[i].y*100.0 << "\t\t"<<
//                   cloud64.points[i].z*100.0 << "\t\t"<<
//                   cloud64.points[i].intensity << "\t\t"<<
//                   endl;
//    }
//    fileout.close();


    cloud64.width = num;
    cloud64.height = 1;
    cloud64.resize(cloud64.width*cloud64.height);	// 重新调整点云尺寸至真实值
    cout<<"cloud64 points size: "<<cloud64.points.size()<<endl;


    // 将pcl::PointCloud<pcl::PointCloudXYZI>格式转换成pcl::PCLPointCLoud2格式
    pcl::PCLPointCloud2 tmp_cloud64;
    pcl::toPCLPointCloud2(cloud64, tmp_cloud64);

    // 将pcl::PCLPointCLoud2格式转换成sensor_msgs::PointCloud2格式
    sensor_msgs::PointCloud2 output64;
    pcl_conversions::fromPCL(tmp_cloud64, output64);

    // 发布消息
    output64.header.frame_id = "/camera";

    pub64.publish(output64);

    // spin & sleep
    i+=frameStep;
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete pvelodyneParser;
  return 0;
}

#endif

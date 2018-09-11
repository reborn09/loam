#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <strstream>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;

const int MAX_POINTNUM32 = 60000;
const int PointMaxNum = MAX_POINTNUM32;  // maximum number of points in a single point cloud

bool systemInited = false;

float framePeriod = 0.1;

double timeLaserCloud32 = 0;
double timeCornerPointsSharp32 = 0;
double timeCornerPointsLessSharp32 = 0;
double timeSurfPointsFlat32 = 0;
double timeSurfPointsLessFlat32 = 0;

bool newLaserCloud32 = false;
bool newCornerPointsSharp32 = false;
bool newCornerPointsLessSharp32 = false;
bool newSurfPointsFlat32 = false;
bool newSurfPointsLessFlat32 = false;

  // keypoints from lidar32
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud32(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp32(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp32(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat32(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat32(new pcl::PointCloud<pcl::PointXYZI>());
  // point cloud truly used for odometry
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());
  // published key points
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSharpLast(new pcl::PointCloud<pcl::PointXYZI>());	// 上一帧的角点
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFlatLast(new pcl::PointCloud<pcl::PointXYZI>());		// 上一帧的面点
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());	// 上一帧的reference角点
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());		// 上一帧的reference面点

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
  // k-d tree
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

int laserCloudCornerLastNum;	// 函数外的变量默认初始化为0
int laserCloudSurfLastNum;

//int pointSelCornerInd[PointMaxNum];
int pointSearchCornerInd1[PointMaxNum];
int pointSearchCornerInd2[PointMaxNum];

//int pointSelSurfInd[PointMaxNum];
int pointSearchSurfInd1[PointMaxNum];
int pointSearchSurfInd2[PointMaxNum];
int pointSearchSurfInd3[PointMaxNum];

float transform[6] = {0};	// 当前帧的运动量，需要注意的是，这个运动量指的将当前帧坐标系移回上一帧坐标系所需要的运动量
                          // 即：对于空间中一个点X,前后两帧对其观测的局部坐标分别为X1, X2，则有 X2 = transform * X1
                          // 所以TransformToStart()才需要使用transform的逆变换，即 X1 = transform^-1 * X2
float transformSum[6] = {0};	// 从第一帧开始累积的变换

float keyFramePose[6] = {0};  // key frame's pose
float keyFrameTransform[6] = {0}; // transform between two key frames


void TransformToStart(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  // can't understand why plus framePeriod?
  // float s = (pi->intensity - int(pi->intensity) + framePeriod) / framePeriod;
  float s = (pi->intensity - int(pi->intensity)) / framePeriod;
  float rx = s * transform[0]; // 基于线性运动假设
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  // 逆旋转变换，先Z轴旋转逆变换，在X轴，再Y轴；说明正变换的旋转矩阵是按照 R=Rz*Rx*Ry得到的
  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

void TransformToEnd(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  // 先 transform to start
  //  float s = (pi->intensity - int(pi->intensity) + framePeriod) / framePeriod;
  float s = (pi->intensity - int(pi->intensity)) / framePeriod;

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];
  
  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  // 再 transform to end
  rx = transform[0];
  ry = transform[1];
  rz = transform[2];
  tx = transform[3];
  ty = transform[4];
  tz = transform[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  po->x = cos(rz) * x5 - sin(rz) * y5 + tx;
  po->y = sin(rz) * x5 + cos(rz) * y5 + ty;
  po->z = z5 + tz;
  po->intensity = pi->intensity;
}

void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                        float &ox, float &oy, float &oz)
{
  float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
               + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
  float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
               - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
               + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
  float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
               - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

  // laser cloud handler for lidar32
void laserCloud32Handler(const sensor_msgs::PointCloud2ConstPtr& laserCloud32Msg)
{
  timeLaserCloud32 = laserCloud32Msg->header.stamp.toSec();
  laserCloud32->clear();
  pcl::fromROSMsg(*laserCloud32Msg, *laserCloud32);
  newLaserCloud32 = true;
}
void laserCloudSharp32Handler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharp2)
{
  timeCornerPointsSharp32 = cornerPointsSharp2->header.stamp.toSec();

  cornerPointsSharp32->clear();
  pcl::fromROSMsg(*cornerPointsSharp2, *cornerPointsSharp32);

  newCornerPointsSharp32 = true;
}
void laserCloudLessSharp32Handler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharp2)
{
  timeCornerPointsLessSharp32 = cornerPointsLessSharp2->header.stamp.toSec();

  cornerPointsLessSharp32->clear();
  pcl::fromROSMsg(*cornerPointsLessSharp2, *cornerPointsLessSharp32);

  newCornerPointsLessSharp32 = true;
}
void laserCloudFlat32Handler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlat2)
{
  timeSurfPointsFlat32 = surfPointsFlat2->header.stamp.toSec();

  surfPointsFlat32->clear();
  pcl::fromROSMsg(*surfPointsFlat2, *surfPointsFlat32);

  newSurfPointsFlat32 = true;
}
void laserCloudLessFlat32Handler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlat2)
{
  timeSurfPointsLessFlat32 = surfPointsLessFlat2->header.stamp.toSec();

  surfPointsLessFlat32->clear();
  pcl::fromROSMsg(*surfPointsLessFlat2, *surfPointsLessFlat32);

  newSurfPointsLessFlat32 = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  // subscriber
    // lidar32
  ros::Subscriber subLaserCloud32 = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/laser_cloud_32", 2, laserCloud32Handler);

  ros::Subscriber subCornerPointsSharp32 = nh.subscribe<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_sharp_32", 2, laserCloudSharp32Handler);

  ros::Subscriber subCornerPointsLessSharp32 = nh.subscribe<sensor_msgs::PointCloud2>
                                             ("/laser_cloud_less_sharp_32", 2, laserCloudLessSharp32Handler);

  ros::Subscriber subSurfPointsFlat32 = nh.subscribe<sensor_msgs::PointCloud2>
                                      ("/laser_cloud_flat_32", 2, laserCloudFlat32Handler);

  ros::Subscriber subSurfPointsLessFlat32 = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_less_flat_32", 2, laserCloudLessFlat32Handler);

  // publisher
  ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_corner_last", 2);

  ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_surf_last", 2);

  ros::Publisher pubLaserCloudSharpLast = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_sharp_last", 2);

  ros::Publisher pubLaserCloudFlatLast = nh.advertise<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_flat_last", 2);


  ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::PointXYZI pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  struct timeval t_start, t_end;
  ros::Rate rate(100);
  bool status = ros::ok();
  int frameNo = 0;
  while (status) 
  {
    ros::spinOnce();    

    // 检查是否收到点云，且时间戳一致
    if (newLaserCloud32 &&
        newCornerPointsSharp32 && newCornerPointsLessSharp32 && newSurfPointsFlat32 && newSurfPointsLessFlat32 &&
        fabs(timeLaserCloud32 - timeSurfPointsLessFlat32) < 0.005 &&
        fabs(timeCornerPointsSharp32 - timeSurfPointsLessFlat32) < 0.005 &&
        fabs(timeCornerPointsLessSharp32 - timeSurfPointsLessFlat32) < 0.005 &&
        fabs(timeSurfPointsFlat32 - timeSurfPointsLessFlat32) < 0.005 )
    {
      gettimeofday(&t_start, NULL);

      newLaserCloud32 = false;
      newCornerPointsSharp32 = false;
      newCornerPointsLessSharp32 = false;
      newSurfPointsFlat32 = false;
      newSurfPointsLessFlat32 = false;

      frameNo++;

      cornerPointsSharp->clear();
      cornerPointsLessSharp->clear();
      surfPointsFlat->clear();
      surfPointsLessFlat->clear();

//      *cornerPointsSharp += *cornerPointsSharp32;
//      *cornerPointsLessSharp += *cornerPointsLessSharp32;
//      *surfPointsFlat += *surfPointsFlat32;
//      *surfPointsLessFlat += *surfPointsLessFlat32;

      *cornerPointsSharp += *cornerPointsSharp32;
      cornerPointsSharp->is_dense = false;
      cornerPointsSharp->width = cornerPointsSharp32->points.size();
      cornerPointsSharp->height = 1;
      cornerPointsSharp->resize(cornerPointsSharp ->width * cornerPointsSharp->height);

      *cornerPointsLessSharp += *cornerPointsLessSharp32;
      cornerPointsSharp->is_dense = false;
      cornerPointsLessSharp->width = cornerPointsLessSharp32->points.size();
      cornerPointsLessSharp->height = 1;
      cornerPointsLessSharp->resize(cornerPointsLessSharp->width * cornerPointsLessSharp->height);

      *surfPointsFlat += *surfPointsFlat32;
      cornerPointsSharp->is_dense = false;
      surfPointsFlat->width = surfPointsFlat32->points.size();
      surfPointsFlat->height = 1;
      surfPointsFlat->resize(surfPointsFlat->width * surfPointsFlat->height);

      *surfPointsLessFlat += *surfPointsLessFlat32;
      cornerPointsSharp->is_dense = false;
      surfPointsLessFlat->width = surfPointsLessFlat32->points.size();
      surfPointsLessFlat->height = 1;
      surfPointsLessFlat->resize(surfPointsLessFlat->width * surfPointsLessFlat->height);


      if (!systemInited) {
        // 更新laserCloudSharpLast, laserCloudFlatLast, laserCloudCornerLast和laserCloudSurfLast以及相应的kd-tree
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsSharp;
        cornerPointsSharp = laserCloudSharpLast;
        laserCloudSharpLast = laserCloudTemp;

        laserCloudTemp = surfPointsFlat;
        surfPointsFlat = laserCloudFlatLast;
        laserCloudFlatLast = laserCloudTemp;

        laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        // publish laserCloudSharpLast, laserCloudFlatLast, laserCloudCornerLast, laserCloudSurfLast
        sensor_msgs::PointCloud2 laserCloudSharpLast2;
        pcl::toROSMsg(*laserCloudSharpLast, laserCloudSharpLast2);
        laserCloudSharpLast2.header.stamp = ros::Time().fromSec(timeCornerPointsSharp32);
        laserCloudSharpLast2.header.frame_id = "/camera";
        pubLaserCloudSharpLast.publish(laserCloudSharpLast2);

        sensor_msgs::PointCloud2 laserCloudFlatLast2;
        pcl::toROSMsg(*laserCloudFlatLast, laserCloudFlatLast2);
        laserCloudFlatLast2.header.stamp = ros::Time().fromSec(timeSurfPointsFlat32);
        laserCloudFlatLast2.header.frame_id = "/camera";
        pubLaserCloudFlatLast.publish(laserCloudFlatLast2);

        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeCornerPointsLessSharp32);
        laserCloudCornerLast2.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat32);
        laserCloudSurfLast2.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        systemInited = true;
        continue;
      }

      laserCloudOri->clear();
      coeffSel->clear();

      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) 
      {
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum = surfPointsFlat->points.size();

        std::cout<<"key points num: "<<cornerPointsSharpNum + surfPointsFlatNum << std::endl;

        // step1. optimize transform iteratively
        for (int iterCount = 0; iterCount < 25; iterCount++) 
        {
          // step1.1 select corner points
          for (int i = 0; i < cornerPointsSharpNum; i++) 
          {            
            TransformToStart(&cornerPointsSharp->points[i], &pointSel);
            if (iterCount % 5 == 0)
            {
              // poIntSearchInd表示最近邻点的ID，pointSearchSqDis表示相应的距离平方              
              kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

              int closestPointInd = -1, minPointInd2 = -1;
              if (pointSearchSqDis[0] < 25)
              {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);	// 第几个beam

                float pointSqDis, minPointSqDis2 = 25;
                for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
                {
                  if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5)
                  {
                    break;
                  }

                  pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                               (laserCloudCornerLast->points[j].x - pointSel.x) + 
                               (laserCloudCornerLast->points[j].y - pointSel.y) * 
                               (laserCloudCornerLast->points[j].y - pointSel.y) + 
                               (laserCloudCornerLast->points[j].z - pointSel.z) * 
                               (laserCloudCornerLast->points[j].z - pointSel.z);

                  if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
                for (int j = closestPointInd - 1; j >= 0; j--)
                {
                  if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                               (laserCloudCornerLast->points[j].x - pointSel.x) + 
                               (laserCloudCornerLast->points[j].y - pointSel.y) * 
                               (laserCloudCornerLast->points[j].y - pointSel.y) + 
                               (laserCloudCornerLast->points[j].z - pointSel.z) * 
                               (laserCloudCornerLast->points[j].z - pointSel.z);

                  if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
              }

              pointSearchCornerInd1[i] = closestPointInd;
              pointSearchCornerInd2[i] = minPointInd2;
            }

            if (pointSearchCornerInd2[i] >= 0)
            {
              tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
              tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

              float x0 = pointSel.x;
              float y0 = pointSel.y;
              float z0 = pointSel.z;
              float x1 = tripod1.x;
              float y1 = tripod1.y;
              float z1 = tripod1.z;
              float x2 = tripod2.x;
              float y2 = tripod2.y;
              float z2 = tripod2.z;

              float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                         * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                         + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                         * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                         + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                         * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

              float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

              float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                       + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

              float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                       - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

              float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                       + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

              float ld2 = a012 / l12;

              pointProj = pointSel;
              pointProj.x -= la * ld2;
              pointProj.y -= lb * ld2;
              pointProj.z -= lc * ld2;

              float s = 1;
              if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(ld2);
              }

              coeff.x = s * la;
              coeff.y = s * lb;
              coeff.z = s * lc;
              coeff.intensity = s * ld2;

              if (s > 0.1) {
                laserCloudOri->push_back(cornerPointsSharp->points[i]);
                coeffSel->push_back(coeff);
              }
            }
          }

          // step1.2 select surf points
          for (int i = 0; i < surfPointsFlatNum; i++)
          {
              TransformToStart(&surfPointsFlat->points[i], &pointSel);
              if (iterCount % 5 == 0) {
              kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

              int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
              if (pointSearchSqDis[0] < 25) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                  if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                    break;
                  }

                  pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                               (laserCloudSurfLast->points[j].x - pointSel.x) + 
                               (laserCloudSurfLast->points[j].y - pointSel.y) * 
                               (laserCloudSurfLast->points[j].y - pointSel.y) + 
                               (laserCloudSurfLast->points[j].z - pointSel.z) * 
                               (laserCloudSurfLast->points[j].z - pointSel.z);

                  if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                     if (pointSqDis < minPointSqDis2) {
                       minPointSqDis2 = pointSqDis;
                       minPointInd2 = j;
                     }
                  } else {
                     if (pointSqDis < minPointSqDis3) {
                       minPointSqDis3 = pointSqDis;
                       minPointInd3 = j;
                     }
                  }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                               (laserCloudSurfLast->points[j].x - pointSel.x) + 
                               (laserCloudSurfLast->points[j].y - pointSel.y) * 
                               (laserCloudSurfLast->points[j].y - pointSel.y) + 
                               (laserCloudSurfLast->points[j].z - pointSel.z) * 
                               (laserCloudSurfLast->points[j].z - pointSel.z);

                  if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  } else {
                    if (pointSqDis < minPointSqDis3) {
                      minPointSqDis3 = pointSqDis;
                      minPointInd3 = j;
                    }
                  }
                }
              }

              pointSearchSurfInd1[i] = closestPointInd;
              pointSearchSurfInd2[i] = minPointInd2;
              pointSearchSurfInd3[i] = minPointInd3;
            }

            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
              tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
              tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
              tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

              float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
                       - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
              float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
                       - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
              float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
                       - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
              float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

              float ps = sqrt(pa * pa + pb * pb + pc * pc);
              pa /= ps;
              pb /= ps;
              pc /= ps;
              pd /= ps;

              float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

              pointProj = pointSel;
              pointProj.x -= pa * pd2;
              pointProj.y -= pb * pd2;
              pointProj.z -= pc * pd2;

              float s = 1;
              if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                  + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
              }

              coeff.x = s * pa;
              coeff.y = s * pb;
              coeff.z = s * pc;
              coeff.intensity = s * pd2;

              if (s > 0.1) {
                laserCloudOri->push_back(surfPointsFlat->points[i]);
                coeffSel->push_back(coeff);
              }
            }
          }

          int pointSelNum = laserCloudOri->points.size();
          if (pointSelNum < 10) {
            continue;
          }

          // step1.3 optimization
          cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
          for (int i = 0; i < pointSelNum; i++)
          {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            // can't understand
//            float s = (pointOri.intensity - int(pointOri.intensity) + framePeriod) / framePeriod;
//            float s = (pointOri.intensity - int(pointOri.intensity)) / framePeriod;
            float s = 1;

            float srx = sin(s * transform[0]);
            float crx = cos(s * transform[0]);
            float sry = sin(s * transform[1]);
            float cry = cos(s * transform[1]);
            float srz = sin(s * transform[2]);
            float crz = cos(s * transform[2]);
            float tx = s * transform[3];
            float ty = s * transform[4];
            float tz = s * transform[5];

            float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z 
                      + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                      + (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
                      + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                      + (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
                      + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

            float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x 
                      + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z 
                      + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx) 
                      + s*tz*crx*cry) * coeff.x
                      + ((s*cry*crz - s*srx*sry*srz)*pointOri.x 
                      + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
                      + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry) 
                      - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

            float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
                      + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                      + (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
                      + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                      + ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
                      + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

            float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y 
                      - s*(crz*sry + cry*srx*srz) * coeff.z;
  
            float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y 
                      - s*(sry*srz - cry*crz*srx) * coeff.z;
  
            float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;
  
            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
//            matB.at<float>(i, 0) = -0.7 * d2;  // this number seems to be very important

          }
          cv::transpose(matA, matAt);
          matAtA = matAt * matA;
          matAtB = matAt * matB;
          cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

          if (iterCount == 0)
          {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) {
              if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                  matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
              } else {
                break;
              }
            }
            matP = matV.inv() * matV2;
          }

          if (isDegenerate /*&& 0*/)
          {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;

            ROS_WARN ("laser odometry degenerate");
          }

          float sigma_0 = matX.at<float>(0, 0);
          float sigma_1 = matX.at<float>(1, 0);
          float sigma_2 = matX.at<float>(2, 0);
          float sigma_3 = matX.at<float>(3, 0);
          float sigma_4 = matX.at<float>(4, 0);
          float sigma_5 = matX.at<float>(5, 0);
          if(!isnan(sigma_0) && !isnan(sigma_1) && !isnan(sigma_2)
             && !isnan(sigma_3) && !isnan(sigma_4) && !isnan(sigma_5))
          {
            transform[0] += sigma_0;
            transform[1] += sigma_1;
            transform[2] += sigma_2;
            transform[3] += sigma_3;
            transform[4] += sigma_4;
            transform[5] += sigma_5;
          }

          float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
                       + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
                       + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
          float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
                       + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
                       + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

          if (deltaR < 0.1 && deltaT < 0.1) { // 两次迭代更新值很小时，认为收敛
            break;
          }

          ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
        }

        // step2. generate transformSum
        float rx, ry, rz, tx, ty, tz;
        AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                           -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);

        float x1 = cos(rz) * (transform[3] /*- imuShiftFromStartXLast*/) 
                 - sin(rz) * (transform[4] /*- imuShiftFromStartYLast*/);
        float y1 = sin(rz) * (transform[3] /*- imuShiftFromStartXLast*/) 
                 + cos(rz) * (transform[4] /*- imuShiftFromStartYLast*/);
        float z1 = transform[5] * 1.05 /*- imuShiftFromStartZLast*/;

        float x2 = x1;
        float y2 = cos(rx) * y1 - sin(rx) * z1;
        float z2 = sin(rx) * y1 + cos(rx) * z1;

        tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
        ty = transformSum[4] - y2;
        tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        transformSum[0] = rx;
        transformSum[1] = ry;
        transformSum[2] = rz;
        transformSum[3] = tx;
        transformSum[4] = ty;
        transformSum[5] = tz;

        // step3. publish
          // publish accumulated transform
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);
        laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat32);
        laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry.pose.pose.orientation.z = geoQuat.x;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = tx;
        laserOdometry.pose.pose.position.y = ty;
        laserOdometry.pose.pose.position.z = tz;
        pubLaserOdometry.publish(laserOdometry);

          // broadcast tf
        laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat32);
        laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
        tfBroadcaster.sendTransform(laserOdometryTrans);

          // project key points to coordinate of End of this frame
        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
          TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }


        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++)
        {
          TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        for (int i = 0; i < cornerPointsSharpNum; i++)
        {
          TransformToEnd(&cornerPointsSharp->points[i], &cornerPointsSharp->points[i]);
        }

        for (int i = 0; i < surfPointsFlatNum; i++)
        {
          TransformToEnd(&surfPointsFlat->points[i], &surfPointsFlat->points[i]);
        }
      }

      // 更新laserCloudSharpLast, laserCloudFlatLast, laserCloudCornerLast和laserCloudSurfLast以及相应的kd-tree
      pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsSharp;
      cornerPointsSharp = laserCloudSharpLast;
      laserCloudSharpLast = laserCloudTemp;

      laserCloudTemp = surfPointsFlat;
      surfPointsFlat = laserCloudFlatLast;
      laserCloudFlatLast = laserCloudTemp;

      laserCloudTemp = cornerPointsLessSharp;
      cornerPointsLessSharp = laserCloudCornerLast;
      laserCloudCornerLast = laserCloudTemp;

      laserCloudTemp = surfPointsLessFlat;
      surfPointsLessFlat = laserCloudSurfLast;
      laserCloudSurfLast = laserCloudTemp;

      laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      laserCloudSurfLastNum = laserCloudSurfLast->points.size();
      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
      {
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
      }

      // 发布laserCloudSharpLast, laserCloudFlatLast, laserCloudCornerLast和laserCloudSurfLast
      sensor_msgs::PointCloud2 laserCloudSharpLast2;
      pcl::toROSMsg(*laserCloudSharpLast, laserCloudSharpLast2);
      laserCloudSharpLast2.header.stamp = ros::Time().fromSec(timeCornerPointsSharp32);
      laserCloudSharpLast2.header.frame_id = "/camera";
      pubLaserCloudSharpLast.publish(laserCloudSharpLast2);

      sensor_msgs::PointCloud2 laserCloudFlatLast2;
      pcl::toROSMsg(*laserCloudFlatLast, laserCloudFlatLast2);
      laserCloudFlatLast2.header.stamp = ros::Time().fromSec(timeSurfPointsFlat32);
      laserCloudFlatLast2.header.frame_id = "/camera";
      pubLaserCloudFlatLast.publish(laserCloudFlatLast2);

      sensor_msgs::PointCloud2 laserCloudCornerLast2;
      pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
      laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeCornerPointsLessSharp32);
      laserCloudCornerLast2.header.frame_id = "/camera";
      pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

      sensor_msgs::PointCloud2 laserCloudSurfLast2;
      pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
      laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat32);
      laserCloudSurfLast2.header.frame_id = "/camera";
      pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

      gettimeofday(&t_end, NULL);
      std::cout<<"laserOdometry consuming time: "<<(t_end.tv_sec - t_start.tv_sec)*1000.0
                                                  + (t_end.tv_usec - t_start.tv_usec)/1000.0 <<" ms"<<std::endl;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

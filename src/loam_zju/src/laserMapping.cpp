#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

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
#include "loam_zju/parameterReader.h"

const double PI = 3.1415926;
double timeLaserCloudSharpLast = 0.0;
double timeLaserCloudFlatLast = 0.0;
double timeLaserCloudCornerLast = 0.0;
double timeLaserCloudSurfLast = 0.0;
double timeLaserOdometry = 0.0;

bool newLaserCloudSharpLast = false;
bool newLaserCloudFlatLast = false;
bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserOdometry = false;

PARAMETER_READER parameterReader("/root/catkin_SLAM/src/loam_zju/parameters/laserMappingPara.txt");

// laser mapping takes place in every skipFrameNum frames
const int skipFrameNum = (int)atof(parameterReader.getValue("skipFrameNum").c_str());
// map update every mapSkipFrameNum*skipFrameNum frames
const int mapSkipFrameNum = (int)atof(parameterReader.getValue("mapSkipFrameNum").c_str());
// the central cube position of current frame
int laserCloudCenWidth = (int)atof(parameterReader.getValue("laserCloudCenWidth").c_str());
int laserCloudCenHeight = (int)atof(parameterReader.getValue("laserCloudCenHeight").c_str());
int laserCloudCenDepth = (int)atof(parameterReader.getValue("laserCloudCenDepth").c_str());
// the cube number of 3 axis
const int laserCloudWidth = (int)atof(parameterReader.getValue("laserCloudWidth").c_str());
const int laserCloudHeight = (int)atof(parameterReader.getValue("laserCloudHeight").c_str());
const int laserCloudDepth = (int)atof(parameterReader.getValue("laserCloudDepth").c_str());
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
// the local is devided into cubes, each is of size cubeSize*cubeSize*cubeSize in m
const float cubeSize = atof(parameterReader.getValue("cubeSize").c_str());
// the query neighbor cube num, in finding overlapped cubes
const int queryNeighborCubeNum = (int)atof(parameterReader.getValue("queryNeighborCubeNum").c_str());
const int surroundingSize = (2*queryNeighborCubeNum+1)*(2*queryNeighborCubeNum+1)*(2*queryNeighborCubeNum+1);
// leaf size for voxel downsize
const double cornerLeafSize = atof(parameterReader.getValue("cornerLeafSize").c_str());
const double surfLeafSize = atof(parameterReader.getValue("surfLeafSize").c_str());
const double mapLeafSize = atof(parameterReader.getValue("mapLeafSize").c_str());

int *laserCloudValidInd;
int *laserCloudSurroundInd;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSharpLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFlatLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr *laserCloudCornerArray;
pcl::PointCloud<pcl::PointXYZI>::Ptr *laserCloudSurfArray;
pcl::PointCloud<pcl::PointXYZI>::Ptr *laserCloudCornerArray2;
pcl::PointCloud<pcl::PointXYZI>::Ptr *laserCloudSurfArray2;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());

float transformSum[6] = {0};        // current frame's pose generated from laser odometry
float transformBefMapped[6] = {0};  // initially the last frame's pose generated from laser odometry,
                                    // at the end of mapping, it is updated to current frame's transformSum
float transformAftMapped[6] = {0};	// initially the last frame's adjusted pose after laser mapping,
                                    // at the end of mapping, it is updated to current frame's transformTobeMapped
float transformTobeMapped[6] = {0}; // pose to be adjusted by laser mapping
float transformIncre[6] = {0};      // incremental value between last & current pose
float frameTrans[6] = {0};          // frame to frame transform, generated from laserOdometry


/* this function does one thing:
 *   Because of the inaccuracy of laserOdometry, the actual pose is
 *   not the same as transformSum. At the beginning of new transformSum's
 *   arrival, use the incremental transform between the current frame's
 *   transformSum and last frame's transformSum(i.e. transformBefMapped)
 *   to estimate current frame's actual pose, i.e. transformTobeMapped. And
 *   after that, the laser mapping algorithm adjusts the pose to be more
 *   accurate, denoted as transformAftMapped.
*/
void transformAssociateToMap()
{
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
           + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
            - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
            - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  transformTobeMapped[0] = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                 crycrx / cos(transformTobeMapped[0]));
  
  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
               - calx*calz*cblx*sblz);
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] = transformAftMapped[3] 
                         - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] = transformAftMapped[5] 
                         - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void transformUpdate()
{
  //transformTobeMapped[0] = 0.9 * transformTobeMapped[0] + 0.1 * transformSum[0];
  //transformTobeMapped[2] = 0.9 * transformTobeMapped[2] + 0.1 * transformSum[2];

  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
}

// reproject the point to the map cordinate, i.e. the world coordinate
void projectToMapCoordinate(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{ 
  // rotate around z axis
  float x1 = cos(transformTobeMapped[2]) * pi->x - sin(transformTobeMapped[2]) * pi->y;
  float y1 = sin(transformTobeMapped[2]) * pi->x + cos(transformTobeMapped[2]) * pi->y;
  float z1 = pi->z;
  // rotate around x axis
  float x2 = x1;
  float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
  // rotate around y axis, and translate
  po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2 + transformTobeMapped[3];
  po->y = y2 + transformTobeMapped[4];
  po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2 + transformTobeMapped[5];
  po->intensity = pi->intensity;
}

// reproject the point to local coordinate
void projectToLocalCoordinate(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  // 就是pointAssociateToMap中的逆变换
  float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3]) 
           - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
  float y1 = pi->y - transformTobeMapped[4];
  float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3]) 
           + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

  float x2 = x1;
  float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
  float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  po->x = cos(transformTobeMapped[2]) * x2
        + sin(transformTobeMapped[2]) * y2;
  po->y = -sin(transformTobeMapped[2]) * x2
        + cos(transformTobeMapped[2]) * y2;
  po->z = z2;
  po->intensity = pi->intensity;
}

// receive last frame's sharp points
void laserCloudSharpLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSharpLast2)
{
  timeLaserCloudSharpLast = laserCloudSharpLast2->header.stamp.toSec();

  laserCloudSharpLast->clear();
  pcl::fromROSMsg(*laserCloudSharpLast2, *laserCloudSharpLast);

  newLaserCloudSharpLast = true;
}

// receive last frame's flat points
void laserCloudFlatLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFlatLast2)
{
  timeLaserCloudFlatLast = laserCloudFlatLast2->header.stamp.toSec();

  laserCloudFlatLast->clear();
  pcl::fromROSMsg(*laserCloudFlatLast2, *laserCloudFlatLast);

  newLaserCloudFlatLast = true;
}

// receive last frame's corner points
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
  timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

  laserCloudCornerLast->clear();
  pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

  newLaserCloudCornerLast = true;
}

// receive last frame's surface points
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
  timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

  laserCloudSurfLast->clear();
  pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

  newLaserCloudSurfLast = true;
}

// receive last frame's pose generated from laser odometry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  timeLaserOdometry = laserOdometry->header.stamp.toSec();

  // 从四元数转成位置和朝向
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = laserOdometry->pose.pose.position.x;
  transformSum[4] = laserOdometry->pose.pose.position.y;
  transformSum[5] = laserOdometry->pose.pose.position.z;

  newLaserOdometry = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserMapping");

  // init parameters
  laserCloudValidInd = new int[surroundingSize];
  laserCloudSurroundInd = new int[surroundingSize];
  laserCloudCornerArray = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum];
  laserCloudSurfArray = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum];
  laserCloudCornerArray2 = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum];
  laserCloudSurfArray2 = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum];
  for (int i = 0; i < laserCloudNum; i++)
  {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }


  ros::NodeHandle nh;

  ros::Subscriber subLaserCloudSharpLast = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_sharp_last", 2, laserCloudSharpLastHandler);

  ros::Subscriber subLaserCloudFlatLast = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_flat_last", 2, laserCloudFlatLastHandler);

  ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);

  ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler);

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
                                     ("/laser_odom_to_init", 5, laserOdometryHandler);

  ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 1);
  ros::Publisher pubLaserCloudSharpForG2O = nh.advertise<sensor_msgs::PointCloud2> ("/laserCloudSharpForG2O", 2);
  ros::Publisher pubLaserCloudFlatForG2O = nh.advertise<sensor_msgs::PointCloud2> ("/laserCloudFlatForG2O", 2);
  ros::Publisher pubLaserCloudCornerForG2O = nh.advertise<sensor_msgs::PointCloud2> ("/laserCloudCornerForG2O", 2);
  ros::Publisher pubLaserCloudSurfForG2O = nh.advertise<sensor_msgs::PointCloud2> ("/laserCloudSurfForG2O", 2);

  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.child_frame_id = "/aft_mapped";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform aftMappedTrans;
  aftMappedTrans.frame_id_ = "/camera_init";
  aftMappedTrans.child_frame_id_ = "/aft_mapped";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::PointXYZI pointOri, pointSel, pointProj, coeff;

  cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;
  downSizeFilterCorner.setLeafSize(cornerLeafSize, cornerLeafSize, cornerLeafSize);  // cornerLeafSize is by default 0.2

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(surfLeafSize, surfLeafSize, surfLeafSize);    // surfLeafSize is by default 0.4

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(mapLeafSize, mapLeafSize, mapLeafSize);     // mapLeafSize is by default 0.6

  struct timeval t_start, t_end;
  int frameCount = skipFrameNum - 1;
  int mapFrameCount = mapSkipFrameNum - 1;
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    if (newLaserCloudSharpLast && newLaserCloudFlatLast && newLaserCloudCornerLast &&
        newLaserCloudSurfLast && newLaserOdometry &&
        fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudSharpLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudFlatLast - timeLaserOdometry) < 0.005 )
    {
      newLaserCloudSharpLast = false;
      newLaserCloudFlatLast = false;
      newLaserCloudCornerLast = false;
      newLaserCloudSurfLast = false;
      newLaserOdometry = false;

      frameCount++;
      if (frameCount >= skipFrameNum)
      {
        gettimeofday(&t_start, NULL);
        frameCount = 0;

        // step1. associate current frame's key points to map coordinate
        transformAssociateToMap();  // estimate the current pose

        *laserCloudCornerStack2 = *laserCloudCornerLast;
        *laserCloudSurfStack2 = *laserCloudSurfLast;

        // step2. choose overlapped cubes
        pcl::PointXYZI pointOnYAxis;
        pointOnYAxis.x = 0.0;
        pointOnYAxis.y = 10.0;
        pointOnYAxis.z = 0.0;
        projectToMapCoordinate(&pointOnYAxis, &pointOnYAxis);

        int centerCubeI = int((transformTobeMapped[3] + cubeSize/2.0) / cubeSize) + laserCloudCenWidth;
        int centerCubeJ = int((transformTobeMapped[4] + cubeSize/2.0) / cubeSize) + laserCloudCenHeight;
        int centerCubeK = int((transformTobeMapped[5] + cubeSize/2.0) / cubeSize) + laserCloudCenDepth;

        if (transformTobeMapped[3] + cubeSize/2.0 < 0) centerCubeI--;
        if (transformTobeMapped[4] + cubeSize/2.0 < 0) centerCubeJ--;
        if (transformTobeMapped[5] + cubeSize/2.0 < 0) centerCubeK--;

          // 2.1 adjust cubes to make the current pose(i.e. transformTobeMapped) be in the centrual area of it
        while (centerCubeI < 3)
        {
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = laserCloudWidth - 1;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; i >= 1; i--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI++;
          laserCloudCenWidth++;
          }

        while (centerCubeI >= laserCloudWidth - 3)
        {
          for (int j = 0; j < laserCloudHeight; j++)
          {
            for (int k = 0; k < laserCloudDepth; k++)
            {
              int i = 0;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; i < laserCloudWidth - 1; i++)
              {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI--;
          laserCloudCenWidth--;
        }

        while (centerCubeJ < 3)
        {
          for (int i = 0; i < laserCloudWidth; i++)
          {
            for (int k = 0; k < laserCloudDepth; k++)
            {
              int j = laserCloudHeight - 1;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; j >= 1; j--)
              {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeJ++;
          laserCloudCenHeight++;
        }

        while (centerCubeJ >= laserCloudHeight - 3)
        {
          for (int i = 0; i < laserCloudWidth; i++)
          {
            for (int k = 0; k < laserCloudDepth; k++)
            {
              int j = 0;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; j < laserCloudHeight - 1; j++)
              {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeJ--;
          laserCloudCenHeight--;
        }

        while (centerCubeK < 3)
        {
          for (int i = 0; i < laserCloudWidth; i++)
          {
            for (int j = 0; j < laserCloudHeight; j++)
            {
              int k = laserCloudDepth - 1;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; k >= 1; k--)
              {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

        centerCubeK++;
        laserCloudCenDepth++;
      }

        while (centerCubeK >= laserCloudDepth - 3)
        {
          for (int i = 0; i < laserCloudWidth; i++)
          {
            for (int j = 0; j < laserCloudHeight; j++)
            {
              int k = 0;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; k < laserCloudDepth - 1; k++)
              {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK--;
          laserCloudCenDepth--;
        }

          // 2.2 find overlapped cubes
        int laserCloudValidNum = 0;
        int laserCloudSurroundNum = 0;
        for (int i = centerCubeI - queryNeighborCubeNum; i <= centerCubeI + queryNeighborCubeNum; i++)
        {
          for (int j = centerCubeJ - queryNeighborCubeNum; j <= centerCubeJ + queryNeighborCubeNum; j++)
          {
            for (int k = centerCubeK - queryNeighborCubeNum; k <= centerCubeK + queryNeighborCubeNum; k++)
            {
              if (i >= 0 && i < laserCloudWidth &&
                  j >= 0 && j < laserCloudHeight &&
                  k >= 0 && k < laserCloudDepth)
              {
                float centerX = cubeSize * (i - laserCloudCenWidth);
                float centerY = cubeSize * (j - laserCloudCenHeight);
                float centerZ = cubeSize * (k - laserCloudCenDepth);

                bool isInLaserFOV = false;
                for (int ii = -1; ii <= 1; ii += 2)
                {
                  for (int jj = -1; jj <= 1; jj += 2)
                  {
                    for (int kk = -1; kk <= 1; kk += 2)
                    {
                      float cornerX = centerX + (cubeSize/2.0) * ii;
                      float cornerY = centerY + (cubeSize/2.0) * jj;
                      float cornerZ = centerZ + (cubeSize/2.0) * kk;

                      float squaredSide1 = (transformTobeMapped[3] - cornerX) 
                                         * (transformTobeMapped[3] - cornerX) 
                                         + (transformTobeMapped[4] - cornerY) 
                                         * (transformTobeMapped[4] - cornerY)
                                         + (transformTobeMapped[5] - cornerZ) 
                                         * (transformTobeMapped[5] - cornerZ);

                      float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) 
                                         + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                         + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                      float check1 = 100.0 + squaredSide1 - squaredSide2
                                   - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                      float check2 = 100.0 + squaredSide1 - squaredSide2
                                   + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                      if (check1 < 0 && check2 > 0) {
                        isInLaserFOV = true;
                      }
                    }
                  }
                }

                if (isInLaserFOV) {
                  laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j 
                                                       + laserCloudWidth * laserCloudHeight * k;
                  laserCloudValidNum++;
                }
                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j 
                                                             + laserCloudWidth * laserCloudHeight * k;
                laserCloudSurroundNum++;
              }
            }
          }
        }

        // step3. select reference key points and refine transformTobeMapped
          // 3.1 select reference key points in map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < laserCloudValidNum; i++) {
          *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
          *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }
        int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        laserCloudCornerStack2->clear();
        laserCloudSurfStack2->clear();

          // 3.2 refine transformTobeMapped, and update it to transformAftMapped
        if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
        {
          kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
          kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

          for (int iterCount = 0; iterCount < 10; iterCount++)
          {
            laserCloudOri->clear();
            //laserCloudSel->clear();
            //laserCloudCorr->clear();
            //laserCloudProj->clear();
            coeffSel->clear();

            // 3.2.1 select k-nearest neighbors, estimate a line, and calculate point-2-line distances
            for (int i = 0; i < laserCloudCornerStackNum; i++)
            {
              pointOri = laserCloudCornerStack->points[i];
              projectToMapCoordinate(&pointOri, &pointSel);  // in each iteration, reproject the keypoints to map coord again
              kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
              
              if (pointSearchSqDis[4] < 1.0)
              {
                float cx = 0;
                float cy = 0; 
                float cz = 0;
                for (int j = 0; j < 5; j++)
                {
                  cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                  cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                  cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5; 
                cz /= 5;

                float a11 = 0;
                float a12 = 0; 
                float a13 = 0;
                float a22 = 0;
                float a23 = 0; 
                float a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                  float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
                  float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
                  float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

                  a11 += ax * ax;
                  a12 += ax * ay;
                  a13 += ax * az;
                  a22 += ay * ay;
                  a23 += ay * az;
                  a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5; 
                a13 /= 5;
                a22 /= 5;
                a23 /= 5; 
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))  // if is line shape
                {
                  float x0 = pointSel.x;
                  float y0 = pointSel.y;
                  float z0 = pointSel.z;
                  float x1 = cx + 0.1 * matV1.at<float>(0, 0);  // generate 2 points on line
                  float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                  float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                  float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                  float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                  float z2 = cz - 0.1 * matV1.at<float>(0, 2);

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

                  float s = 1 - 0.9 * fabs(ld2);

                  coeff.x = s * la;
                  coeff.y = s * lb;
                  coeff.z = s * lc;
                  coeff.intensity = s * ld2;

                  if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    //laserCloudSel->push_back(pointSel);
                    //laserCloudProj->push_back(pointProj);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
                    coeffSel->push_back(coeff);
                  }
                }
              }
            }

            // 3.2.2 select k-nearest neighbors, estimate a plane, and calculate point-2-plane distances
            for (int i = 0; i < laserCloudSurfStackNum; i++)
            {
              pointOri = laserCloudSurfStack->points[i];
              projectToMapCoordinate(&pointOri, &pointSel);
              kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

              if (pointSearchSqDis[4] < 1.0)
              {
                for (int j = 0; j < 5; j++)
                {
                  matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                  matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                  matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0, 0);
                float pb = matX0.at<float>(1, 0);
                float pc = matX0.at<float>(2, 0);
                float pd = 1;
 
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                  if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                      pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                      pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
                  { // if any of these points is too far from the plane
                    planeValid = false;
                    break;
                  }
                }

                if (planeValid) {
                  float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                  pointProj = pointSel;
                  pointProj.x -= pa * pd2;
                  pointProj.y -= pb * pd2;
                  pointProj.z -= pc * pd2;

                  float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                          + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                  coeff.x = s * pa;
                  coeff.y = s * pb;
                  coeff.z = s * pc;
                  coeff.intensity = s * pd2;

                  if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    //laserCloudSel->push_back(pointSel);
                    //laserCloudProj->push_back(pointProj);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
                    coeffSel->push_back(coeff);
                  }
                }
              }
            }

            // 3.2.3 optimization
            float srx = sin(transformTobeMapped[0]);
            float crx = cos(transformTobeMapped[0]);
            float sry = sin(transformTobeMapped[1]);
            float cry = cos(transformTobeMapped[1]);
            float srz = sin(transformTobeMapped[2]);
            float crz = cos(transformTobeMapped[2]);

            int laserCloudSelNum = laserCloudOri->points.size();
            if (laserCloudSelNum < 50) {
              continue;
            }

            cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
            for (int i = 0; i < laserCloudSelNum; i++)
            {
              pointOri = laserCloudOri->points[i];
              coeff = coeffSel->points[i];

              float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                        + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                        + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

              float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                        + ((-cry*crz - srx*sry*srz)*pointOri.x 
                        + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

              float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                        + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                        + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

              matA.at<float>(i, 0) = arx;
              matA.at<float>(i, 1) = ary;
              matA.at<float>(i, 2) = arz;
              matA.at<float>(i, 3) = coeff.x;
              matA.at<float>(i, 4) = coeff.y;
              matA.at<float>(i, 5) = coeff.z;
              matB.at<float>(i, 0) = -coeff.intensity;
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
              float eignThre[6] = {100, 100, 100, 100, 100, 100};
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

            if (isDegenerate /*&& 0*/) {
              cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
              matX.copyTo(matX2);
              matX = matP * matX2;

              ROS_INFO ("laser mapping degenerate");
            }

            transformTobeMapped[0] += matX.at<float>(0, 0);
            transformTobeMapped[1] += matX.at<float>(1, 0);
            transformTobeMapped[2] += matX.at<float>(2, 0);
            transformTobeMapped[3] += matX.at<float>(3, 0);
            transformTobeMapped[4] += matX.at<float>(4, 0);
            transformTobeMapped[5] += matX.at<float>(5, 0);

            float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
                         + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
                         + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
            float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
                         + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
                         + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

            if (deltaR < 0.05 && deltaT < 0.05) {
              break;
            }

            //ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
          }

          transformUpdate();
        }

         // 3.3 add key points to map
         // 3.3.1 add corner points
        for (int i = 0; i < laserCloudCornerStackNum; i++)
        {
          projectToMapCoordinate(&laserCloudCornerStack->points[i], &pointSel);

          int cubeI = int((pointSel.x + (cubeSize/2.0)) / cubeSize) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + (cubeSize/2.0)) / cubeSize) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + (cubeSize/2.0)) / cubeSize) + laserCloudCenDepth;

          if (pointSel.x + (cubeSize/2.0) < 0) cubeI--;
          if (pointSel.y + (cubeSize/2.0) < 0) cubeJ--;
          if (pointSel.z + (cubeSize/2.0) < 0) cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth && 
              cubeJ >= 0 && cubeJ < laserCloudHeight && 
              cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudCornerArray[cubeInd]->push_back(pointSel);
          }
        }
          // 3.3.2 add surf points
        for (int i = 0; i < laserCloudSurfStackNum; i++)
        {
          projectToMapCoordinate(&laserCloudSurfStack->points[i], &pointSel);

          int cubeI = int((pointSel.x + (cubeSize/2.0)) / cubeSize) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + (cubeSize/2.0)) / cubeSize) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + (cubeSize/2.0)) / cubeSize) + laserCloudCenDepth;

          if (pointSel.x + (cubeSize/2.0) < 0) cubeI--;
          if (pointSel.y + (cubeSize/2.0) < 0) cubeJ--;
          if (pointSel.z + (cubeSize/2.0) < 0) cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth && 
              cubeJ >= 0 && cubeJ < laserCloudHeight && 
              cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudSurfArray[cubeInd]->push_back(pointSel);
          }
        }
          // 3.3.3 downsize filter
        for (int i = 0; i < laserCloudValidNum; i++)
        {
          int ind = laserCloudValidInd[i];

          laserCloudCornerArray2[ind]->clear();
          downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
          downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

          laserCloudSurfArray2[ind]->clear();
          downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
          downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
          laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
          laserCloudCornerArray2[ind] = laserCloudTemp;

          laserCloudTemp = laserCloudSurfArray[ind];
          laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
          laserCloudSurfArray2[ind] = laserCloudTemp;
        }

        // step4. publish the local 5*5*5 cubes' points to local surrounding map
        // this local map stores only the local 5*5*5 cubes' points, mainly for visualization
        // the bigger map is stored in laserCloudCornerArray & laserCloudSurfArray
        mapFrameCount++;
        if (mapFrameCount >= mapSkipFrameNum)
        {
          mapFrameCount = 0;
          laserCloudSurround2->clear();


//          for (int i = 0; i < laserCloudSurroundNum; i++) {
//            int ind = laserCloudSurroundInd[i];
//            *laserCloudSurround2 += *laserCloudCornerArray[ind];
//            *laserCloudSurround2 += *laserCloudSurfArray[ind];
//          }


//          for (int i = 0; i < laserCloudValidNum; i++) {
//            int ind = laserCloudValidInd[i];
//            *laserCloudSurround2 += *laserCloudCornerArray[ind];
//            *laserCloudSurround2 += *laserCloudSurfArray[ind];
//          }


          for (int i = 0; i < laserCloudNum; i++) {
            *laserCloudSurround2 += *laserCloudCornerArray[i];
            *laserCloudSurround2 += *laserCloudSurfArray[i];
          }

          laserCloudSurround->clear();
          downSizeFilterCorner.setInputCloud(laserCloudSurround2);
          downSizeFilterCorner.filter(*laserCloudSurround);

          sensor_msgs::PointCloud2 laserCloudSurround3;
          pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
          laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
          laserCloudSurround3.header.frame_id = "/camera_init";
          pubLaserCloudSurround.publish(laserCloudSurround3);
        }

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped[3];
        odomAftMapped.pose.pose.position.y = transformAftMapped[4];
        odomAftMapped.pose.pose.position.z = transformAftMapped[5];
        odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
        odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
        odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
        odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
        odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
        odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3],
                                             transformAftMapped[4], transformAftMapped[5]));
        tfBroadcaster.sendTransform(aftMappedTrans);

        // publish laserCloudCornerLast & laserCloudSurfLast
        sensor_msgs::PointCloud2 pcSharp;
        pcl::toROSMsg(*laserCloudSharpLast, pcSharp);
        pcSharp.header.stamp = ros::Time().fromSec(timeLaserCloudSharpLast);
        pcSharp.header.frame_id = "/camera";
        pubLaserCloudSharpForG2O.publish(pcSharp);

        sensor_msgs::PointCloud2 pcFlat;
        pcl::toROSMsg(*laserCloudFlatLast, pcFlat);
        pcFlat.header.stamp = ros::Time().fromSec(timeLaserCloudFlatLast);
        pcFlat.header.frame_id = "/camera";
        pubLaserCloudFlatForG2O.publish(pcFlat);

        sensor_msgs::PointCloud2 pcCorner;
        pcl::toROSMsg(*laserCloudCornerLast, pcCorner);
        pcCorner.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
        pcCorner.header.frame_id = "/camera";
        pubLaserCloudCornerForG2O.publish(pcCorner);

        sensor_msgs::PointCloud2 pcSurf;
        pcl::toROSMsg(*laserCloudSurfLast, pcSurf);
        pcSurf.header.stamp = ros::Time().fromSec(timeLaserCloudSurfLast);
        pcSurf.header.frame_id = "/camera";
        pubLaserCloudSurfForG2O.publish(pcSurf);

        gettimeofday(&t_end, NULL);
//        std::cout << "laserMapping consuming time: "<<(t_end.tv_sec - t_start.tv_sec)*1000.0
//                     + (t_end.tv_usec - t_start.tv_usec)/1000.0 << " ms" << std::endl;
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  // release memory
  delete[] laserCloudValidInd;
  delete[] laserCloudSurroundInd;
  delete[] laserCloudCornerArray;
  delete[] laserCloudSurfArray;
  delete[] laserCloudCornerArray2;
  delete[] laserCloudSurfArray2;
  return 0;
}

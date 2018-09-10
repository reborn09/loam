/* Created by : Linhui
 * Date       : 2016-12-08
 * Usage      : use g2o to optimize pose graph
*/
// std headers
#include <cmath>
#include <tuple>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>

// ros headers
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// opencv headers
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

// pcl headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

// g2o headers
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>

// local headers
#include "loam_zju/parameterReader.h"

using std::vector;
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;


const double PI = 3.1415926;
const int PointMaxNum = 40000;  // maximum number of points in a single point cloud

double timeLaserCloudSharp = 0.0;
double timeLaserCloudFlat = 0.0;
double timeLaserCloudCorner = 0.0;
double timeLaserCloudSurf = 0.0;
double timeLaserPose = 0.0;

bool newLaserCloudSharp = false;
bool newLaserCloudFlat = false;
bool newLaserCloudCorner = false;
bool newLaserCloudSurf = false;
bool newLaserPose = false;

struct TRANSFORM
{
  TRANSFORM():rx(0.0), ry(0.0), rz(0.0), tx(0.0), ty(0.0), tz(0.0){}
  TRANSFORM &operator = (const TRANSFORM &other)
  {
    rx = other.rx;
    ry = other.ry;
    rz = other.rz;
    tx = other.tx;
    ty = other.ty;
    tz = other.tz;
    return *this;
  }

  float rx;
  float ry;
  float rz;
  float tx;
  float ty;
  float tz;
};

vector< TRANSFORM > poseList;
vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > laserCloudSharpList;
vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > laserCloudFlatList;
vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > laserCloudCornerList;
vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > laserCloudSurfList;

pcl::PointCloud<pcl::PointXYZI> tmpCloudSharp;
pcl::PointCloud<pcl::PointXYZI> tmpCloudFlat;
pcl::PointCloud<pcl::PointXYZI> tmpCloudCorner;
pcl::PointCloud<pcl::PointXYZI> tmpCloudSurf;
TRANSFORM pose;

// parameter from config file
PARAMETER_READER parameterReader("/root/catkin_SLAM/src/loam_zju/parameters/poseGraphPara.txt");
const int neighborSize = (int)atof(parameterReader.getValue("neighborSize").c_str());
const float loopClosureSqDisThs = atof(parameterReader.getValue("loopClosureSqDisThs").c_str());
const double MaxCorrespondenceDistance = (double)atof(parameterReader.getValue("MaxCorrespondenceDistance").c_str());
const double TransformationEpsilon = (double)atof(parameterReader.getValue("TransformationEpsilon").c_str());
const double EuclideanFitnessEpsilon = (double)atof(parameterReader.getValue("EuclideanFitnessEpsilon").c_str());
const int MaximumIterations = (int)atof(parameterReader.getValue("MaximumIterations").c_str());
const bool g2oOpen = (atof(parameterReader.getValue("g2oOpen").c_str()) > 0 );
const int OptimizationSteps = (int)atof(parameterReader.getValue("OptimizationSteps").c_str());
const double confidence = atof(parameterReader.getValue("confidence").c_str());
const int MaxBeamForShow = (int)atof(parameterReader.getValue("MaxBeamForShow").c_str());
const int WaitTime = (int)atof(parameterReader.getValue("WaitTime").c_str());

const double NDTtransformEpsilon = (double)atof(parameterReader.getValue("NDTtransformEpsilon").c_str());
const double NDTStepSize = (double)atof(parameterReader.getValue("NDTStepSize").c_str());
const double NDTResolution = (double)atof(parameterReader.getValue("NDTResolution").c_str());
const int NDTMaxIterNum = (int)atof(parameterReader.getValue("NDTMaxIterNum").c_str());
const int addLoopClousure = (int)atof(parameterReader.getValue("addLoopClousure").c_str());
const double rotationThres = (double)atof(parameterReader.getValue("rotationThres").c_str());

// msgs handler
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSharpMsg)
{
  timeLaserCloudSharp = laserCloudSharpMsg->header.stamp.toSec();
  tmpCloudSharp.clear();
  pcl::fromROSMsg(*laserCloudSharpMsg, tmpCloudSharp);
  newLaserCloudSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFlatMsg)
{
  timeLaserCloudFlat = laserCloudFlatMsg->header.stamp.toSec();
  tmpCloudFlat.clear();
  pcl::fromROSMsg(*laserCloudFlatMsg, tmpCloudFlat);
  newLaserCloudFlat = true;
}

void laserCloudCornerHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerMsg)
{
  timeLaserCloudCorner = laserCloudCornerMsg->header.stamp.toSec();
  tmpCloudCorner.clear();
  pcl::fromROSMsg(*laserCloudCornerMsg, tmpCloudCorner);
  newLaserCloudCorner = true;
}

void laserCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfMsg)
{
  timeLaserCloudSurf = laserCloudSurfMsg->header.stamp.toSec();
  tmpCloudSurf.clear();
  pcl::fromROSMsg(*laserCloudSurfMsg, tmpCloudSurf);
  newLaserCloudSurf = true;
}

void laserPoseHandler(const nav_msgs::Odometry::ConstPtr& laserPoseMsg)
{
  timeLaserPose = laserPoseMsg->header.stamp.toSec();
  // 从四元数转成位置和朝向
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserPoseMsg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  pose.rx = -pitch;
  pose.ry = -yaw;
  pose.rz = roll;
  pose.tx = laserPoseMsg->pose.pose.position.x;
  pose.ty = laserPoseMsg->pose.pose.position.y;
  pose.tz = laserPoseMsg->pose.pose.position.z;

  newLaserPose = true;
}


////////////////////////////////////////////////////////////////////////////////
/// \brief caculate transform from pose1 to pose2
/// \param pose1
/// \param pose2
/// \return
////////////////////////////////////////////////////////////////////////////////
TRANSFORM TransformBetweenPoses(const TRANSFORM pose1, const TRANSFORM pose2)
{
  tf::Transform transform1, transform2, transform;
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(pose1.rz, -pose1.rx, -pose1.ry);
  transform1.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  transform1.setOrigin(tf::Vector3(pose1.tx, pose1.ty, pose1.tz));

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(pose2.rz, -pose2.rx, -pose2.ry);
  transform2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  transform2.setOrigin(tf::Vector3(pose2.tx, pose2.ty, pose2.tz));

  /* denote Xw as world coordinate, X1 as pose1 coordinate and X2 as pose2 coordinate,
   * T1 as pose1, T2 as pose2, then we have:
   *   Xw = T1 * X1
   *   Xw = T2 * X2
   * ---> T1 * X1 = T2 * X2
   * ---> X1 = (T1^-1 * T2) * X2
   * so, the transform from pose1 to pose2 is deltaT = T1^-1 * T2
  */
  transform = transform1.inverseTimes(transform2);  // deltaT = T1^-1 * T2

  double roll, pitch, yaw;
  tf::Quaternion geoQuat2 = transform.getRotation();
  tf::Vector3 position = transform.getOrigin();
  tf::Matrix3x3(tf::Quaternion(geoQuat2.z(), -geoQuat2.x(), -geoQuat2.y(), geoQuat2.w())).getRPY(roll, pitch, yaw);
  TRANSFORM transformDelta;
  transformDelta.rx = -pitch;
  transformDelta.ry = -yaw;
  transformDelta.rz = roll;
  transformDelta.tx = position.x();
  transformDelta.ty = position.y();
  transformDelta.tz = position.z();

  return transformDelta;
}


////////////////////////////////////////////////////////////////////////////////
/// \brief update pose2 from pose1 & transformDelta
/// \param posePrev: Previous pose
/// \param transformDelta: the transform between current pose & previous pose
/// \return updated current pose
////////////////////////////////////////////////////////////////////////////////
TRANSFORM UpdatePoseByTransform(const TRANSFORM posePrev, const TRANSFORM transformDelta)
{
  tf::Transform transform1, transform2, transform;
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(posePrev.rz, -posePrev.rx, -posePrev.ry);
  transform1.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  transform1.setOrigin(tf::Vector3(posePrev.tx, posePrev.ty, posePrev.tz));

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformDelta.rz, -transformDelta.rx, -transformDelta.ry);
  transform2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  transform2.setOrigin(tf::Vector3(transformDelta.tx, transformDelta.ty, transformDelta.tz));

  /* denote T1 as pose1, T2 as pose2, then we have:
   *   deltaT = T1^-1 * T2
   * now given deltaT & T1, we have:
   *   T2 = T1 * deltaT
  */
  transform = transform1.inverse().inverseTimes(transform2);  //

  double roll, pitch, yaw;
  tf::Quaternion geoQuat2 = transform.getRotation();
  tf::Vector3 position = transform.getOrigin();
  tf::Matrix3x3(tf::Quaternion(geoQuat2.z(), -geoQuat2.x(), -geoQuat2.y(), geoQuat2.w())).getRPY(roll, pitch, yaw);
  TRANSFORM poseCurrent;
  poseCurrent.rx = -pitch;
  poseCurrent.ry = -yaw;
  poseCurrent.rz = roll;
  poseCurrent.tx = position.x();
  poseCurrent.ty = position.y();
  poseCurrent.tz = position.z();

  return poseCurrent;
}


// project the pi to po at given transform
void projectWithTransform(pcl::PointXYZI *pi, pcl::PointXYZI *po, const TRANSFORM transform)
{
  // rotate around z axis
  float x1 = cos(transform.rz) * pi->x - sin(transform.rz) * pi->y;
  float y1 = sin(transform.rz) * pi->x + cos(transform.rz) * pi->y;
  float z1 = pi->z;
  // rotate around x axis
  float x2 = x1;
  float y2 = cos(transform.rx) * y1 - sin(transform.rx) * z1;
  float z2 = sin(transform.rx) * y1 + cos(transform.rx) * z1;
  // rotate around y axis, and translate
  po->x = cos(transform.ry) * x2 + sin(transform.ry) * z2 + transform.tx;
  po->y = y2 + transform.ty;
  po->z = -sin(transform.ry) * x2 + cos(transform.ry) * z2 + transform.tz;
  po->intensity = pi->intensity;
}


// an overload version
void projectWithTransform(pcl::PointXYZI *pi, pcl::PointXYZI *po, const Eigen::Matrix4f transform)
{
  Eigen::Vector4f p(pi->x, pi->y, pi->z, 1.0);
  Eigen::Vector4f q = transform * p;
  po->x = q(0);
  po->y = q(1);
  po->z = q(2);
  po->intensity = pi->intensity;
}


////////////////////////////////////////////////////////////////////////////////
/// \brief Vec2Matrix
/// \param transform
/// \return
////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Vec2Matrix(const TRANSFORM &transform)
{
  float rx = transform.rx;
  float ry = transform.ry;
  float rz = transform.rz;
  float tx = transform.tx;
  float ty = transform.ty;
  float tz = transform.tz;

  Eigen::Matrix4f Rx, Ry, Rz;
  Rx(0,0) = 1;        Rx(0,1) = 0;        Rx(0,2) = 0;          Rx(0,3) = 0;
  Rx(1,0) = 0;        Rx(1,1) = cos(rx);  Rx(1,2) = -sin(rx);   Rx(1,3) = 0;
  Rx(2,0) = 0;        Rx(2,1) = sin(rx);  Rx(2,2) = cos(rx);    Rx(2,3) = 0;
  Rx(3,0) = 0;        Rx(3,1) = 0;        Rx(3,2) = 0;          Rx(3,3) = 1;

  Ry(0,0) = cos(ry);  Ry(0,1) = 0;        Ry(0,2) = sin(ry);    Ry(0,3) = 0;
  Ry(1,0) = 0;        Ry(1,1) = 1;        Ry(1,2) = 0;          Ry(1,3) = 0;
  Ry(2,0) = -sin(ry); Ry(2,1) = 0;        Ry(2,2) = cos(ry);    Ry(2,3) = 0;
  Ry(3,0) = 0;        Ry(3,1) = 0;        Ry(3,2) = 0;          Ry(3,3) = 1;

  Rz(0,0) = cos(rz);  Rz(0,1) = -sin(rz); Rz(0,2) = 0;          Rz(0,3) = 0;
  Rz(1,0) = sin(rz);  Rz(1,1) = cos(rz);  Rz(1,2) = 0;          Rz(1,3) = 0;
  Rz(2,0) = 0;        Rz(2,1) = 0;        Rz(2,2) = 1;          Rz(2,3) = 0;
  Rz(3,0) = 0;        Rz(3,1) = 0;        Rz(3,2) = 0;          Rz(3,3) = 1;

  Eigen::Matrix4f result = Ry*Rx*Rz;  // R = Ry*Rx*Rz
  result(0,3) = tx;   result(1,3) = ty;   result(2,3) = tz;

  return result;
}


////////////////////////////////////////////////////////////////////////////////
/// \brief NDTAlignment: use NDT algorithm to align two clouds
/// \param srcInd: source cloud index
/// \param dstInd: target cloud index
/// \param setInitGuess: whether or not use the poses transform as initial guess
/// \return transform matrix from source cloud to target cloud
////////////////////////////////////////////////////////////////////////////////
std::tuple<Eigen::Matrix4f, double> NDTAlignment(const int srcInd, const int dstInd, const bool setInitGuess)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr referenceCloud(new pcl::PointCloud<pcl::PointXYZI>);
  srcCloud->clear();
  referenceCloud->clear();
  *srcCloud += *(laserCloudCornerList[srcInd]);
  *srcCloud += *(laserCloudSurfList[srcInd]);
  *referenceCloud += *(laserCloudCornerList[dstInd]);
  *referenceCloud += *(laserCloudSurfList[dstInd]);

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(NDTtransformEpsilon);
  ndt.setStepSize(NDTStepSize);
  ndt.setResolution(NDTResolution);
  ndt.setMaximumIterations(NDTMaxIterNum);
  ndt.setInputSource(srcCloud);
  ndt.setInputTarget(referenceCloud);

  Eigen::Matrix4f calculateFromPoses = Vec2Matrix(poseList[dstInd]).inverse() * Vec2Matrix(poseList[srcInd]);
  Eigen::Matrix4f init_guess;
  if(setInitGuess)
  {
    init_guess = calculateFromPoses;
  }
  else
  {
    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0,0,0);
    init_guess = (init_translation*init_rotation).matrix();
  }
  pcl::PointCloud<pcl::PointXYZI> output;
  ndt.align(output, init_guess);
  std::cout<<"---ndt has converged: "<<ndt.hasConverged()<<", score: "<<ndt.getFitnessScore()<<std::endl;

//  if(ndt.hasConverged())
//    return ndt.getFinalTransformation();
//  else
//    return calculateFromPoses;
  return std::make_tuple(ndt.getFinalTransformation(), ndt.getFitnessScore());
}


////////////////////////////////////////////////////////////////////////////////
/// \brief ICPAlignment: use ICP algorithm to align two clouds
/// \param srcInd: source cloud index
/// \param dstInd: target cloud index
/// \return transform matrix from source cloud to target cloud
////////////////////////////////////////////////////////////////////////////////
std::tuple<Eigen::Matrix4f, double> ICPAlignment(const int srcInd, const int dstInd)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr referenceCloud(new pcl::PointCloud<pcl::PointXYZI>);
  srcCloud->clear();
  referenceCloud->clear();
  *srcCloud += *(laserCloudCornerList[srcInd]);
  *srcCloud += *(laserCloudSurfList[srcInd]);
  *referenceCloud += *(laserCloudCornerList[dstInd]);
  *referenceCloud += *(laserCloudSurfList[dstInd]);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
  icp.setTransformationEpsilon(TransformationEpsilon);
  icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
  icp.setMaximumIterations(MaximumIterations);
  icp.setInputSource(srcCloud);
  icp.setInputTarget(referenceCloud);
  pcl::PointCloud<pcl::PointXYZI> output;
  icp.align(output);
  std::cout<<"---icp has converged: "<<icp.hasConverged()<<", score: "<<icp.getFitnessScore()<<std::endl;

  return std::make_tuple(icp.getFinalTransformation(), icp.getFitnessScore());
}


////////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "poseGraphOptimization");

  ros::NodeHandle nh;
  ros::Subscriber subLaserCloudSharp = nh.subscribe<sensor_msgs::PointCloud2>
                                           ("/laserCloudSharpForG2O", 2, laserCloudSharpHandler);
  ros::Subscriber subLaserCloudFlat = nh.subscribe<sensor_msgs::PointCloud2>
                                           ("/laserCloudFlatForG2O", 2, laserCloudFlatHandler);
  ros::Subscriber subLaserCloudCorner = nh.subscribe<sensor_msgs::PointCloud2>
                                           ("/laserCloudCornerForG2O", 2, laserCloudCornerHandler);
  ros::Subscriber subLaserCloudSurf = nh.subscribe<sensor_msgs::PointCloud2>
                                           ("/laserCloudSurfForG2O", 2, laserCloudSurfHandler);
  ros::Subscriber subLaserPose = nh.subscribe<nav_msgs::Odometry>
                                           ("/aft_mapped_to_init", 5, laserPoseHandler);

  ros::Publisher pubLaserCloudICP = nh.advertise<sensor_msgs::PointCloud2> ("/tmpCloudICP", 2);
  ros::Publisher pubLaserCloudDst = nh.advertise<sensor_msgs::PointCloud2> ("/tmpCloudDst", 2);
  ros::Publisher pubLaserCloudSrc = nh.advertise<sensor_msgs::PointCloud2> ("/tmpCloudSrc", 2);
  ros::Publisher pubAffinedMap = nh.advertise<sensor_msgs::PointCloud2> ("/affinedMap", 1);

  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorner1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorner2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurf1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurf2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr affinedMap(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr affinedMap2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(0.2, 0.2, 0.2);

  // initialize g2o optimizer
  SlamLinearSolver *linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  g2o::SparseOptimizer globalOptimizer;
  globalOptimizer.setAlgorithm(solver);
  globalOptimizer.setVerbose(false);  // no debug info

  Eigen::Isometry3d prevPose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d curPose =  Eigen::Isometry3d::Identity();
  Eigen::Matrix4f tmpPose;

  struct timeval t_start, t_end;
  const double freq = 100.0;  // 100 Hz spin rate
  const int OptimizationWait = WaitTime * freq; // if no new msg for 4 seconds, then perform ICP
  int counter = 0;
  bool loopClosed = false;
  ros::Rate rate(freq);
  bool status = ros::ok();
  int frameInd = 0;
  while(status)
  {
    ros::spinOnce();
    counter++;

    if( newLaserCloudCorner && newLaserCloudSurf && newLaserPose
        && newLaserCloudSharp && newLaserCloudFlat
        && fabs(timeLaserCloudCorner-timeLaserPose) < 0.005
        && fabs(timeLaserCloudSurf-timeLaserPose) < 0.005
        && fabs(timeLaserCloudSharp-timeLaserPose) < 0.005
        && fabs(timeLaserCloudFlat-timeLaserPose) < 0.005 )
    {
      gettimeofday(&t_start, NULL);      

      // reset time counter
      counter = 0;
      newLaserCloudSharp = false;
      newLaserCloudFlat = false;
      newLaserCloudCorner = false;
      newLaserCloudSurf = false;
      newLaserPose = false;

      // store pointcloud & corresponding pose
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcSharpPtr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcFlatPtr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcCornerPtr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcSurfPtr(new pcl::PointCloud<pcl::PointXYZI>);
      *pcSharpPtr = tmpCloudSharp;
      *pcFlatPtr = tmpCloudFlat;
      *pcCornerPtr = tmpCloudCorner;
      *pcSurfPtr = tmpCloudSurf;
      laserCloudSharpList.push_back(pcSharpPtr);
      laserCloudFlatList.push_back(pcFlatPtr);
      laserCloudCornerList.push_back(pcCornerPtr);
      laserCloudSurfList.push_back(pcSurfPtr);
      poseList.push_back(pose);

      // add first vertex
      if(frameInd == 0)
      {
        g2o::VertexSE3 *v0 = new g2o::VertexSE3();
        v0->setId(0);
        tmpPose = Vec2Matrix(poseList[0]);
        for(int i=0; i<4; i++)
          for(int j=0; j<4; j++)
            prevPose(i,j) = tmpPose(i,j);
        v0->setEstimate(prevPose);
        v0->setFixed(true);
        globalOptimizer.addVertex(v0);
      }
      else
      {
        //
//        TRANSFORM delta = TransformBetweenPoses(poseList[frameInd-1], poseList[frameInd]);
//        if(fabs(delta.rx) > rotationThres ||  fabs(delta.ry) > rotationThres || fabs(delta.rz) > rotationThres)
//          continue;

        // add vertex
        g2o::VertexSE3 *vi = new g2o::VertexSE3();
        vi->setId(frameInd);
        curPose = Eigen::Isometry3d::Identity();
        tmpPose = Vec2Matrix(poseList[frameInd]);
        for(int i=0; i<4; i++)
          for(int j=0; j<4; j++)
            curPose(i,j) = tmpPose(i,j);
        vi->setEstimate(curPose);
//        if(frameInd < 80)
//           vi->setFixed(true);
        globalOptimizer.addVertex(vi);

        // add edge for neighbors
        for(int cnt = 1; cnt <= neighborSize; cnt++)
        {
          int referenceInd = frameInd - cnt;
          if(referenceInd < 0)
            break;

          g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
          g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
          edge->vertices()[0] = globalOptimizer.vertex(referenceInd);
          edge->vertices()[1] = globalOptimizer.vertex(frameInd);
          edge->setRobustKernel(robustKernel);

//            // estimation of edge
//          Eigen::Matrix4f poseTransform;
//          double icpScore;
//          std::tie(poseTransform, icpScore) = ICPAlignment(frameInd, referenceInd);
//          // information matrix
//          Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
//          information(0,0) = information(1,1) = information(2,2) =
//          information(3,3) = information(4,4) = information(5,5) = 1.0/icpScore;
//          edge->setInformation(information);

             // estimation of edge
          Eigen::Matrix4f poseTransform = Vec2Matrix(poseList[referenceInd]).inverse() * Vec2Matrix(poseList[frameInd]);
            // information matrix
          Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
          information(0,0) = information(1,1) = information(2,2) = confidence;
          information(3,3) = information(4,4) = information(5,5) = confidence;
          edge->setInformation(information);


          Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
          for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
              T(i,j) = poseTransform(i,j);
          edge->setMeasurement(T);
          globalOptimizer.addEdge(edge);
        }

      }

      frameInd++;
      gettimeofday(&t_end, NULL);
      std::cout << "Frame: " << frameInd << ", update graph time: "<<(t_end.tv_sec - t_start.tv_sec)*1000.0
                   + (t_end.tv_usec - t_start.tv_usec)/1000.0 << " ms" << std::endl;
    }

    // assume the last & first frame forms loop closure, use g2o to optimize pose graph and generate new map
    if(g2oOpen && !loopClosed && frameInd > 0 && counter >= OptimizationWait)
    {
      gettimeofday(&t_start, NULL);
      loopClosed = true;
      counter = 0;
      frameInd--;

      if(addLoopClousure)
      {
        // alignment
        Eigen::Matrix4f loopError;
        double icpScore;
        std::tie(loopError, icpScore) = ICPAlignment(frameInd, 0);


        // publish for visualization of ndt result
        laserCloudSurf1->clear();
        laserCloudSurf2->clear();
        *laserCloudSurf1 = *laserCloudSurfList[0];
        *laserCloudSurf2 = *laserCloudSurfList[frameInd];
        int laserCloudSurfNum1 = laserCloudSurf1->points.size();
        int laserCloudSurfNum2 = laserCloudSurf2->points.size();

        for(int i=0; i<laserCloudSurfNum2; i++)
          projectWithTransform(&laserCloudSurf2->points[i], &laserCloudSurf2->points[i], loopError);
        Eigen::Matrix4f pose0 = Vec2Matrix(poseList[0]);
        for(int i=0; i<laserCloudSurfNum1; i++)
          projectWithTransform(&laserCloudSurf1->points[i], &laserCloudSurf1->points[i], pose0);
        for(int i=0; i<laserCloudSurfNum2; i++)
          projectWithTransform(&laserCloudSurf2->points[i], &laserCloudSurf2->points[i], pose0);


        sensor_msgs::PointCloud2 laserCloudSurfMsg1;
        pcl::toROSMsg(*laserCloudSurf1, laserCloudSurfMsg1);
        laserCloudSurfMsg1.header.stamp = ros::Time().fromSec(timeLaserCloudSurf);
        laserCloudSurfMsg1.header.frame_id = "/camera_init";
        pubLaserCloudDst.publish(laserCloudSurfMsg1);

        sensor_msgs::PointCloud2 outputMsg2;
        pcl::toROSMsg(*laserCloudSurf2, outputMsg2);
        outputMsg2.header.stamp = ros::Time().fromSec(timeLaserCloudSurf);
        outputMsg2.header.frame_id = "/camera_init";
        pubLaserCloudICP.publish(outputMsg2);


        // add loop clousure edge to g2o otimizer
        g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->vertices()[0] = globalOptimizer.vertex(0);
        edge->vertices()[1] = globalOptimizer.vertex(frameInd);
        edge->setRobustKernel(robustKernel);

        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        information(0,0) = information(1,1) = information(2,2) = confidence;
        information(3,3) = information(4,4) = information(5,5) = confidence;
        edge->setInformation(information);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for(int i = 0; i < 4; i++)
          for(int j = 0; j< 4; j++)
            T(i, j) = loopError(i, j);
        edge->setMeasurement(T);
        globalOptimizer.addEdge(edge);
      }


      // optimize
      std::cout << " optimization...";
      globalOptimizer.save("/root/catkin_SLAM/src/loam_cmu/g2o_result/result_before.g2o");
      globalOptimizer.initializeOptimization();
      globalOptimizer.optimize(OptimizationSteps);
      globalOptimizer.save("/root/catkin_SLAM/src/loam_cmu/g2o_result/result_after.g2o");

      // reconstruct map
      affinedMap2->clear();
      pcl::PointCloud<pcl::PointXYZI>::Ptr keyFrame(new pcl::PointCloud<pcl::PointXYZI>);
      for(int cnt=0; cnt<globalOptimizer.vertices().size()/*frameInd*/; cnt++)
      {
        Eigen::Matrix4f newPose;
        g2o::VertexSE3 *vetx = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(cnt));
        Eigen::Isometry3d estimatePose = vetx->estimate();

        for(int i=0; i<4; i++)
          for(int j=0; j<4; j++)
            newPose(i, j) = estimatePose(i, j);
        keyFrame->clear();
        *keyFrame = *laserCloudSurfList[vetx->id()];
        int keyFrameNum = keyFrame->points.size();
        for(int i = 0; i<keyFrameNum; i++)
        {
          if((int)keyFrame->points[i].intensity < MaxBeamForShow)
          {
            projectWithTransform(&keyFrame->points[i], &keyFrame->points[i], newPose);
            affinedMap2->points.push_back(keyFrame->points[i]);
          }
        }
      }

      // publish reconstructed map
      affinedMap->clear();
      downSizeFilterMap.setInputCloud(affinedMap2);
      downSizeFilterMap.filter(*affinedMap);
      sensor_msgs::PointCloud2 affinedMapMsg;
      pcl::toROSMsg(*affinedMap, affinedMapMsg);
      affinedMapMsg.header.stamp = ros::Time().fromSec(timeLaserCloudSurf);
      affinedMapMsg.header.frame_id = "/camera_init";
      pubAffinedMap.publish(affinedMapMsg);

      globalOptimizer.clear();
      gettimeofday(&t_end, NULL);
      std::cout << "...done." << std::endl;
      std::cout << "ICP & optimization time: "<<(t_end.tv_sec - t_start.tv_sec)*1000.0
                   + (t_end.tv_usec - t_start.tv_usec)/1000.0 << " ms" << std::endl;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

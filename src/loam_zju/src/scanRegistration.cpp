// C++ headers
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <time.h>
// ROS headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// OpenCV headers
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
// PCL headers
#include <pcl_conversions/pcl_conversions.h> // 点云格式转化
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>	//	定义了一系列可用的pointT类型
#include <pcl/filters/voxel_grid.h> // 过滤算法
#include <pcl/kdtree/kdtree_flann.h> // K-D TREE查找算法

//#define Laser16_enable

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud32(new pcl::PointCloud<pcl::PointXYZI>());	// float x, float y, float z, float intensity
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud16Left(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud16Right(new pcl::PointCloud<pcl::PointXYZI>());
// laser 32
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp32(new pcl::PointCloud<pcl::PointXYZI>());	// 急拐角点
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp32(new pcl::PointCloud<pcl::PointXYZI>());	// 不那么急的拐角点
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat32(new pcl::PointCloud<pcl::PointXYZI>());	// 平滑面点
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat32(new pcl::PointCloud<pcl::PointXYZI>());	// 实际上是surfPointsFlat降采样得到的集合
// laser16 left
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp16Left(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp16Left(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat16Left(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat16Left(new pcl::PointCloud<pcl::PointXYZI>());
// laser16 right
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp16Right(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp16Right(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat16Right(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat16Right(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>()); // 用于surfPointsFlat降采样
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZI>()); // 用于surfPointsFlat降采样

// 4个发布对象，分别发布角点、弱角点、面点、弱面点
ros::Publisher* pubCornerPointsSharpPointer;
ros::Publisher* pubCornerPointsLessSharpPointer;
ros::Publisher* pubSurfPointsFlatPointer;
ros::Publisher* pubSurfPointsLessFlatPointer;

bool newLidar32 = false;
bool newLidar16Left =false;
bool newLidar16Right = false;
int timeLidar32;
int timeLidar16Left;
int timeLidar16Right;
std_msgs::Header Lidar32Header;
std_msgs::Header Lidar16LeftHeader;
std_msgs::Header Lidar16RightHeader;

const int MAX_POINTNUM64 = 120000;
const int MAX_POINTNUM32 = 60000;
const int MAX_POINTNUM16L = 20000;
const int MAX_POINTNUM16R = 20000;
const int WAIT_FRAME_NUM = 20;   // WARNNING: wait 5 frames for system initialization, otherwise laserOdometry will crash, don't know why, fuck!

struct EXTRINSIC_PARA
{
  EXTRINSIC_PARA()
  {
    R[0][0] = 1.0; R[0][1] = 0.0; R[0][2] = 0.0;
    R[1][0] = 0.0; R[1][1] = 1.0; R[1][2] = 0.0;
    R[2][0] = 0.0; R[2][1] = 0.0; R[2][2] = 1.0;
    T[0] = 0.0; T[1] = 0.0;  T[2] = 0.0;
  }
  float R[3][3];
  float T[3];
};

void init_lidar16_para(EXTRINSIC_PARA &lidar16ExParaLeft, EXTRINSIC_PARA &lidar16ExParaRight)
{
    string lidar16para_L_filename = "/root/ZJUALV/bin/parameters/lidar16para_L.ini";
    string lidar16para_R_filename = "/root/ZJUALV/bin/parameters/lidar16para_R.ini";
    ifstream infile;
    stringstream sline;
    string line;

    // left
    infile.open(lidar16para_L_filename, ios::in);
    if(!infile)
    {
        cout<<"***Error: can't open vlp16-L para file \""<<lidar16para_L_filename<<"\""<<endl;
        return;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(std::getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    std::getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>lidar16ExParaLeft.R[i][0]>>lidar16ExParaLeft.R[i][1]>>lidar16ExParaLeft.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                std::getline(infile, line);
                sline.str("");  // remember to clear!!!!!
                sline.clear();
                sline<<line;
                sline>>lidar16ExParaLeft.T[0]>>lidar16ExParaLeft.T[1]>>lidar16ExParaLeft.T[2];
                break;
            }
        }
        line.clear();
    }
    infile.close();

    // right
    infile.open(lidar16para_R_filename, ios::in);
    if(!infile)
    {
        cout<<"***Error: can't open vlp16-R para file \""<<lidar16para_R_filename<<"\""<<endl;
        return;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(std::getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    std::getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>lidar16ExParaRight.R[i][0]>>lidar16ExParaRight.R[i][1]>>lidar16ExParaRight.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                std::getline(infile, line);
                sline.str("");  // remember to clear!!!!!
                sline.clear();
                sline<<line;
                sline>>lidar16ExParaRight.T[0]>>lidar16ExParaRight.T[1]>>lidar16ExParaRight.T[2];
            }

        }
        line.clear();
    }
    infile.close();
}

void laserCloud32Handler(const sensor_msgs::PointCloud2ConstPtr& laserCloud32In)
{
  timeLidar32 = laserCloud32In->header.stamp.toSec();
  Lidar32Header = laserCloud32In->header;
  laserCloud32->clear();

  pcl::fromROSMsg(*laserCloud32In, *laserCloud32);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloud32,*laserCloud32, indices);
  newLidar32= true;
}

void laserCloud16LeftHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud16LIn)
{
  timeLidar16Left = laserCloud16LIn->header.stamp.toSec();
  Lidar16LeftHeader = laserCloud16LIn->header;
  laserCloud16Left->clear();

  pcl::fromROSMsg(*laserCloud16LIn, *laserCloud16Left);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloud16Left,*laserCloud16Left, indices);
  newLidar16Left = true;
}

void laserCloud16RightHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud16RIn)
{
  timeLidar16Right = laserCloud16RIn->header.stamp.toSec();
  Lidar16RightHeader = laserCloud16RIn->header;
  laserCloud16Right->clear();

  pcl::fromROSMsg(*laserCloud16RIn, *laserCloud16Right);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloud16Right,*laserCloud16Right, indices);
  newLidar16Right= true;
}


//////////////////////////////////////////////////////////////////////////////////////
/// \brief ExtractKeyPoints
/// \param laserCloud
/// \param N_SCANS
/// \param NSegments
/// \param MAX_POINTSNUM
/// \param cornerPointsSharp
/// \param cornerPointsLessSharp
/// \param surfPointsFlat
/// \param surfPointsLessFlat
/// \param exPara
//////////////////////////////////////////////////////////////////////////////////////
void ExtractKeyPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloud,
                      const int N_SCANS,
                      const int NSegments,
                      const int MAX_POINTSNUM,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cornerPointsSharp,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cornerPointsLessSharp,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &surfPointsFlat,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &surfPointsLessFlat
                      )
{
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  std::vector<float> cloudCurvature(MAX_POINTSNUM, 0);    // 每个点平滑度
  std::vector<int> cloudSortInd(MAX_POINTSNUM, 0);        // 排序序号查找表，cloudSortInd[i]表示第i大的点对应在原始点云中的序号
  std::vector<int> cloudNeighborPicked(MAX_POINTSNUM, 0); // cloudeNeighborPicked[i]表示点i要么是几乎平行与激光方向，要么是遮挡造成的跳跃点，总之是不可靠特征点
  std::vector<int> cloudLabel(MAX_POINTSNUM, 0);

  int cloudSize = laserCloud->points.size();

  // step1. 计算每个点的平滑度，并且得到每个beam起始、结尾点序号的查找表
  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++)
  {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;

    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ; // 10个向量之和的模的平方
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    if (int(laserCloud->points[i].intensity) != scanCount)
    {	// 实际上是预先用intensity的整数部分来存储这个点属于第几圈的信息
      scanCount = int(laserCloud->points[i].intensity);
      if (scanCount > 0)
      {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }
  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;



  for (int i = 5; i < cloudSize - 6; i++)
  {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;	// 邻接点距离平方

    // 标记由遮挡造成的无效候选点
    if (diff > 0.1)
    {
      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2)
      {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1; // 实际上diffX, diffY, diffZ都是将更远的点按照光
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1; // 追踪法移到更近点相同的深度计算的
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
        {  // 实际上就是tan值，这个值很小意味着两个点是相邻点
          cloudNeighborPicked[i - 5] = 1;	// cloudNeighborPicked表示这个点不能再被选取为特征点
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      }
      else
      {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
        {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    // 标记几乎平行于扫描线的平面上的无效候选点
    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) // roughly parallel plane
    {
      cloudNeighborPicked[i] = 1;
    }
  }

  // step2. extract key points
  for (int i = 0; i < N_SCANS; i++)
  {
    surfPointsLessFlatScan->clear();
    for (int j = 0; j < NSegments; j++)
    {
      int sp = (scanStartInd[i] * (NSegments - j)  + scanEndInd[i] * j) / NSegments;			// 将每一个beam分成6等份，sp为每一等分的头
      int ep = (scanStartInd[i] * (NSegments - 1 - j)  + scanEndInd[i] * (j + 1)) / NSegments - 1;	// ep为每一等分的尾

      for (int k = sp + 1; k <= ep; k++)
      {
        for (int l = k; l >= sp + 1; l--)
        {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]])
          {		// 每一等分内插入增序排序，排的是序号
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      // 挑选角点
      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--)
      {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
        {
          largestPickedNum++;
          if (largestPickedNum <= 2) // 最大的2个角点label=2；第2~20个角点label=1，可能是用于多层次选点
          {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(laserCloud->points[ind]);
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          }
          else if (largestPickedNum <= 20)
          {
                cloudLabel[ind] = 1;
                cornerPointsLessSharp->push_back(laserCloud->points[ind]);
              }
          else
          {
            break;
          }

          cloudNeighborPicked[ind] = 1; // 标记一下，此点不可再选
          // 对于选取的特征点，检查其临近左右5个点，如果点与点之间距离很近，说明是在一个物体上连续扫描得到的，因此标记这些点不能再选为特征点
          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      // 挑选面点
      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++)
      {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
        {
          cloudLabel[ind] = -1;	// 面点标记为-1
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) // 每一等分上，面点不超过4个
          {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        if (cloudLabel[k] <= 0)
        {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    surfPointsLessFlatScanDS->clear();
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);// 降采样

    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
}


//////////////////////////////////////////////////////////////////////////////////////
/// \brief ProjectPoints
/// \param laserCloud
/// \param exPara
//////////////////////////////////////////////////////////////////////////////////////
void ProjectPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloud,
                     const EXTRINSIC_PARA &exPara)
{
  int cloudNum = laserCloud->points.size();
  for(int i = 0; i < cloudNum; i++)
  {
    pcl::PointXYZI tmp = laserCloud->points[i];
    laserCloud->points[i].x = (exPara.R[0][0] * tmp.x * 100.0 + exPara.R[0][1] * tmp.y * 100.0 + exPara.R[0][2] * tmp.z * 100.0 + exPara.T[0]) / 100.0;
    laserCloud->points[i].y = (exPara.R[1][0] * tmp.x * 100.0 + exPara.R[1][1] * tmp.y * 100.0 + exPara.R[1][2] * tmp.z * 100.0 + exPara.T[1]) / 100.0;
    laserCloud->points[i].z = (exPara.R[2][0] * tmp.x * 100.0 + exPara.R[2][1] * tmp.y * 100.0 + exPara.R[2][2] * tmp.z * 100.0 + exPara.T[2]) / 100.0;
    laserCloud->points[i].intensity = tmp.intensity;
  }
}

//////////////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
//////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // init
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  EXTRINSIC_PARA lidar16ExParaLeft, lidar16ExParaRight;
  init_lidar16_para(lidar16ExParaLeft, lidar16ExParaRight);

  // declaration
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>	// 订阅velodyne_points话题，接收PointCloud2消息
                                  ("/velodyne_points", 2, laserCloud32Handler);

  ros::Subscriber subLaserCloud16L = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/lidar16_L_points", 2, laserCloud16LeftHandler);

  ros::Subscriber subLaserCloud16R = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/lidar16_R_points", 2, laserCloud16RightHandler);
    // lidar32
  ros::Publisher pubLaserCloud32 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_32", 2);

  ros::Publisher pubCornerPointsSharp32 = nh.advertise<sensor_msgs::PointCloud2>	// 发布角点
                                        ("/laser_cloud_sharp_32", 2);

  ros::Publisher pubCornerPointsLessSharp32 = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱角点
                                            ("/laser_cloud_less_sharp_32", 2);

  ros::Publisher pubSurfPointsFlat32 = nh.advertise<sensor_msgs::PointCloud2>  // 发布面点
                                       ("/laser_cloud_flat_32", 2);

  ros::Publisher pubSurfPointsLessFlat32 = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱面点
                                           ("/laser_cloud_less_flat_32", 2);
    // lidar16 left
  ros::Publisher pubLaserCloud16Left = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_16Left", 2);
  ros::Publisher pubCornerPointsSharp16Left = nh.advertise<sensor_msgs::PointCloud2>	// 发布角点
                                        ("/laser_cloud_sharp_16Left", 2);

  ros::Publisher pubCornerPointsLessSharp16Left = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱角点
                                            ("/laser_cloud_less_sharp_16Left", 2);

  ros::Publisher pubSurfPointsFlat16Left = nh.advertise<sensor_msgs::PointCloud2>  // 发布面点
                                       ("/laser_cloud_flat_16Left", 2);

  ros::Publisher pubSurfPointsLessFlat16Left = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱面点
                                           ("/laser_cloud_less_flat_16Left", 2);
    // lidar16 right
  ros::Publisher pubLaserCloud16Right = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_16Right", 2);
  ros::Publisher pubCornerPointsSharp16Right = nh.advertise<sensor_msgs::PointCloud2>	// 发布角点
                                        ("/laser_cloud_sharp_16Right", 2);

  ros::Publisher pubCornerPointsLessSharp16Right = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱角点
                                            ("/laser_cloud_less_sharp_16Right", 2);

  ros::Publisher pubSurfPointsFlat16Right = nh.advertise<sensor_msgs::PointCloud2>  // 发布面点
                                       ("/laser_cloud_flat_16Right", 2);

  ros::Publisher pubSurfPointsLessFlat16Right = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱面点
                                           ("/laser_cloud_less_flat_16Right", 2);


  struct timeval t_start, t_end;
  ros::Rate rate(100);
  int frameNo = 0;
  while (ros::ok())
  {
    ros::spinOnce();

#ifdef Laser16_enable
    if (newLidar32 && newLidar16Left && newLidar16Right &&
        fabs(timeLidar16Left - timeLidar32) < 0.005 &&
        fabs(timeLidar16Right - timeLidar32) < 0.005)
#else
    if (newLidar32)
#endif
    {
      gettimeofday(&t_start, NULL);
      newLidar32 = false;
      newLidar16Left = false;
      newLidar16Right = false;
      frameNo++;

      if(frameNo < WAIT_FRAME_NUM)
      {
        rate.sleep();
        continue;
      }

      // clear key points
      cornerPointsSharp32->clear();
      cornerPointsLessSharp32->clear();
      surfPointsFlat32->clear();
      surfPointsLessFlat32->clear();

      cornerPointsSharp16Left->clear();
      cornerPointsLessSharp16Left->clear();
      surfPointsFlat16Left->clear();
      surfPointsLessFlat16Left->clear();

      cornerPointsSharp16Right->clear();
      cornerPointsLessSharp16Right->clear();
      surfPointsFlat16Right->clear();
      surfPointsLessFlat16Right->clear();


//      ExtractKeyPoints(laserCloud32, 64, 6, MAX_POINTNUM64,
//                       cornerPointsSharp32, cornerPointsLessSharp32,
//                       surfPointsFlat32, surfPointsLessFlat32);

      // extract key points
      ExtractKeyPoints(laserCloud32, 32, 6, MAX_POINTNUM32,
                       cornerPointsSharp32, cornerPointsLessSharp32,
                       surfPointsFlat32, surfPointsLessFlat32);

//      ExtractKeyPoints(laserCloud32, 16, 6, MAX_POINTNUM32,
//                       cornerPointsSharp32, cornerPointsLessSharp32,
//                       surfPointsFlat32, surfPointsLessFlat32);


#ifdef Laser16_enable
      ExtractKeyPoints(laserCloud16Left, 16, 4, MAX_POINTNUM16L,
                       cornerPointsSharp16Left, cornerPointsLessSharp16Left,
                       surfPointsFlat16Left, surfPointsLessFlat16Left);
      ExtractKeyPoints(laserCloud16Right, 16, 4, MAX_POINTNUM16R,
                       cornerPointsSharp16Right, cornerPointsLessSharp16Right,
                       surfPointsFlat16Right, surfPointsLessFlat16Right);

        // calibration
      ProjectPoints(laserCloud16Left, lidar16ExParaLeft);
      ProjectPoints(cornerPointsSharp16Left, lidar16ExParaLeft);
      ProjectPoints(cornerPointsLessSharp16Left, lidar16ExParaLeft);
      ProjectPoints(surfPointsFlat16Left, lidar16ExParaLeft);
      ProjectPoints(surfPointsLessFlat16Left, lidar16ExParaLeft);
      ProjectPoints(laserCloud16Right, lidar16ExParaRight);
      ProjectPoints(cornerPointsSharp16Right, lidar16ExParaRight);
      ProjectPoints(cornerPointsLessSharp16Right, lidar16ExParaRight);
      ProjectPoints(surfPointsFlat16Right, lidar16ExParaRight);
      ProjectPoints(surfPointsLessFlat16Right, lidar16ExParaRight);
#endif
      // publish
        // lidar32
      sensor_msgs::PointCloud2 laserCloud32Msg;
      pcl::toROSMsg(*laserCloud32, laserCloud32Msg);
      laserCloud32Msg.header.stamp = Lidar32Header.stamp;
      laserCloud32Msg.header.frame_id = "/camera";
      pubLaserCloud32.publish(laserCloud32Msg);

      sensor_msgs::PointCloud2 cornerPointsSharp32Msg;
      pcl::toROSMsg(*cornerPointsSharp32, cornerPointsSharp32Msg);
      cornerPointsSharp32Msg.header.stamp = Lidar32Header.stamp;
      cornerPointsSharp32Msg.header.frame_id = "/camera";
      pubCornerPointsSharp32.publish(cornerPointsSharp32Msg);

      sensor_msgs::PointCloud2 cornerPointsLessSharp32Msg;
      pcl::toROSMsg(*cornerPointsLessSharp32, cornerPointsLessSharp32Msg);
      cornerPointsLessSharp32Msg.header.stamp = Lidar32Header.stamp;
      cornerPointsLessSharp32Msg.header.frame_id = "/camera";
      pubCornerPointsLessSharp32.publish(cornerPointsLessSharp32Msg);

      sensor_msgs::PointCloud2 surfPointsFlat32Msg;
      pcl::toROSMsg(*surfPointsFlat32, surfPointsFlat32Msg);
      surfPointsFlat32Msg.header.stamp = Lidar32Header.stamp;
      surfPointsFlat32Msg.header.frame_id = "/camera";
      pubSurfPointsFlat32.publish(surfPointsFlat32Msg);

      sensor_msgs::PointCloud2 surfPointsLessFlat32Msg;
      pcl::toROSMsg(*surfPointsLessFlat32, surfPointsLessFlat32Msg);
      surfPointsLessFlat32Msg.header.stamp = Lidar32Header.stamp;
      surfPointsLessFlat32Msg.header.frame_id = "/camera";
      pubSurfPointsLessFlat32.publish(surfPointsLessFlat32Msg);

        // lidar16Left
      sensor_msgs::PointCloud2 laserCloud16LeftMsg;
      pcl::toROSMsg(*laserCloud16Left, laserCloud16LeftMsg);
      laserCloud16LeftMsg.header.stamp = Lidar32Header.stamp;
      laserCloud16LeftMsg.header.frame_id = "/camera";
      pubLaserCloud16Left.publish(laserCloud16LeftMsg);

      sensor_msgs::PointCloud2 cornerPointsSharp16LeftMsg;
      pcl::toROSMsg(*cornerPointsSharp16Left, cornerPointsSharp16LeftMsg);
      cornerPointsSharp16LeftMsg.header.stamp = Lidar16LeftHeader.stamp;
      cornerPointsSharp16LeftMsg.header.frame_id = "/camera";
      pubCornerPointsSharp16Left.publish(cornerPointsSharp16LeftMsg);

      sensor_msgs::PointCloud2 cornerPointsLessSharp16LeftMsg;
      pcl::toROSMsg(*cornerPointsLessSharp16Left, cornerPointsLessSharp16LeftMsg);
      cornerPointsLessSharp16LeftMsg.header.stamp = Lidar16LeftHeader.stamp;
      cornerPointsLessSharp16LeftMsg.header.frame_id = "/camera";
      pubCornerPointsLessSharp16Left.publish(cornerPointsLessSharp16LeftMsg);

      sensor_msgs::PointCloud2 surfPointsFlat16LeftMsg;
      pcl::toROSMsg(*surfPointsFlat16Left, surfPointsFlat16LeftMsg);
      surfPointsFlat16LeftMsg.header.stamp = Lidar16LeftHeader.stamp;
      surfPointsFlat16LeftMsg.header.frame_id = "/camera";
      pubSurfPointsFlat16Left.publish(surfPointsFlat16LeftMsg);

      sensor_msgs::PointCloud2 surfPointsLessFlat16LeftMsg;
      pcl::toROSMsg(*surfPointsLessFlat16Left, surfPointsLessFlat16LeftMsg);
      surfPointsLessFlat16LeftMsg.header.stamp = Lidar16LeftHeader.stamp;
      surfPointsLessFlat16LeftMsg.header.frame_id = "/camera";
      pubSurfPointsLessFlat16Left.publish(surfPointsLessFlat16LeftMsg);

        // lidar16Right
      sensor_msgs::PointCloud2 laserCloud16RightMsg;
      pcl::toROSMsg(*laserCloud16Right, laserCloud16RightMsg);
      laserCloud16RightMsg.header.stamp = Lidar32Header.stamp;
      laserCloud16RightMsg.header.frame_id = "/camera";
      pubLaserCloud16Right.publish(laserCloud16RightMsg);

      sensor_msgs::PointCloud2 cornerPointsSharp16RightMsg;
      pcl::toROSMsg(*cornerPointsSharp16Right, cornerPointsSharp16RightMsg);
      cornerPointsSharp16RightMsg.header.stamp = Lidar16RightHeader.stamp;
      cornerPointsSharp16RightMsg.header.frame_id = "/camera";
      pubCornerPointsSharp16Right.publish(cornerPointsSharp16RightMsg);

      sensor_msgs::PointCloud2 cornerPointsLessSharp16RightMsg;
      pcl::toROSMsg(*cornerPointsLessSharp16Right, cornerPointsLessSharp16RightMsg);
      cornerPointsLessSharp16RightMsg.header.stamp = Lidar16RightHeader.stamp;
      cornerPointsLessSharp16RightMsg.header.frame_id = "/camera";
      pubCornerPointsLessSharp16Right.publish(cornerPointsLessSharp16RightMsg);

      sensor_msgs::PointCloud2 surfPointsFlat16RightMsg;
      pcl::toROSMsg(*surfPointsFlat16Right, surfPointsFlat16RightMsg);
      surfPointsFlat16RightMsg.header.stamp = Lidar16RightHeader.stamp;
      surfPointsFlat16RightMsg.header.frame_id = "/camera";
      pubSurfPointsFlat16Right.publish(surfPointsFlat16RightMsg);

      sensor_msgs::PointCloud2 surfPointsLessFlat16RightMsg;
      pcl::toROSMsg(*surfPointsLessFlat16Right, surfPointsLessFlat16RightMsg);
      surfPointsLessFlat16RightMsg.header.stamp = Lidar16RightHeader.stamp;
      surfPointsLessFlat16RightMsg.header.frame_id = "/camera";
      pubSurfPointsLessFlat16Right.publish(surfPointsLessFlat16RightMsg);

      gettimeofday(&t_end, NULL);
//      std::cout<<"scanRegistration consuming time: "<<(t_end.tv_sec - t_start.tv_sec)*1000.0
//                                                  + (t_end.tv_usec - t_start.tv_usec)/1000.0 <<" ms"<<std::endl;
    }

    rate.sleep();
  }

  return 0;
}

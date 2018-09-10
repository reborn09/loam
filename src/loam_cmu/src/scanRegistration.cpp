#include <math.h>
#include <time.h>
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

#include <pcl_conversions/pcl_conversions.h> // 点云格式转化
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>	//	定义了一系列可用的pointT类型
#include <pcl/filters/voxel_grid.h> // 过滤算法
#include <pcl/kdtree/kdtree_flann.h> // K-D TREE查找算法

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());	// float x, float y, float z, float intensity
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());	// 急拐角点
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());	// 不那么急的拐角点
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());	// 平滑面点
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());	// 实际上是surfPointsFlat降采样得到的集合
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>()); // 用于surfPointsFlat降采样
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZI>()); // 用于surfPointsFlat降采样

float cloudCurvature[40000]; // 每个点平滑度
int cloudSortInd[40000];	//	排序序号查找表，cloudSortInd[i]表示第i大的点对应在原始点云中的序号
int cloudNeighborPicked[40000];	// cloudeNeighborPicked[i]表示点i要么是几乎平行与激光方向，要么是遮挡造成的跳跃点，总之是不可靠特征点
int cloudLabel[40000];

int scanStartInd[16]; // 每根激光点云的开始id与结束id
int scanEndInd[16];

// 4个发布对象，分别发布角点、弱角点、面点、弱面点
ros::Publisher* pubCornerPointsSharpPointer;
ros::Publisher* pubCornerPointsLessSharpPointer;
ros::Publisher* pubSurfPointsFlatPointer;
ros::Publisher* pubSurfPointsLessFlatPointer;

// 这里的点坐标都是以m为单位
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{
  pcl::fromROSMsg(*laserCloudIn2, *laserCloud); // ROS-PCL格式转换
  int cloudSize = laserCloud->points.size();

  // 计算每个点的平滑度，并且得到每个beam起始、结尾点序号的查找表
  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++) {
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

    if (int(laserCloud->points[i].intensity) != scanCount) {	// 实际上是预先用intensity的整数部分来存储这个点属于第几圈的信息
      scanCount = int(laserCloud->points[i].intensity);

      if (scanCount > 0) {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }
  scanStartInd[0] = 5;
  scanEndInd[15] = cloudSize - 5;


  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;	// 邻接点距离平方

  // 标记由遮挡造成的无效候选点
    if (diff > 0.1) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1; // 实际上diffX, diffY, diffZ都是将更远的点按照光
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1; // 追踪法移到更近点相同的深度计算的
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {  // 实际上就是tan值，这个值很小意味着两个点是相邻点
          cloudNeighborPicked[i - 5] = 1;	// cloudNeighborPicked表示这个点不能再被选取为特征点
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
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

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {	// roughly parallel plane
      cloudNeighborPicked[i] = 1;
    }
  }

  for (int i = 0; i < 16; i++)
  {
    surfPointsLessFlatScan->clear();
    for (int j = 0; j < 6; j++)
  {
      int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;			// 将每一个beam分成6等份，sp为每一等分的头
      int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;	// ep为每一等分的尾

      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {		// 每一等分内插入增序排序，排的是序号
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
          for (int l = -1; l >= -5; l--) {
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
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;	// 面点标记为-1
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {	// 每一等分上，面点不超过4个
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
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

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    surfPointsLessFlatScanDS->clear();
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);

    *surfPointsLessFlat += *surfPointsLessFlatScanDS;	// 降采样
  }

  sensor_msgs::PointCloud2 cornerPointsSharp2;
  pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharp2);
  cornerPointsSharp2.header.stamp = laserCloudIn2->header.stamp;
  cornerPointsSharp2.header.frame_id = "/camera";
  pubCornerPointsSharpPointer->publish(cornerPointsSharp2);

  sensor_msgs::PointCloud2 cornerPointsLessSharp2;
  pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharp2);
  cornerPointsLessSharp2.header.stamp = laserCloudIn2->header.stamp;
  cornerPointsLessSharp2.header.frame_id = "/camera";
  pubCornerPointsLessSharpPointer->publish(cornerPointsLessSharp2);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudIn2->header.stamp;
  surfPointsFlat2.header.frame_id = "/camera";
  pubSurfPointsFlatPointer->publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudIn2->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/camera";
  pubSurfPointsLessFlatPointer->publish(surfPointsLessFlat2);

  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>	// 订阅velodyne_points话题，接收PointCloud2消息
                                  ("/velodyne_points", 2, laserCloudHandler);

  ros::Publisher pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>	// 发布角点
                                        ("/laser_cloud_sharp", 2);

  ros::Publisher pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱角点
                                            ("/laser_cloud_less_sharp", 2);

  ros::Publisher pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>  // 发布面点
                                       ("/laser_cloud_flat", 2);

  ros::Publisher pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>  // 发布弱面点
                                           ("/laser_cloud_less_flat", 2);

  pubCornerPointsSharpPointer = &pubCornerPointsSharp;
  pubCornerPointsLessSharpPointer = &pubCornerPointsLessSharp;
  pubSurfPointsFlatPointer = &pubSurfPointsFlat;
  pubSurfPointsLessFlatPointer = &pubSurfPointsLessFlat;

  ros::spin();

  return 0;
}

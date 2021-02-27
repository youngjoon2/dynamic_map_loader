/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_map/voxel_grid_covariance.h>
#include <dynamic_map/get_file.h>

// template <typename PointT>
// struct MapCell
// {
//   MapCell() 
//   : index_(-1)
//   , centroid_(Eigen::Vector3f::Zero())
//   , use_this_(false)
//   {
//   }

//   int index_;

//   Eigen::Vector3f centroid_;

//   pcl::PointCloud<PointT> points_;

//   bool use_this_;
// };

// /** \brief Pointer to MapCell structure */
// typedef MapCell* MapCellPtr;

// /** \brief Const pointer to MapCell structure */
// typedef const MapCell* MapCellConstPtr;

static ros::Subscriber sub_current_pose_;
static ros::Subscriber sub_initialpose_;
static ros::Subscriber sub_lane_waypoints_array_;

static ros::Publisher pub_point_map_;
static ros::Publisher pub_pmap_stat_;

static geometry_msgs::PoseStamped current_pose_, previous_pose_;
bool update_current_pose_ = false;

// pcl::PointCloud<pcl::PointXYZ> map_cloud_;

std::list<pcl::PointCloud<pcl::PointXYZ>> cell_queue_;

// static geometry_msgs::PoseStamped initial_pose_;
// bool update_initial_pose_ = false;

void callbackCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  double dist = std::hypot(previous_pose_.pose.position.x - msg->pose.position.x, previous_pose_.pose.position.y - msg->pose.position.y); 
  if (dist > 10.0)
  {
    previous_pose_ = current_pose_;
    current_pose_ = *msg;
    update_current_pose_ = true;
    std::cout << "Update current_pose!!" << std::endl;
  }
}

void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  // initial_pose_ = *msg;
  // current_pose_ = *msg;
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;
  update_current_pose_ = true;
  std::cout << "Update initialpose!!" << std::endl;
  // update_initial_pose_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_map");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::vector<std::string> pcd_paths;
  private_nh.getParam("pcd_paths", pcd_paths);

  sub_current_pose_ = nh.subscribe("current_pose", 1, &callbackCurrentPose);
  sub_initialpose_ = nh.subscribe("initialpose", 1, &callbackInitialPose);

  pub_point_map_ = nh.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);
  pub_pmap_stat_ = nh.advertise<std_msgs::Bool>("pmap_stat", 1, true);

  

  std::cout << "pcd_paths.size(): " << pcd_paths.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr overall_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (std::uint32_t i = 0; i < pcd_paths.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_paths.at(i), cloud) == -1)
    {
      std::cout << "Couldn't find " << pcd_paths.at(i) << "." << std::endl;
      return -1;
    }
    *overall_map_cloud += cloud;
  }

  std::cout << "overall_map_cloud->size(): " << overall_map_cloud->size() << std::endl;
  
  pcl::VoxelGridCovariance<pcl::PointXYZ> cells;
  cells.setLeafSize(5.0, 5.0, 5.0);
  cells.setInputCloud(overall_map_cloud);
  cells.filter(true);

  std_msgs::Bool stat_msg;
  stat_msg.data = false;
  pub_pmap_stat_.publish(stat_msg);

  std::cout << "Start dynamic map loader loop!!" << std::endl;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    std_msgs::Bool stat_msg;
    stat_msg.data = true;
    pub_pmap_stat_.publish(stat_msg);

    if (update_current_pose_)
    {
      ros::WallTime start = ros::WallTime::now();

      pcl::PointXYZ p;
      std::vector<pcl::VoxelGridCovariance<pcl::PointXYZ>::LeafConstPtr> map_cells;
      std::vector<float> distances;

      p.x = current_pose_.pose.position.x;
      p.y = current_pose_.pose.position.y;
      p.z = current_pose_.pose.position.z;

      cells.radiusSearch(p, 80.0, map_cells, distances);

      for (std::uint32_t i = 0; i < map_cells.size(); i++)
      {
        if (!map_cells.at(i)->use_this_)
        {
          cell_queue_.push_back(map_cells.at(i)->points_);
          cells.setUseCell(map_cells.at(i)->index_); // need to write codes
        }
      }

      // removeUselessCells(); // need to write codes

      pcl::PointCloud<pcl::PointXYZ> map_cloud;
      for (std::list<pcl::PointCloud<pcl::PointXYZ>>::const_iterator iter = cell_queue_.begin(); iter != cell_queue_.end(); ++iter)
      {
        map_cloud += *iter;
      }
      std::cout << "map_cloud.size(): " << map_cloud.size() << std::endl;

      sensor_msgs::PointCloud2 map_msg;
      pcl::toROSMsg(map_cloud, map_msg);

      map_msg.header.stamp = ros::Time::now();
      map_msg.header.frame_id = "map";
      pub_point_map_.publish(map_msg);

      update_current_pose_ = false;

      std::cout << "exec time: " << ros::WallTime::now() - start << std::endl;
    }

    loop_rate.sleep();
  }

  // const std::map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf> leaves = cells.getLeaves();

  
  // std::cout << "leaves.size(): " << leaves.size() << std::endl;
  // for (std::uint32_t i = 0; i < leaves.size(); i++)
  // {
  //   std::cout << "leaves.at(i).point_indices_.size(): " << (leaves.begin() + i)->point_indices_.size() << std::endl;
  //   break;
  // }
  // for (std::map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf>::const_iterator it = leaves.begin(); it != leaves.end(); ++it)
  // {
    // std::cout << "it->size(): " << it->second.point_indices_.size() << std::endl;
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // for (const auto& i : it->second.point_indices_)
    // {
    //   cloud.push_back(map_cloud->at(i));
    // }
    // std::cout << "it->second.points_.size(): " << it->second.points_.size() << std::endl;

    // std::string output_path = "/home/autoware/shared_dir/ndt_test/" + std::to_string(it->first) + ".pcd";
    // if (pcl::io::savePCDFileBinary(output_path, cloud) < 0)
    // {
    //   std::cout << "Failed saving " << output_path << "." << std::endl;
    //   exit(-1);
    // }
    // break;
  // }

  // ros::spin();

  return 0;
}

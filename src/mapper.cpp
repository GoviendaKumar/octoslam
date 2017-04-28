/*
*
*OCTOSLAM
*Octoslam is a ROS package which converts files in Polygon File Format(ply) to Binary Terrain Files(bt).
*This software has been developed by using Octomap- An Efficient Probabilistic 3D Mapping Framework Based on Octrees as infrastructure.
*http://octomap.github.com/.
*
*To find detailed information please visit following address
*https://github.com/yng05/octoslam/wiki
*
*Bauhaus University Weimar 2016-17
*
*/

#include <string> 
#include <sstream> 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_ros/conversions.h>

using namespace std;
using namespace octomap;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Subscriber sub;

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

/*void bt_file_publisher(){

  ros::AsyncSpinner spinner(1);
  ros::NodeHandle nh;

  std::cout<<"Reading File"<<std::endl;
  
  octomap::OcTree* octree = new octomap::OcTree("result_tree.bt");
  std::cout << "File Read Sccessfully" << std::endl;

  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(*octree, bmap_msg);
  ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("display_env",1);

  octomap_publisher.publish(bmap_msg);
  cout << "published from file.bt..." << endl;

}*/

void octomap_publisher(OcTree* octree){
  cout << "========================" << endl;
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle nh;

  for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
    { 
        std::cout << "Node center: " << it.getCoordinate();
        std::cout << " value: " << it->getValue() << "\n";
    }
  
  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(*octree, bmap_msg);

  ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("display_env",1);

  octomap_publisher.publish(bmap_msg);
  cout << "Published" << endl;

}

void print_map(OcTree* tree){
  cout << endl;
  cout << "generating result map" << endl;  
  
  tree->writeBinary("result_tree.bt");
  cout << "wrote example file result_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis result_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  
}

void octreeCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
{

  cout << "receiving..." << endl;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

  print_map(octree);
  cout << "printed..." << endl;
  //bt_file_publisher();
  octomap_publisher(octree);
  sub.shutdown();
}

int main(int argc, char** argv) {
  cout << "listening octree publisher..." << endl;
  ros::init(argc, argv, "mapper");
  ros::NodeHandle n;
  sub = n.subscribe("octree", 1000, octreeCallback);
  ros::spin();

  return 0;

}


#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_unfiltered (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read ("pointcloud.pcd", *cloud_unfiltered);

  std::cerr << "PointCloud before filtering: " << cloud_unfiltered->width * cloud_unfiltered->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1mm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_unfiltered);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud_filtered);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered, *cloud);

  std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("pointcloud_downsampled.pcd", *cloud, false);



  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (1000);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  //extract the segments and save it to file
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  for (int i=0; i <clusters.size(); i++){
    pcl::IndicesPtr indices_ptr (new std::vector<int> (clusters[i].indices.size ()));
        for (int j = 0; j < indices_ptr->size (); j++)
          (*indices_ptr)[j] = clusters[i].indices[j];

    extract.setInputCloud (cloud);
    extract.setIndices (indices_ptr);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the segmented objects: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "segments/seg_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);


  }

  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::visualization::CloudViewer viewer ("Cluster viewer");
  // viewer.showCloud(colored_cloud);
  // while (!viewer.wasStopped ())
  // {
  // }

  return (0);
}

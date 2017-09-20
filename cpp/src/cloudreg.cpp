#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "opencv2/core/core.hpp"
#include <stdlib.h>
#include"cnpy.h"
#include <iostream>
#include <sstream>
#include <json/json.h> 
 
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

double FOCAL = 570.3;
double MM_PER_M = 1000;
int WIDTH = 240;
int HEIGHT = 320;
double DEPTH_THRESHOLD = 400;   
double X_THRESHOLD = 1000;   
double Y_THRESHOLD = 1000;   
const float normal_estimation_search_radius = 1.0f;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void downsample (PointCloudPtr &points, float leaf_size, PointCloudPtr &downsampled_out)
{
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}

void saveTransform (const std::string &file, const Eigen::Matrix4f &transform)
{
  ofstream ofs;
  ofs.open (file.c_str (), ios::trunc | ios::binary);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      ofs.write (reinterpret_cast<const char*>(&transform (i, j)), sizeof (float));  
  ofs.close ();
}

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, int Nreg, float downsample)
{
  //Downsample
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample>0)
  {
    grid.setLeafSize (downsample, downsample, downsample);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  //get Normals
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (10);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  MyPointRepresentation point_representation;
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //Register
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-8);
  reg.setMaxCorrespondenceDistance (100);  
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, sourceToTarget;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (8);

  for (int i = 0; i < Nreg; ++i)
  {
    points_with_normals_src = reg_result;

    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    Ti = reg.getFinalTransformation () * Ti;

    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
    {
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.01);
      //cout<<reg.getMaxCorrespondenceDistance ()<<endl;
    }
    prev = reg.getLastIncrementalTransformation ();
    //cout<<i<<" "<<reg.getFitnessScore()<<endl;
    cout<<reg.getFitnessScore()<<endl;
 
  }

  sourceToTarget = Ti;//.inverse();
  pcl::transformPointCloud (*cloud_src, *output, sourceToTarget);
  final_transform = sourceToTarget;
  
}

int main (int argc, char** argv)
{
   int i1=atoi(argv[3])%120;
   int i2=atoi(argv[4])%120;
   string svg=argv[1];

   Json::Value params;      
   ostringstream pfile;
   pfile<< svg<< "/params.json";
   ifstream paramsFile(pfile.str().c_str());

   Json::Reader reader;
   bool parsingSuccessful = reader.parse( paramsFile, params);
   
   PointCloudPtr pcd1 (new PointCloud);
   PointCloudPtr pcd2 (new PointCloud);
   PointCloudPtr pcd3 (new PointCloud);

   ostringstream pcdfile1;
   pcdfile1<< argv[1] << "/" << params["reg"]["file"].asString() << "/" << setfill('0') << setw(3) << i1 <<".pcd";
   
   pcl::io::loadPCDFile<PointT> (pcdfile1.str(), *pcd1);

   ostringstream pcdfile2;
   pcdfile2<< argv[1] << "/" << params["reg"]["file"].asString() << "/" << setfill('0') << setw(3) << i2 <<".pcd";
   pcl::io::loadPCDFile<PointT> (pcdfile2.str(), *pcd2);
      
   ostringstream pcdfile3;
   pcdfile3<< argv[2] << "/transform/" << setfill('0') << setw(3) << i1 << "_" << setfill('0') << setw(3) << i2 <<".pcd";

   Eigen::Matrix4f pairTransform;
   pairAlign (pcd1, pcd2, pcd3, pairTransform, params["reg"]["Nreg"].asInt(), params["reg"]["downsample"].asFloat());
   //pairAlign (pcd1, pcd2, pcd3, pairTransform, 1, true);
      
   std::stringstream sT;
   sT << argv[2] <<"/transform/"<< setfill('0') << setw(3) << i1 <<"_"<< setfill('0') << setw(3) << i2 << ".transform";
   saveTransform (sT.str(), pairTransform);

   pcl::io::savePCDFileASCII (pcdfile3.str(), *pcd3);
}

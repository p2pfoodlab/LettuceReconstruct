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

#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
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

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

void bilfil (PointCloud& pcloud, PointCloud &output, float bifil_sigR, float bifil_sigS)
{
   pcl::FastBilateralFilter<PointT> fbFilter;
   fbFilter.setInputCloud(pcloud.makeShared());
       
   fbFilter.setSigmaR(bifil_sigR);
   fbFilter.setSigmaS(bifil_sigS);
   fbFilter.applyFilter(output);
}

void prefilters(PointCloud& pcloud, float bifil_sigR, float bifil_sigS, int sor_N, float sor_th)
{

   if (bifil_sigS>0) bilfil (pcloud, pcloud, bifil_sigR, bifil_sigS);
    
   pcl::StatisticalOutlierRemoval<PointT> sor;
   sor.setInputCloud (pcloud.makeShared());
   sor.setMeanK (sor_N);
   sor.setStddevMulThresh (sor_th);
   sor.setKeepOrganized (true);
   sor.filter (pcloud);

   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(pcloud, pcloud, indices);
}

void mls_smoothing(PointCloud& pcloud, PointCloudWithNormals& mls_points)
{
   float search_radius=2;
   float sqr_gauss_param=4;
   bool use_polynomial_fit=false;
   int polynomial_order=2;
   
   pcl::MovingLeastSquares<PointT, PointNormalT> mls;
   mls.setInputCloud (pcloud.makeShared());
   mls.setSearchRadius (search_radius);
   mls.setSqrGaussParam (sqr_gauss_param);
   mls.setPolynomialFit (use_polynomial_fit);
   mls.setPolynomialOrder (polynomial_order);
   
   mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointNormalT>::NONE);
   mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
   mls.setUpsamplingRadius (0.025);
   mls.setUpsamplingStepSize (0.015);
   mls.setDilationIterations (2);
   mls.setDilationVoxelSize (0.01f);
   
   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
   mls.setSearchMethod (tree);
   mls.setComputeNormals (true);
   mls.process (mls_points);
}

int main (int argc, char** argv)
{
   string svg=argv[1];
   int k=atoi(argv[2]);

   Json::Value params;      
   ostringstream pfile;
   pfile<< svg << "/params.json";
   ifstream paramsFile(pfile.str().c_str());

   Json::Reader reader;
   bool parsingSuccessful = reader.parse( paramsFile, params);
   cout<< pfile.str() <<endl;
  
   cout<<"Loading..."; 
   cout<<argv[1]<<" ... "<<argv[2]<<"...";

     
   PointCloud pcd;
   
   ostringstream pcdfile;
   pcdfile<< argv[1] << "/raw/"<< setfill('0') << setw(3) << k <<".pcd";
   pcl::io::loadPCDFile<PointT> (pcdfile.str(), pcd);

   cout<< "filtering" <<endl;
   prefilters(pcd, params["prefilter"]["bifil_sigR"].asFloat(), params["prefilter"]["bifil_sigS"].asFloat(),  params["prefilter"]["sor"]["N"].asInt(), params["prefilter"]["sor"]["th"].asFloat());   

   ostringstream pcdfile_fil;
   pcdfile_fil<< argv[1] << "/filtered/" << setfill('0') << setw(3) << k <<".pcd";
  
   pcl::io::savePCDFileASCII (pcdfile_fil.str(), pcd);
  

}

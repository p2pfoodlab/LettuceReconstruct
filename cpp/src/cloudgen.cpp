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
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include"cnpy.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <json/json.h> 

using namespace cv;
using namespace cnpy;
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

void loadPCD (NpyArray vert_arr, NpyArray sync_arr, Mat confimg, PointCloud& pointcloud, bool box, bool planecut)
{
   short* vert=reinterpret_cast<short*>(vert_arr.data);
   uint8_t* sync=reinterpret_cast<uint8_t*>(sync_arr.data);
   int s=vert_arr.shape[0]*vert_arr.shape[1];
   const float bad_point = std::numeric_limits<float>::quiet_NaN();
   pointcloud.resize(confimg.rows*confimg.cols);
 
   pointcloud.width=confimg.cols;
   pointcloud.height=confimg.rows;

   // convert all the points in the mask into point cloud
   for (int i = 0; i < confimg.rows; i++)
   {
     for (int j = 0; j < confimg.cols; j++)
     {
      int confidence = (int)confimg.at<uchar> (i, j);
      //int k=(i*confimg.cols+j);
      int k=(j*confimg.rows+i);

      bool selector=confidence>254;
      
      if (box) selector = (selector && vert[3*k+2]<DEPTH_THRESHOLD && vert[3*k]<X_THRESHOLD && vert[3*k+1]<Y_THRESHOLD);
 
      if (planecut) selector = (selector && ((vert[3*k]*.32+vert[3*k+1]*1.18+391.9-vert[3*k+2])>100));

      PointT point;
       
      if (selector) {   
        point.x = vert[3*k];
        point.y = vert[3*k+1];
        point.z = vert[3*k+2];
        point.b = sync[3*k];
        point.g = sync[3*k+1];
        point.r = sync[3*k+2];
      }
      else{
        point.x = bad_point;
        point.y = bad_point;
        point.z = bad_point;
        point.b = sync[3*k];
        point.g = sync[3*k+1];
        point.r = sync[3*k+2];
      } 
      pointcloud.at(j,i)=point; 
        
    }
  }
}  

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
   pcl::StatisticalOutlierRemoval<PointT> sor;
   sor.setInputCloud (pcloud.makeShared());
   sor.setMeanK (sor_N);
   sor.setStddevMulThresh (sor_th);
   sor.setKeepOrganized (true);
   sor.filter (pcloud);
   if (bifil_sigS>0) bilfil (pcloud, pcloud, bifil_sigR, bifil_sigS);

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
   string data_folder = argv[1];
   string svg=argv[2];
   int k=atoi(argv[3]);
   //string data_folder = "/home/kodda/Dropbox/p2pflab/data/2017/";
   
   Json::Value params;      
   ostringstream pfile;
   pfile<< svg << "/params.json";
   ifstream paramsFile(pfile.str().c_str());

   Json::Reader reader;
   bool parsingSuccessful = reader.parse( paramsFile, params);
   //cout<< pfile.str() <<endl;
  
   cout<<"Loading..."; 
   cout<<data_folder<<"...";

   NpyArray vert_arr; 
   NpyArray sync_arr;
   Mat confimg;

   ostringstream vertfile;
   vertfile<<data_folder<<"/vert-"<<setfill('0')<< setw(3)<<k<<".npy";
   vert_arr = npy_load(vertfile.str());
   cout<<vertfile.str()<<endl;

   ostringstream syncfile;
   syncfile<<data_folder<<"/sync-"<<setfill('0')<< setw(3)<<k<<".npy";
   sync_arr = npy_load(syncfile.str());
   //cout<<syncfile.str()<<endl;
   
   ostringstream conffile;
   conffile<<data_folder<<"/confidence-"<<setfill('0')<< setw(3)<<k<<".png";
   confimg = imread (conffile.str(), 0);   
   //cout<<conffile.str()<<endl;
     
   PointCloud pcloud;
   loadPCD (vert_arr, sync_arr, confimg, pcloud, params["selector"]["box"].asBool(), params["selector"]["planecut"].asBool());
   cout<<"done"<<endl;
   vert_arr.destruct();
   sync_arr.destruct();

   ostringstream pcdfile_raw;
   pcdfile_raw << svg << "/raw/" << setfill('0') << setw(3) << k <<".pcd";

   pcl::io::savePCDFileASCII (pcdfile_raw.str(), pcloud);

   if (params["prefilter"]["ON"].asBool()){
      cout<< "filtering" <<endl;
      prefilters(pcloud, params["prefilter"]["bifil_sigR"].asFloat(), params["prefilter"]["bifil_sigS"].asFloat(),  params["prefilter"]["sor"]["N"].asInt(), params["prefilter"]["sor"]["th"].asFloat());   

      ostringstream pcdfile_fil;
      pcdfile_fil<< svg << "/filtered/" << setfill('0') << setw(3) << k <<".pcd";
      cout<<pcdfile_fil.str()<<endl;
      pcl::io::savePCDFileASCII (pcdfile_fil.str(), pcloud);
   }

}

//c++ standard lib
#include <iostream>
#include <string>

using namespace std;

//OpenCV lib

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL lib

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//define point cloud types

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//camera internal parameter

const double camera_factor = 1000;

const double camera_cx = 325.5;

const double camera_cy = 253.5;

const double camera_fx = 518.0;

const double camera_fy = 519.0;

//main function

int main(int argc, char** argv)
{
    //read .png and transform to pcl

    //image matrices

    cv::Mat rgb, depth;

    //use cv::imread()to read the image
    //API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
    rgb = cv::imread("./data/rgb.png");
    //rgb image is 8UC3 color image
    //depth is 16UC1 single channel image. -1 reprents, that read the original data without changes
    depth = cv::imread("./data/depth.png", -1);

    //pcl variable
    //use smart pointer to set up an empty pcl. This pointer will realese after use.
    PointCloud:: Ptr cloud (new PointCloud);

    //traversal depth image
    for (int m=0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
          //get the value by (m,n)
          ushort d = depth.ptr<ushort>(m)[n];
          // d may have no value, no? skip this point_types
          if(d==0)
              continue;
          // d has value, then add a point in pcl
          PointT p;


          //Calculate the space coordinate of this point
          p.z = double(d) / camera_factor;
          p.x = (n - camera_cx) * p.z / camera_fx;
          p.y = (m - camera_cy) * p.z / camera_fy;

          //get the color von rgb image
          p.b = rgb.ptr<uchar>(m)[n*3];
          p.g = rgb.ptr<uchar>(m)[n*3+1];
          p.r = rgb.ptr<uchar>(m)[n*3+2];


          //add the p to PCL
          cloud->points.push_back(p);
        }
      //set and save pcl
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );

    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;


}

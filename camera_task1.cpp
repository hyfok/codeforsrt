#include <iostream>
#include <string>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


const double camera_factor = 1000;
const double camera_cx = 319.500000;
const double camera_cy = 239.500000;
const double camera_fx = 571.623718;
const double camera_fy = 571.623718;


// In parameter: destination image, depth image, pcolortype
// Out parameter: none
// Function: convert a depth image to a false color image
// Include: #include <opencv2/imgproc/imgproc.hpp>
void depth2pcolor(cv::Mat & pcolor, const cv::Mat & depth, int pcolortype = 2) {
	/*
	COLORMAP_AUTUMN = 0, //!< ![autumn](pics/colormaps/colorscale_autumn.jpg)
	COLORMAP_BONE = 1, //!< ![bone](pics/colormaps/colorscale_bone.jpg)
	COLORMAP_JET = 2, //!< ![jet](pics/colormaps/colorscale_jet.jpg)
	COLORMAP_WINTER = 3, //!< ![winter](pics/colormaps/colorscale_winter.jpg)
	COLORMAP_RAINBOW = 4, //!< ![rainbow](pics/colormaps/colorscale_rainbow.jpg)
	COLORMAP_OCEAN = 5, //!< ![ocean](pics/colormaps/colorscale_ocean.jpg)
	COLORMAP_SUMMER = 6, //!< ![summer](pics/colormaps/colorscale_summer.jpg)
	COLORMAP_SPRING = 7, //!< ![spring](pics/colormaps/colorscale_spring.jpg)
	COLORMAP_COOL = 8, //!< ![cool](pics/colormaps/colorscale_cool.jpg)
	COLORMAP_HSV = 9, //!< ![HSV](pics/colormaps/colorscale_hsv.jpg)
	COLORMAP_PINK = 10, //!< ![pink](pics/colormaps/colorscale_pink.jpg)
	COLORMAP_HOT = 11, //!< ![hot](pics/colormaps/colorscale_hot.jpg)
	COLORMAP_PARULA = 12 //!< ![parula](pics/colormaps/colorscale_parula.jpg)
	 */
	 // the maximum/minimux pixel value in depth image;
	double max = 0, min = 0;
	int cols = depth.cols;
	int rows = depth.rows;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			ushort d = depth.at<ushort>(i, j);
			if (d <= min && d != 0) {
				min = d;
			}
			if (d >= max) {
				max = d;
			}
		}
	}

	double alpha = 255.0 / (max - min);
	// expand your range to 0..255. Similar to histEq();
	cv::Mat grayImage;
	depth.convertTo(grayImage, CV_8UC1, alpha, -alpha * min);
	//It converts your grayscale image into a tone-mapped one,
	// much more pleasing for the eye function is found in contrib module,
	// so include contrib.hpp  and link accordingly
	applyColorMap(grayImage, pcolor, pcolortype);
}

int main(int argc, char ** argv) {
	cv::Mat rgb, depth, prgb;
	rgb = cv::imread("task\\color\\color0.jpg");
	depth = cv::imread("task\\depth\\depth0.pgm", cv::IMREAD_UNCHANGED);
	resize(rgb, rgb, depth.size());
	depth2pcolor(prgb, depth, 2);
	
	cv::Mat test;
	PointCloud::Ptr cloud(new PointCloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZ>);

	int sum = 0;
	
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++) {
			// type your code here

			PointT p; 			
			ushort d = depth.ptr<ushort>(m)[n];
			p.z = float(d) / camera_factor;		
			p.x = (n - camera_cx) * p.z / camera_fx;		
			p.y = (m - camera_cy) * p.z / camera_fy; 						
			p.b = rgb.ptr<uchar>(m)[n * 3];		
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];		
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2]; 			
//			p.a = prgb.ptr<uchar>(m)[n];
			cloud->points.push_back(p);
		
		}

	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = true;


	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(cloud);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.spin();


	pcl::io::savePCDFile( "nice.pcd", *cloud);
	cloud->points.clear();
	cout << "Point cloud saved" << endl;
	imshow("prgb", prgb);
	imshow("rgb", rgb);
	imshow("depth", depth);

	cv::waitKey(0);
	return 0;
}



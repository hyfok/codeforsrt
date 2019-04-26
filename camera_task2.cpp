#include <iostream>
#include <string>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
*/

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
	cv::Mat rgb1, rgb2, depth0, depth1, prgb, gray0;
	rgb1 = cv::imread("task\\color\\color0.jpg");
	rgb2 = cv::imread("task\\color\\color1.jpg");
	
	depth0= cv::imread("task\\depth\\depth0.pgm", cv::IMREAD_UNCHANGED);
	depth1 = cv::imread("task\\depth\\depth1.pgm", cv::IMREAD_UNCHANGED);
	FILE *fp_txt;
	double txt1[16], txt2[16];
	fp_txt = fopen("task\\pose\\pose0.txt", "r");

	int i=0;
	cv::Mat_<double> t1(4, 4), t2(4, 4);
	while (fscanf(fp_txt, "%lf %lf %lf %lf", txt1 + i, txt1 + i + 1, txt1 + i + 2, txt1 + i + 3) != EOF)
	{		
		for (int j = 0; j < 4; j++)
			t1(i / 4 , j) = txt1[i + j];
		i += 4;
	}
	fp_txt = fopen("task\\pose\\pose1.txt", "r");
	i = 0;
	while (fscanf(fp_txt, "%lf %lf %lf %lf", txt2 + i, txt2 + i + 1, txt2 + i + 2, txt2 + i + 3) != EOF)
	{
		for (int j = 0; j < 4; j++)
			t2(i / 4, j) = txt1[i + j];
		i += 4;
	}
	cout << rgb1.size();
	resize(rgb1, rgb1, depth0.size());
	cout << rgb1.size();
	cv::cvtColor(rgb1,gray0, cv::COLOR_RGB2GRAY);
//	depth2pcolor(prgb, depth0, 2);

	cv::Mat_<double> pcamera1(4, 1), pcamera2(4, 1), pworld(4, 1);	
	cv::Mat_<cv::Vec3b> img2(rgb1.rows, rgb1.cols, cv::Vec3b(0,0,0));

	int flag = 0;
	
	for (int m = 0; m < depth0.rows; m++)
	{
		for (int n = 0; n < depth0.cols; n++)
		{
			// type your code here
			int u, v;
			ushort d0 = depth0.ptr<ushort>(m)[n];
			ushort d1 = depth1.ptr<ushort>(m)[n];
			double z0 = double(d0) / camera_factor;
			double z1 = double(d1) / camera_factor; //两深度图坐标对应值
			
			pcamera1(0, 0) = (double(n) - camera_cx) * z0 / camera_fx;  //第一个相机坐标
			pcamera1(1, 0) = (double(m) - camera_cy) * z0 / camera_fy;
			pcamera1(2, 0) = z0;
			pcamera1(3, 0) = 1.0;
			pworld = t1.inv() * pcamera1;  //第一个相机坐标转世界坐标
			pcamera2 = t2 * pworld;  //世界坐标转第二个相机坐标
			
		//	if (z1 != 0)
			//{
				u = int(pcamera2(0, 0) * camera_fx / pcamera2(2, 0) + camera_cx + 0.5);  //第二个相机坐标转像素坐标
				v = int(pcamera2(1, 0) * camera_fy / pcamera2(2, 0) + camera_cy + 0.5);
				if (u >= img2.cols) { u = img2.cols - 1; flag++; }
				if (u < 0){u = 0; flag++; }
				if (v >= img2.rows) {v = img2.rows - 1; flag++; }
				if (v < 0){v = 0; flag++; }
				img2(v, u) = rgb1.ptr<cv::Vec3b>(m)[n];
		//	}
		//	else { flag++; }
		}
	}cout << flag;
	//cv::GaussianBlur(img2, img2,cv::Size(7,7),15);
	cv::imshow("img2", img2);
	cv::imwrite("img.jpg",img2);
	cv::waitKey(0);
	return 0;
}
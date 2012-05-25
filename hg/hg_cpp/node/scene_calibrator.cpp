#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGB color_point;


void camera_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

    // convert cloud to PCL
    pcl::PointCloud<color_point> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // get an OpenCV image from the cloud
    pcl::toROSMsg (cloud, *image_msg);

    // convert image to OpenCV
    cv_bridge::CvImagePtr cv_image;
    try
    {
    	cv_image = cv_bridge::toCvCopy(image_msg, "rgb8");
        ROS_INFO("New image/cloud.");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Conversion failed");
    }

    //find chessboard corner
    cv::Mat rgb = cv_image->image;
    cv::Mat gray;
    cv::cvtColor(rgb, gray, CV_RGB2GRAY);

    cv::Size pattern_size(4, 11); //interior number of corners
    std::vector<cv::Point2f> corners;

    bool pattern_found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);

    /*
    bool pattern_found = cv::findChessboardCorners(gray, pattern_size, corners,
    		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
    		+ cv::CALIB_CB_FAST_CHECK);


    if(pattern_found)
    {
      cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
	*/
    cv::drawChessboardCorners(rgb, pattern_size, cv::Mat(corners), pattern_found);
    cv::imshow("corner image", rgb);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "scene_calibrator");

	ros::NodeHandle nh("~");

    ros::Subscriber cloud_subscriber;
	image_transport::Publisher image_publisher;
	image_transport::ImageTransport it(nh);
	image_publisher = it.advertise("out_image", 1);

	cloud_subscriber = nh.subscribe("/camera/depth_registered/points", 1, camera_callback);

	std::string info_url;
	if(nh.getParam("/camera/driver/rgb_camera_info_url", info_url))
		ROS_INFO_STREAM(info_url);
	else
		ROS_INFO("RGB Camera information not found!");

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {
		cv::waitKey(1);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}

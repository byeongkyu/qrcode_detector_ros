#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/cudawarping.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "zbar.h"

class QRCodeDetectorNode
{
    public:
        QRCodeDetectorNode()
        : it_(nh_)
        {
            image_sub_ =  new message_filters::Subscriber<sensor_msgs::Image>(nh_, "image_raw", 10);
            pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "points", 10);
            sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), *image_sub_, *pointcloud_sub_);
            sync_->registerCallback(boost::bind(&QRCodeDetectorNode::callback, this, _1, _2));

            // cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
            cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
        }
        ~QRCodeDetectorNode()
        {
            delete sync_;
            delete image_sub_;
            delete pointcloud_sub_;
        }

    public:
        void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pointcloud)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
			    return;
            }

            pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    	    pcl::fromROSMsg(*pointcloud, pcl_cloud);

            cv::imshow("Display window", cv_ptr->image);
            cv::waitKey(1);
        }

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
	    message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud_sub_;
	    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	    message_filters::Synchronizer<MySyncPolicy> *sync_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "qrcode_detector");
    QRCodeDetectorNode m = QRCodeDetectorNode();
    ros::spin();
    return 0;
}
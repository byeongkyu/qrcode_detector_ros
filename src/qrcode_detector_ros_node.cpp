#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#include "zbar.h"
#include "qrcode_detector_ros/Result.h"

class QRCodeDetectorNode
{
    public:
        QRCodeDetectorNode()
        : it_(nh_)
        {
            image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "image_raw", 10);
            pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "points", 10);
            sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), *image_sub_, *pointcloud_sub_);
            sync_->registerCallback(boost::bind(&QRCodeDetectorNode::callback, this, _1, _2));

            pub_debug_image_ = it_.advertise("detected_image", 1);
            pub_result_ = nh_.advertise<qrcode_detector_ros::Result>("detected_code", 10);

            scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

            //cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
            ROS_INFO("[%s] initialized...", ros::this_node::getName().c_str());
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
            // Convert from ROS message to OpenCV Image and PointCloud.
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

            // Convert Image to Gray and Y800 Format for zbar.
            cv::Mat imGray;
            cv::cvtColor(cv_ptr->image, imGray, CV_BGR2GRAY);
            zbar::Image zImage(cv_ptr->image.cols, cv_ptr->image.rows, "Y800", (unsigned char*) imGray.data, cv_ptr->image.cols * cv_ptr->image.rows);

            // Scan image and find the codes.
            int numCodes = scanner.scan(zImage);
            ROS_DEBUG("Found %d codes...", numCodes);

            // Loop for each code in found codes.
            int count = 0;
            cv::RNG rng(12345);
            for (zbar::Image::SymbolIterator symbol = zImage.symbol_begin(); symbol != zImage.symbol_end(); ++symbol)
            {
                ROS_DEBUG("[%d] Type: %s, Data: %s", count, symbol->get_type_name().c_str(), symbol->get_data().c_str());

                std::vector<cv::Point> points;
                for(int i = 0; i < symbol->get_location_size(); i++)
                    points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));

                std::vector<cv::Point> hull;

                if(points.size() > 4)
                    cv::convexHull(points, hull);
                else
                    hull = points;

                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
                for(int i = 0; i < hull.size(); i++)
                {
                    cv::line(cv_ptr->image, hull[i], hull[(i+1) % hull.size()], color, 2);
                }

                // get ROI rectangle for get plane on PointCloud
                cv::Rect rect_roi = cv::Rect(
                        cv::Point(std::max(hull[0].x, hull[1].x), std::max(hull[0].y, hull[3].y)),
                        cv::Point(std::min(hull[2].x, hull[3].x), std::min(hull[1].y, hull[2].y))
                );
                cv::rectangle(cv_ptr->image, rect_roi, cv::Scalar(0, 0, 255), 2);

                pcl::PointCloud<pcl::PointXYZRGB> pcl_centroid_points;
                for(int y = 0; y < rect_roi.height; y++)
                {
                    for(int x = 0; x < rect_roi.width; x++)
                    {
                        cv::Point pt;
                        pt.x = rect_roi.x + x;
                        pt.y = rect_roi.y + y;
                        pcl_centroid_points.push_back(pcl_cloud(pt.x, pt.y));
                    }
                }

                // get centroid point
                Eigen::Vector4f centroid;
                unsigned int ret = pcl::compute3DCentroid(pcl_centroid_points, centroid);

                if(!std::isnan(centroid[0]))
                {
                    ROS_DEBUG("%f, %f, %f", centroid[0], centroid[1], centroid[2]);
                }
                else
                {
                    return;
                }

                // find plane equation
                Eigen::Vector4f plane_parameters;
                float curvature;
                Eigen::Matrix3f covariance_matrix;

                pcl::computePointNormal(pcl_centroid_points, plane_parameters, curvature);
                if(!std::isnan(plane_parameters[0]))
                {
                    ROS_DEBUG("Plane a: %f, b: %f, c: %f, d: %f", plane_parameters[0], plane_parameters[1], plane_parameters[2], plane_parameters[3]);
                }
                else
                {
                    return;
                }

                // find offset point on the normal vector
                double offset_distance = -100.0;
                double offset_origin_x = centroid[0] + plane_parameters[0] * offset_distance / 1000.0;
                double offset_origin_y = centroid[1] + plane_parameters[1] * offset_distance / 1000.0;
                double offset_origin_z = centroid[2] + plane_parameters[2] * offset_distance / 1000.0;

                // create static tf
                geometry_msgs::TransformStamped transformStamped;
                std::string qrcode_name = "qrcode_" + std::to_string(count);
                transformStamped.header.frame_id = "camera_color_optical_frame";
                transformStamped.child_frame_id = qrcode_name;

                transformStamped.transform.translation.x = offset_origin_x;
                transformStamped.transform.translation.y = offset_origin_y;
                transformStamped.transform.translation.z = offset_origin_z;

                double dx = offset_origin_x - centroid[0];
                double dy = offset_origin_y - centroid[1];
                double dz = offset_origin_z - centroid[2];

                Eigen::Vector3f v1(0, 0, 1);
                Eigen::Vector3f v2(plane_parameters[0], plane_parameters[1], plane_parameters[2]);

                Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(v1, v2);
                q.normalize();
                ROS_DEBUG("Quaternion result: %f %f %f %f", q.x(), q.y(), q.z(), q.w());

                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();

                transformStamped.header.stamp = ros::Time::now();
                tfb_.sendTransform(transformStamped);

                // Publish result
                qrcode_detector_ros::Result msg;
                msg.type = symbol->get_type_name();
                msg.data = symbol->get_data();

                msg.pose.header.stamp = ros::Time::now();
                msg.pose.header.frame_id = "camera_color_optical_frame";
                msg.pose.pose.position.x = offset_origin_x;
                msg.pose.pose.position.y = offset_origin_y;
                msg.pose.pose.position.z = offset_origin_z;

                msg.pose.pose.orientation.x = q.x();
                msg.pose.pose.orientation.y = q.y();
                msg.pose.pose.orientation.z = q.z();
                msg.pose.pose.orientation.w = q.w();

                pub_result_.publish(msg);

                count++;
            }

            zImage.set_data(NULL, 0);
            pub_debug_image_.publish(cv_ptr->toImageMsg());

            // cv::imshow("result", cv_ptr->image);
            // cv::waitKey(1);
        }

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        tf2_ros::TransformBroadcaster tfb_;
        message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> *sync_;

        zbar::ImageScanner scanner;
        ros::Publisher pub_result_;
        image_transport::Publisher pub_debug_image_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "qrcode_detector");
    QRCodeDetectorNode m = QRCodeDetectorNode();
    ros::spin();
    return 0;
}
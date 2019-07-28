# qrcode_detector_ros

## install this package

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/byeongkyu/qrcode_detector_ros.git
    $ cd qrcode_detector_ros
    $ rosdep install --from-paths . --ignore-src -r -y
    $ catkin build

## run node

    $ rosrun qrcode_detector_ros qrcode_detector_node


## Topic

### Subscribe

    /image_raw (sensor_msgs/Image)

    /points (sensor_msgs/PointCloud2)

### Publish

    /detected_code (qrcode_detector_ros::Result)

### Result Message

*qrcode_detector_ros::Result*

    string type
    string data
    geometry_msgs/PoseStamped pose


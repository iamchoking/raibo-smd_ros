// Refrence: https://answers.ros.org/question/366440/ros-2-message_filters-timesynchronizer-minimal-example-does-not-reach-callback-function/
#include <chrono>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>    
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


using namespace std::chrono_literals;

class SyncerNode : public rclcpp::Node {
public:
    typedef sensor_msgs::msg::PointCloud2 PCL2;
    typedef message_filters::sync_policies::ApproximateTime<PCL2,PCL2> SyncPolicy;

    SyncerNode() : Node("sync_bridge") {
        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();

        this->declare_parameter<std::string>("depthcam1_name","d435i_R");
        this->declare_parameter<std::string>("depthcam2_name","d435i_L");
        this->declare_parameter<std::string>("lidar_name","velodyne");

        std::string temp_base_frame;
        this->declare_parameter<std::string>("base_frame_id","base");

        this->declare_parameter<bool>("enable_head",true);
        this->declare_parameter<bool>("enable_lidar",true);

        this->get_parameter("depthcam1_name",depth1_name);
        this->get_parameter("depthcam2_name",depth2_name);
        this->get_parameter("lidar_name",lidar_name);

        this->get_parameter("base_frame_id",temp_base_frame);
        std::strcpy(base_frame_id, temp_base_frame.c_str());

        this->get_parameter("enable_head",enable_head);
        this->get_parameter("enable_lidar",enable_lidar);
        
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_ -> setUsingDedicatedThread(true);

        char depth1_topic_name[100];
        char depth2_topic_name[100];
        char lidar_topic_name[100];
        char depth1_frame_id[100];
        char depth2_frame_id[100];
        char lidar_frame_id[100];

        if(enable_head){
            std::sprintf(depth1_topic_name,"/%s/depth/color/points",depth1_name.c_str());
            std::sprintf(depth2_topic_name,"/%s/depth/color/points",depth2_name.c_str());
            std::sprintf(depth1_frame_id  ,"%s_depth_optical_frame",depth1_name.c_str());
            std::sprintf(depth2_frame_id  ,"%s_depth_optical_frame",depth2_name.c_str());

            get_tf_eigen4(depth1_frame_id,base_frame_id,depth1_tfstamped,depth1_eigen4);
            get_tf_eigen4(depth2_frame_id,base_frame_id,depth2_tfstamped,depth2_eigen4);
            // get_tf_eigen4(source_lidar ,target_frame, lidar_tfstamped, lidar_eigen4);

            subscriber_depth1_.subscribe(this, std::string(depth1_topic_name), rmw_qos_profile);
            subscriber_depth2_.subscribe(this, std::string(depth2_topic_name), rmw_qos_profile);

            // depth_sync_ = std::make_shared<message_filters::TimeSynchronizer<PCL2,PCL2>>(subscriber_depth1_, subscriber_depth2_, 10);
            depth_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(3),subscriber_depth1_,subscriber_depth2_);
            
            depth_sync_->registerCallback(std::bind(&SyncerNode::DepthSyncCallback, this, std::placeholders::_1, std::placeholders::_2));

            publisher_depth_ = this->create_publisher<PCL2>("raibo_head_pcl2",3);

            RCLCPP_INFO(this->get_logger(),"Depth Initialized with:\ntopic names: [ %s , %s ]\nframe id's : [ %s , %s ]",depth1_topic_name,depth2_topic_name,depth1_frame_id,depth2_frame_id);
        }

        if(enable_lidar){
            std::sprintf(lidar_topic_name,"/%s_points",lidar_name.c_str());
            std::sprintf(lidar_frame_id,"%s",lidar_name.c_str());

            get_tf_eigen4(lidar_frame_id,base_frame_id,lidar_tfstamped,lidar_eigen4);

            subscriber_lidar_.subscribe(this, std::string(lidar_topic_name), rmw_qos_profile);
            subscriber_lidar_.registerCallback(std::bind(&SyncerNode::LidarCallback,this,std::placeholders::_1));

            publisher_lidar_ = this->create_publisher<PCL2>("raibo_lidar_pcl2",3);
            RCLCPP_INFO(this->get_logger(),"LiDAR Initialized with:\ntopic name: [ %s ]\nframe id: [ %s ]",lidar_topic_name,lidar_frame_id);
        }
    }

private:
    void DepthSyncCallback(
        const PCL2::ConstSharedPtr& msg_1,
        const PCL2::ConstSharedPtr& msg_2
    ) {
        PCL2 data_1;
        PCL2 data_2;
        PCL2 data_full;

        RCLCPP_DEBUG(
            this->get_logger(),
            "Received 2 msg each at: %u ns and %u ns (Difference: %u ns)",
            (msg_1->header.stamp.nanosec),(msg_2->header.stamp.nanosec),abs(int(msg_1->header.stamp.nanosec-msg_2->header.stamp.nanosec))
        );

        pcl_ros::transformPointCloud(depth1_eigen4,*msg_1,data_1);
        pcl_ros::transformPointCloud(depth2_eigen4,*msg_2,data_2);
        pcl::concatenatePointCloud(data_1,data_2,data_full);
        data_full.header.frame_id = std::string(base_frame_id); //TODO: parametrize this!
        // data_full.header.stamp.sec     = msg_1->header.stamp.sec;
        // data_full.header.stamp.nanosec = msg_1->header.stamp.nanosec;

        publisher_depth_->publish(data_full);
    }

    void LidarCallback(const PCL2::ConstSharedPtr& msg){
        PCL2 data;
        RCLCPP_DEBUG(this->get_logger(),"Received msg at: %u ns",(msg->header.stamp.nanosec));
        pcl_ros::transformPointCloud(lidar_eigen4,*msg,data);
        
        data.header.frame_id = std::string(base_frame_id);
        publisher_lidar_->publish(data);
    }

    void get_tf_eigen4(std::string source_frame , std::string target_frame ,geometry_msgs::msg::TransformStamped& tf_stamped, Eigen::Matrix4f& tf_eigen4){
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string warning_msg;
        rclcpp::Rate rate(1.0);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        while (rclcpp::ok() && !tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePoint(), &warning_msg)){
            RCLCPP_INFO(
                this->get_logger(),"Waiting for transform %s ->  %s:\n%s\nAvailable:\n%s",
                source_frame.c_str(), target_frame.c_str(),warning_msg.c_str(),(tf_buffer_->allFramesAsString()).c_str());
            rate.sleep();
        }

        tf_stamped = tf_buffer_-> lookupTransform(target_frame,source_frame,tf2::TimePoint());
        tf_eigen4  = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();

        if(tf_stamped.transform.translation.x != 0.0){
            auto trn = tf_stamped.transform.translation;
            auto rot = tf_stamped.transform.rotation;
            RCLCPP_INFO(this->get_logger(),"Transformations successfully retreived.");
            RCLCPP_INFO(this->get_logger(),"[{%s} ->  {%s}]",source_frame.c_str(),target_frame.c_str());
            RCLCPP_INFO(this->get_logger(),"\tTRN-XYZ :[%.3f,%.3f,%.3f]"      ,trn.x,trn.y,trn.z);
            RCLCPP_INFO(this->get_logger(),"\tROT-XYZW:[%.3f,%.3f,%.3f,%.3f]" ,rot.x,rot.y,rot.z,rot.w);
        }
    }

    message_filters::Subscriber<PCL2> subscriber_depth1_;
    message_filters::Subscriber<PCL2> subscriber_depth2_;
    message_filters::Subscriber<PCL2> subscriber_lidar_;
    // std::shared_ptr<message_filters::TimeSynchronizer<PCL2,PCL2>> depth_sync_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>>    depth_sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_depth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_lidar_;

    // rclcpp::TimerBase::SharedPtr timer_;
    bool enable_head;
    bool enable_lidar;
    std::string depth1_name;
    std::string depth2_name;
    std::string lidar_name;

    char base_frame_id[100];

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped depth1_tfstamped;
    geometry_msgs::msg::TransformStamped depth2_tfstamped;
    geometry_msgs::msg::TransformStamped  lidar_tfstamped;
    Eigen::Matrix4f depth1_eigen4;
    Eigen::Matrix4f depth2_eigen4;
    Eigen::Matrix4f  lidar_eigen4;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncerNode>());
    rclcpp::shutdown();
    return 0;
}
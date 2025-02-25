#include <memory>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
using std::placeholders::_1;
using namespace std;

/* 参考ROS wiki
 * http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
 * */

int position_x ;
int position_y ;
int position_z ;
std::string imu_topic;
std::string imu_frame_id, world_frame_id;

static  std::shared_ptr<tf2_ros::TransformBroadcaster>  br;
rclcpp::Node::SharedPtr nh_=nullptr;
class imu_data_to_tf : public rclcpp::Node
{
    public:
        imu_data_to_tf():Node("imu_data_to_tf")
        {

            // node.param("/imu_tf/imu_topic", imu_topic, std::string("/imu"));
            // node.param("/imu_tf/position_x", position_x, 0);
            // node.param("/imu_tf/position_y", position_y, 0);
            // node.param("/imu_tf/position_z", position_z, 0);
            
            this->declare_parameter<std::string>("world_frame_id","/world");
            this->get_parameter("world_frame_id",  world_frame_id);

            this->declare_parameter<std::string>("imu_frame_id","/imu");
            this->get_parameter("imu_frame_id",  imu_frame_id);
            
            this->declare_parameter<std::string>("imu_topic","/imu");
            this->get_parameter("imu_topic",  imu_topic);

            this->declare_parameter<std::int16_t>("position_x",1);
            this->get_parameter("position_x",   position_x);

            this->declare_parameter<std::int16_t>("position_y",1);
            this->get_parameter("position_y", position_y);

            this->declare_parameter<std::int16_t>("position_z",1);
            this->get_parameter("position_z", position_z);
            br =std::make_shared<tf2_ros::TransformBroadcaster>(this);
            sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic.c_str(), 10, std::bind(&imu_data_to_tf::ImuCallback,this,_1));

        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

        //rclcpp::Subscriber sub = node.subscribe(imu_topic.c_str(), 10, &ImuCallback);


        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {

            //static tf2_ros::TransformBroadcaster br;//广播器

            // tf2::Transform transform;
            // transform.setOrigin(tf2::Vector3(position_x, position_y, position_z));//设置平移部分

            // 从IMU消息包中获取四元数数据
            tf2::Quaternion q;
            q.setX(imu_data->orientation.x);
            q.setY(imu_data->orientation.y);
            q.setZ(imu_data->orientation.z);
            q.setW(imu_data->orientation.w);
            q.normalized();//归一化

            // transform.setRotation(q);//设置旋转部分
            //广播出去
            //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));
            geometry_msgs::msg::TransformStamped tfs;
            tfs.header.stamp=rclcpp::Node::now();
            tfs.header.frame_id ="world";
            tfs.child_frame_id="imu";
            tfs.transform.translation.x=position_x;
            tfs.transform.translation.y=position_y;
            tfs.transform.translation.z=position_z;
            tfs.transform.rotation.x=q.getX();
            tfs.transform.rotation.y=q.getY();
            tfs.transform.rotation.z=q.getZ();
            tfs.transform.rotation.w=q.getW();
            br->sendTransform(tfs);
            //tf2::(transform, rclcpp::Node::now(), "world", "imu")
        }
};

int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    //ros::NodeHandle node;, "imu_data_to_tf"
    //auto node =std::make_shared<imu_data_to_tf>();
    rclcpp::spin(std::make_shared<imu_data_to_tf>());
    rclcpp::shutdown();
    return 0;
}

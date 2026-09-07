#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <cmath>

using namespace std::chrono_literals;

class OdomSerialNode : public rclcpp::Node
{
public:
    OdomSerialNode() : Node("odom_publisher"), io_(), serial_(io_)
    {
        // Robot Params
        wheel_separation = this->declare_parameter("wheel_separation", 0.165);
        wheel_radius = this->declare_parameter("wheel_radius", 0.034);
        left_ticks_per_revolute = this->declare_parameter("left_TPR", 4391);
        right_ticks_per_revolute = this->declare_parameter("right_TPR", 4391);

        // Publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&OdomSerialNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Open Serial with Boost.Asio
        try {
            serial_.open("/dev/ttyACM0");
            serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));
        }
        catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Timer (100Hz)
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&OdomSerialNode::readSerial, this)
        );

        RCLCPP_INFO(this->get_logger(), "Odom Serial Node with Boost.Asio Started");
    }

private:
    // ---------------- Members ----------------
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;

    // ---------------- Robot Params -----------
    double wheel_radius;
    double wheel_separation;
    int left_ticks_per_revolute;
    int right_ticks_per_revolute;

    // Odom Variables
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double last_left_distance_ = 0.0, last_right_distance_ = 0.0;
    // ---------------- CMD_VEL ----------------
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Read Command velocities
        double linear_command = msg->linear.x;
        double angular_command = msg->angular.z;

        //Compute wheel velocities
        const double velocity_left = (linear_command - angular_command * wheel_separation / 2.0) / wheel_radius;
        const double velocity_right = (linear_command + angular_command * wheel_separation / 2.0) / wheel_radius;

        int16_t left_cmd = (int16_t)(velocity_left * 1000.0);
        int16_t right_cmd = (int16_t)(velocity_right * 1000.0);    

        uint8_t packet[6];
        packet[0] = 0xAA;
        packet[1] = left_cmd & 0xFF;        // low byte
        packet[2] = (left_cmd >> 8) & 0xFF; // high byte
        packet[3] = right_cmd & 0xFF;
        packet[4] = (right_cmd >> 8) & 0xFF;

        // checksum = sum of first 5 bytes
        uint8_t sum = 0;
        for (int i = 0; i < 5; i++) sum += packet[i];
            packet[5] = sum;

        // send
        boost::asio::write(serial_, boost::asio::buffer(packet, 6));
    }

    // ---------------- SERIAL READ ----------------
    void readSerial()
    {
        try {
            // Check if at least 14 bytes available

            uint8_t header;
            boost::asio::read(serial_, boost::asio::buffer(&header, 1));

            if (header != 0x55) return;

            uint8_t buffer[13];
            boost::asio::read(serial_, boost::asio::buffer(buffer, 13));

            uint8_t checksum_calc = 0;
            for (int i = 0; i < 12; i++) checksum_calc += buffer[i];
            checksum_calc &= 0xFF;

            if (checksum_calc != buffer[12]) {
                RCLCPP_WARN(this->get_logger(), "Checksum failed");
                return;
            }

            int32_t enc_left, enc_right;
            float yaw;

            memcpy(&enc_left, &buffer[0], 4);
            memcpy(&enc_right, &buffer[4], 4);
            memcpy(&yaw, &buffer[8], 4);
            RCLCPP_INFO(this->get_logger(), "enc_l: %d enc_r: %d yaw: %.3f", enc_left, enc_right, yaw);

            updateOdometry(enc_left, enc_right, yaw);

        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        }
    }

    void updateOdometry(int32_t left_count, int32_t right_count, double imu_yaw)
    {   
        double left_radian_per_count = (2 * M_PI) / left_ticks_per_revolute;   // radian per count
        double right_radian_per_count = (2 * M_PI) / right_ticks_per_revolute;

        double left_wheel_radian = left_radian_per_count * left_count;  // wheel position radian
        double right_wheel_radian = right_radian_per_count * right_count;

        double left_pos = left_wheel_radian * wheel_radius;  // arc length formula (s = theta * radius)
        double right_pos = right_wheel_radian * wheel_radius;

        double d_left = left_pos - last_left_distance_;
        double d_right = right_pos - last_right_distance_;
        
        double d_center = (d_left + d_right) * 0.5;
        double d_theta_enc = (d_right - d_left) / wheel_separation;

        // ---------------- IMU fusion ----------------
        double imu_error = imu_yaw - theta_;

         // normalize [-pi, pi]
        while (imu_error > M_PI)  imu_error -= 2.0 * M_PI;
        while (imu_error < -M_PI) imu_error += 2.0 * M_PI;

        // complementary fusion
        const double alpha = 0.85;   // IMU weight (0.7–0.95)

        double d_theta = alpha * imu_error + (1.0 - alpha) * d_theta_enc;

        theta_ += d_theta;

        // normalize theta
        while (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        x_ += d_center * std::cos(theta_);
        y_ += d_center * std::sin(theta_);

        last_left_distance_ = left_pos;
        last_right_distance_ = right_pos;

        publishOdometry();
    }

    void publishOdometry()
    {
        rclcpp::Time stamp = now();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);
    }
        
};

// ---------------- MAIN ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSerialNode>());
    rclcpp::shutdown();
    return 0;
}

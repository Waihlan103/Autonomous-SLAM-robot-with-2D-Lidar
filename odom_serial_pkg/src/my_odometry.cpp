#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/asio.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <algorithm>

// Linux serial
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::chrono_literals;


class OdomSerialNode : public rclcpp::Node
{
public:

    OdomSerialNode()
        : Node("my_odometry"),
          io_(),
          serial_(io_)
    {
        // ============================================================
        // ROBOT PARAMETERS
        // ============================================================

        wheel_separation =
            this->declare_parameter<double>(
                "wheel_separation", 0.165);

        wheel_radius =
            this->declare_parameter<double>(
                "wheel_radius", 0.034);

        left_ticks_per_revolute =
            this->declare_parameter<int>(
                "left_TPR", 4391);

        right_ticks_per_revolute =
            this->declare_parameter<int>(
                "right_TPR", 4391);


        // ============================================================
        // PUBLISHERS
        // ============================================================

        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/odom",
                10);

        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(
                this);


        // ============================================================
        // CMD VEL SUBSCRIBER
        // ============================================================

        cmd_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10,
                std::bind(
                    &OdomSerialNode::cmdVelCallback,
                    this,
                    std::placeholders::_1)
            );


        // ============================================================
        // OPEN SERIAL
        // ============================================================

        try
        {
            serial_.open("/dev/ttyACM0");

            serial_.set_option(
                boost::asio::serial_port_base::baud_rate(115200));

            serial_.set_option(
                boost::asio::serial_port_base::character_size(8));

            serial_.set_option(
                boost::asio::serial_port_base::parity(
                    boost::asio::serial_port_base::parity::none));

            serial_.set_option(
                boost::asio::serial_port_base::stop_bits(
                    boost::asio::serial_port_base::stop_bits::one));

            serial_.set_option(
                boost::asio::serial_port_base::flow_control(
                    boost::asio::serial_port_base::flow_control::none));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open serial port: %s",
                e.what());

            rclcpp::shutdown();
            return;
        }


        // ============================================================
        // SERIAL TIMER
        //
        // Check serial data every 1 ms
        // ============================================================

        serial_timer_ =
            this->create_wall_timer(
                1ms,
                std::bind(
                    &OdomSerialNode::readSerial,
                    this)
            );


        // ============================================================
        // ODOMETRY TIMER
        //
        // 50 Hz = 20 ms
        // ============================================================

        odom_timer_ =
            this->create_wall_timer(
                20ms,
                std::bind(
                    &OdomSerialNode::publishOdometry,
                    this)
            );


        last_update_time_ =
            this->now();


        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "Odom Serial Node Started");

        RCLCPP_INFO(
            this->get_logger(),
            "Serial: /dev/ttyACM0");

        RCLCPP_INFO(
            this->get_logger(),
            "Baudrate: 115200");

        RCLCPP_INFO(
            this->get_logger(),
            "Odometry: 50 Hz");

        RCLCPP_INFO(
            this->get_logger(),
            "Wheel separation: %.3f m",
            wheel_separation);

        RCLCPP_INFO(
            this->get_logger(),
            "Wheel radius: %.3f m",
            wheel_radius);

        RCLCPP_INFO(
            this->get_logger(),
            "Left TPR: %d",
            left_ticks_per_revolute);

        RCLCPP_INFO(
            this->get_logger(),
            "Right TPR: %d",
            right_ticks_per_revolute);

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");
    }


private:

    // ================================================================
    // ROS OBJECTS
    // ================================================================

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_sub_;

    rclcpp::TimerBase::SharedPtr
        serial_timer_;

    rclcpp::TimerBase::SharedPtr
        odom_timer_;


    // ================================================================
    // SERIAL
    // ================================================================

    boost::asio::io_service io_;

    boost::asio::serial_port serial_;

    std::vector<uint8_t> rx_buffer_;


    // ================================================================
    // ROBOT PARAMETERS
    // ================================================================

    double wheel_separation;
    double wheel_radius;

    int left_ticks_per_revolute;
    int right_ticks_per_revolute;


    // ================================================================
    // ODOMETRY STATE
    // ================================================================

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    double last_left_distance_ = 0.0;
    double last_right_distance_ = 0.0;


    // ================================================================
    // LATEST SERIAL DATA
    // ================================================================

    int32_t latest_left_count_ = 0;
    int32_t latest_right_count_ = 0;

    double latest_imu_yaw_ = 0.0;

    bool first_encoder_packet_ = true;

    bool new_encoder_data_ = false;


    // ================================================================
    // VELOCITY
    // ================================================================

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    rclcpp::Time last_update_time_;


    // ================================================================
    // CMD VEL CALLBACK
    // ================================================================

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double linear_command =
            msg->linear.x;

        const double angular_command =
            msg->angular.z;


        // ------------------------------------------------------------
        // Differential drive inverse kinematics
        // ------------------------------------------------------------

        const double velocity_left =
            (
                linear_command
                -
                angular_command *
                wheel_separation / 2.0
            )
            /
            wheel_radius;


        const double velocity_right =
            (
                linear_command
                +
                angular_command *
                wheel_separation / 2.0
            )
            /
            wheel_radius;


        // ------------------------------------------------------------
        // rad/s -> integer
        //
        // STM32 expects value * 1000
        // ------------------------------------------------------------

        int16_t left_cmd =
            static_cast<int16_t>(
                velocity_left * 1000.0);

        int16_t right_cmd =
            static_cast<int16_t>(
                velocity_right * 1000.0);


        // ------------------------------------------------------------
        // Packet
        //
        // [0]    Header 0xAA
        // [1]    Left LOW
        // [2]    Left HIGH
        // [3]    Right LOW
        // [4]    Right HIGH
        // [5]    Checksum
        // ------------------------------------------------------------

        uint8_t packet[6];

        packet[0] = 0xAA;

        packet[1] =
            static_cast<uint8_t>(
                left_cmd & 0xFF);

        packet[2] =
            static_cast<uint8_t>(
                (left_cmd >> 8) & 0xFF);

        packet[3] =
            static_cast<uint8_t>(
                right_cmd & 0xFF);

        packet[4] =
            static_cast<uint8_t>(
                (right_cmd >> 8) & 0xFF);


        // ------------------------------------------------------------
        // Checksum
        // ------------------------------------------------------------

        uint8_t checksum = 0;

        for (int i = 0; i < 5; i++)
        {
            checksum += packet[i];
        }

        packet[5] = checksum;


        // ------------------------------------------------------------
        // Send
        // ------------------------------------------------------------

        try
        {
            if (serial_.is_open())
            {
                boost::asio::write(
                    serial_,
                    boost::asio::buffer(
                        packet,
                        sizeof(packet)));
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Serial write error: %s",
                e.what());
        }
    }


    // ================================================================
    // SERIAL READ
    //
    // NON-BLOCKING
    // ================================================================

    void readSerial()
    {
        try
        {
            if (!serial_.is_open())
                return;


            // --------------------------------------------------------
            // Get Linux serial file descriptor
            // --------------------------------------------------------

            int fd =
                serial_.native_handle();


            // --------------------------------------------------------
            // Check bytes available
            // --------------------------------------------------------

            int bytes_available = 0;

            if (ioctl(
                    fd,
                    FIONREAD,
                    &bytes_available) < 0)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "ioctl(FIONREAD) failed");

                return;
            }


            // No data
            if (bytes_available <= 0)
                return;


            // --------------------------------------------------------
            // Read available bytes
            // --------------------------------------------------------

            constexpr std::size_t MAX_READ = 256;

            std::size_t bytes_to_read =
                std::min(
                    static_cast<std::size_t>(
                        bytes_available),
                    MAX_READ);


            uint8_t temp[MAX_READ];


            boost::system::error_code ec;

            std::size_t bytes_read =
                serial_.read_some(
                    boost::asio::buffer(
                        temp,
                        bytes_to_read),
                    ec);


            if (ec)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Serial read error: %s",
                    ec.message().c_str());

                return;
            }


            if (bytes_read == 0)
                return;


            // --------------------------------------------------------
            // Add bytes to RX buffer
            // --------------------------------------------------------

            rx_buffer_.insert(
                rx_buffer_.end(),
                temp,
                temp + bytes_read);


            // --------------------------------------------------------
            // Parse packets
            // --------------------------------------------------------

            parseSerialBuffer();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Serial exception: %s",
                e.what());
        }
    }


    // ================================================================
    // SERIAL PACKET PARSER
    // ================================================================

    void parseSerialBuffer()
    {
        // ============================================================
        // STM32 packet format
        //
        // Byte 0      = 0x55
        //
        // Byte 1-4    = left encoder int32
        // Byte 5-8    = right encoder int32
        // Byte 9-12   = yaw float
        //
        // Byte 13     = checksum
        //
        // Total       = 14 bytes
        // ============================================================

        constexpr std::size_t PACKET_SIZE = 14;


        while (rx_buffer_.size() >= PACKET_SIZE)
        {
            // --------------------------------------------------------
            // Find header
            // --------------------------------------------------------

            auto header_it =
                std::find(
                    rx_buffer_.begin(),
                    rx_buffer_.end(),
                    static_cast<uint8_t>(0x55));


            // No header
            if (header_it ==
                rx_buffer_.end())
            {
                rx_buffer_.clear();

                return;
            }


            // Remove bytes before header
            if (header_it !=
                rx_buffer_.begin())
            {
                rx_buffer_.erase(
                    rx_buffer_.begin(),
                    header_it);
            }


            // Need complete packet
            if (rx_buffer_.size() <
                PACKET_SIZE)
            {
                return;
            }


            // --------------------------------------------------------
            // Checksum
            //
            // checksum = sum of bytes 1..12
            // --------------------------------------------------------

            uint8_t checksum_calc = 0;

            for (int i = 1; i <= 12; i++)
            {
                checksum_calc +=
                    rx_buffer_[i];
            }


            const uint8_t checksum_received =
                rx_buffer_[13];


            // --------------------------------------------------------
            // Invalid packet
            // --------------------------------------------------------

            if (checksum_calc !=
                checksum_received)
            {
                // Remove one byte and search
                // for next possible header

                rx_buffer_.erase(
                    rx_buffer_.begin());

                continue;
            }


            // --------------------------------------------------------
            // Decode
            // --------------------------------------------------------

            int32_t enc_left = 0;

            int32_t enc_right = 0;

            float yaw_float = 0.0f;


            std::memcpy(
                &enc_left,
                &rx_buffer_[1],
                sizeof(int32_t));


            std::memcpy(
                &enc_right,
                &rx_buffer_[5],
                sizeof(int32_t));


            std::memcpy(
                &yaw_float,
                &rx_buffer_[9],
                sizeof(float));


            // --------------------------------------------------------
            // Store latest data
            // --------------------------------------------------------

            latest_left_count_ =
                enc_left;

            latest_right_count_ =
                enc_right;

            latest_imu_yaw_ =
                static_cast<double>(
                    yaw_float);


            new_encoder_data_ = true;


            // --------------------------------------------------------
            // Remove processed packet
            // --------------------------------------------------------

            rx_buffer_.erase(
                rx_buffer_.begin(),
                rx_buffer_.begin() +
                PACKET_SIZE);
        }
    }


    // ================================================================
    // UPDATE ODOMETRY
    // ================================================================

    void updateOdometry()
    {
        // No new encoder data
        if (!new_encoder_data_)
            return;


        // ------------------------------------------------------------
        // Radian per encoder count
        // ------------------------------------------------------------

        const double left_radian_per_count =
            (2.0 * M_PI)
            /
            static_cast<double>(
                left_ticks_per_revolute);


        const double right_radian_per_count =
            (2.0 * M_PI)
            /
            static_cast<double>(
                right_ticks_per_revolute);


        // ------------------------------------------------------------
        // Wheel angle
        // ------------------------------------------------------------

        const double left_wheel_radian =
            left_radian_per_count
            *
            static_cast<double>(
                latest_left_count_);


        const double right_wheel_radian =
            right_radian_per_count
            *
            static_cast<double>(
                latest_right_count_);


        // ------------------------------------------------------------
        // Wheel distance
        // ------------------------------------------------------------

        const double left_pos =
            left_wheel_radian *
            wheel_radius;


        const double right_pos =
            right_wheel_radian *
            wheel_radius;


        // ------------------------------------------------------------
        // First packet
        // ------------------------------------------------------------

        if (first_encoder_packet_)
        {
            last_left_distance_ =
                left_pos;

            last_right_distance_ =
                right_pos;


            theta_ =
                normalizeAngle(
                    latest_imu_yaw_);


            first_encoder_packet_ = false;

            new_encoder_data_ = false;

            last_update_time_ =
                this->now();

            return;
        }


        // ------------------------------------------------------------
        // Wheel displacement
        // ------------------------------------------------------------

        const double d_left =
            left_pos -
            last_left_distance_;


        const double d_right =
            right_pos -
            last_right_distance_;


        // ------------------------------------------------------------
        // Center displacement
        // ------------------------------------------------------------

        const double d_center =
            (d_left + d_right)
            *
            0.5;


        // ------------------------------------------------------------
        // Encoder heading change
        // ------------------------------------------------------------

        const double d_theta_enc =
            (d_right - d_left)
            /
            wheel_separation;


        // ------------------------------------------------------------
        // IMU fusion
        // ------------------------------------------------------------

        double imu_error =
            latest_imu_yaw_ -
            theta_;


        imu_error =
            normalizeAngle(
                imu_error);


        // IMU weight
        const double alpha = 0.85;


        // Complementary fusion
        const double d_theta =
            alpha * imu_error
            +
            (1.0 - alpha) *
            d_theta_enc;


        // ------------------------------------------------------------
        // Update heading
        // ------------------------------------------------------------

        theta_ += d_theta;

        theta_ =
            normalizeAngle(theta_);


        // ------------------------------------------------------------
        // Update position
        // ------------------------------------------------------------

        x_ +=
            d_center *
            std::cos(theta_);


        y_ +=
            d_center *
            std::sin(theta_);


        // ------------------------------------------------------------
        // Velocity
        // ------------------------------------------------------------

        const rclcpp::Time current_time =
            this->now();


        const double dt =
            (
                current_time -
                last_update_time_
            ).seconds();


        if (dt > 0.0001 &&
            dt < 1.0)
        {
            linear_velocity_ =
                d_center / dt;


            angular_velocity_ =
                d_theta / dt;
        }


        // ------------------------------------------------------------
        // Save
        // ------------------------------------------------------------

        last_left_distance_ =
            left_pos;

        last_right_distance_ =
            right_pos;


        last_update_time_ =
            current_time;


        new_encoder_data_ = false;
    }


    // ================================================================
    // PUBLISH ODOMETRY
    //
    // 50 Hz
    // ================================================================

    void publishOdometry()
    {
        // Update position if new serial data arrived
        updateOdometry();


        const rclcpp::Time stamp =
            this->now();


        // ------------------------------------------------------------
        // Quaternion
        // ------------------------------------------------------------

        tf2::Quaternion q;

        q.setRPY(
            0.0,
            0.0,
            theta_);


        // ------------------------------------------------------------
        // Odometry message
        // ------------------------------------------------------------

        nav_msgs::msg::Odometry odom;


        odom.header.stamp =
            stamp;


        odom.header.frame_id =
            "odom";


        odom.child_frame_id =
            "base_link";


        // Position
        odom.pose.pose.position.x =
            x_;

        odom.pose.pose.position.y =
            y_;

        odom.pose.pose.position.z =
            0.0;


        // Orientation
        odom.pose.pose.orientation =
            tf2::toMsg(q);


        // Velocity
        odom.twist.twist.linear.x =
            linear_velocity_;

        odom.twist.twist.linear.y =
            0.0;

        odom.twist.twist.linear.z =
            0.0;


        odom.twist.twist.angular.x =
            0.0;

        odom.twist.twist.angular.y =
            0.0;

        odom.twist.twist.angular.z =
            angular_velocity_;


        // Publish
        odom_pub_->publish(odom);


        // ============================================================
        // TF: odom -> base_link
        // ============================================================

        geometry_msgs::msg::TransformStamped transform;


        transform.header.stamp =
            stamp;


        transform.header.frame_id =
            "odom";


        transform.child_frame_id =
            "base_link";


        transform.transform.translation.x =
            x_;

        transform.transform.translation.y =
            y_;

        transform.transform.translation.z =
            0.0;


        transform.transform.rotation =
            tf2::toMsg(q);


        tf_broadcaster_->sendTransform(
            transform);
    }


    // ================================================================
    // ANGLE NORMALIZATION
    // ================================================================

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -=
                2.0 * M_PI;
        }


        while (angle < -M_PI)
        {
            angle +=
                2.0 * M_PI;
        }


        return angle;
    }
};


// ====================================================================
// MAIN
// ====================================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);


    auto node =
        std::make_shared<OdomSerialNode>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}

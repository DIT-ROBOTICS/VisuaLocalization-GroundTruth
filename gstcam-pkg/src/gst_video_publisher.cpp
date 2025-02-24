#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#define RTSP_URL "rtspsrc location=rtsp://192.168.50.60:8554/camera latency=50 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink sync=false"
#define CAMERA_INFO_FILE "/home/ceiling-cam/vision-ws/src/gstcam/ceiling_calibration.yaml"  

using namespace std;
using namespace cv;

class GStreamerVideoPublisher : public rclcpp::Node {
public:
    GStreamerVideoPublisher() : Node("gst_video_publisher") {
        pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/ceiling/camera_info", 10);
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/ceiling/image", 10);
        
        cap_.open(RTSP_URL, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open RTSP stream...");
            rclcpp::shutdown();
            return;
        }

        if (!loadCameraInfo(CAMERA_INFO_FILE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera info...");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(33ms, std::bind(&GStreamerVideoPublisher::publish_frame, this));
    }

private:
    bool loadCameraInfo(const string& filename) {
        try {
            YAML::Node config = YAML::LoadFile(filename);
            camera_info_.header.frame_id = config["camera_name"].as<string>();
            camera_info_.width = config["image_width"].as<int>();
            camera_info_.height = config["image_height"].as<int>();
            camera_info_.distortion_model = "plumb_bob";

            auto K_data = config["camera_matrix"]["data"].as<std::vector<double>>();
            std::copy(K_data.begin(), K_data.end(), camera_info_.k.begin());

            auto R_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
            std::copy(R_data.begin(), R_data.end(), camera_info_.r.begin());
            
            auto D_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            camera_info_.d.assign(D_data.begin(), D_data.end());

            auto P_data = config["projection_matrix"]["data"].as<std::vector<double>>();
            std::copy(P_data.begin(), P_data.end(), camera_info_.p.begin());

            return true;
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        }
    }

    void publish_frame() {
        // static auto last_time = this->now();
        // static int frame_count = 0;

        Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;
        
        // double timestamp_msec = cap_.get(cv::CAP_PROP_POS_MSEC);
        // rclcpp::Time frame_time(static_cast<int64_t>(timestamp_msec * 1e6));
        // Publish CameraInfo and Image
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        rclcpp::Time frame_time = this->get_clock()->now();
        msg->header.stamp = frame_time;
        camera_info_.header.stamp = frame_time;

        pub_image_->publish(*msg);
        pub_camera_info_->publish(camera_info_);

        // Calculate FPS
        // frame_count++;
        // auto now_time = this->now();
        // auto duration = (now_time - last_time).seconds();
        // if (duration >= 1.0) {
        //     RCLCPP_INFO(this->get_logger(), "Calculated FPS: %d", frame_count);
        //     frame_count = 0;
        //     last_time = now_time;
        // }
    }

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GStreamerVideoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

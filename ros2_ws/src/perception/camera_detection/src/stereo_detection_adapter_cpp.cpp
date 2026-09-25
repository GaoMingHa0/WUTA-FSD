#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <wuta_msgs/msg/camera_cone_detection_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using DetectionArray = wuta_msgs::msg::CameraConeDetectionArray;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

std::int64_t stamp_ns(const std_msgs::msg::Header &header) {
    return static_cast<std::int64_t>(header.stamp.sec) * 1000000000LL +
           header.stamp.nanosec;
}

bool depth_format_valid(const Image &image) {
    const std::size_t bytes = image.encoding == "32FC1" ? 4 : image.encoding == "16UC1" ? 2 : 0;
    return bytes != 0 && image.width > 0 && image.height > 0 &&
           static_cast<std::size_t>(image.step) >= static_cast<std::size_t>(image.width) * bytes &&
           image.data.size() >= static_cast<std::size_t>(image.step) * image.height;
}

float read_depth(const Image &image, int x, int y) {
    const bool float_encoding = image.encoding == "32FC1";
    const int count = float_encoding ? 4 : 2;
    const std::uint8_t *raw = image.data.data() + static_cast<std::size_t>(y) * image.step +
                              static_cast<std::size_t>(x) * count;
    std::uint32_t bits = 0;
    if (image.is_bigendian) {
        for (int i = 0; i < count; ++i) bits = (bits << 8) | raw[i];
    } else {
        for (int i = count - 1; i >= 0; --i) bits = (bits << 8) | raw[i];
    }
    if (!float_encoding) return static_cast<float>(bits) * 0.001F;
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

double percentile(std::vector<float> &values, double percentage) {
    const double rank = (values.size() - 1) * percentage;
    const auto lower = static_cast<std::size_t>(std::floor(rank));
    const auto upper = static_cast<std::size_t>(std::ceil(rank));
    std::nth_element(values.begin(), values.begin() + lower, values.end());
    const double first = values[lower];
    if (lower == upper) return first;
    std::nth_element(values.begin(), values.begin() + upper, values.end());
    return first + (values[upper] - first) * (rank - lower);
}

double median(std::vector<float> &values) {
    return percentile(values, 0.5);
}

bool estimate_position(const Image &depth, const CameraInfo &info,
                       wuta_msgs::msg::CameraConeDetection &detection) {
    const auto &box = detection.bbox_xyxy;
    const auto &p = info.p;
    for (const double coordinate : box) if (!std::isfinite(coordinate)) return false;
    if (box[2] <= box[0] || box[3] <= box[1] ||
        !std::isfinite(p[0]) || !std::isfinite(p[5]) || p[0] <= 0 || p[5] <= 0) return false;

    const double center_x = (box[0] + box[2]) / 2;
    const double center_y = (box[1] + box[3]) / 2;
    const double extent_x = (box[2] - box[0]) * 0.2;
    const double extent_y = (box[3] - box[1]) * 0.2;
    const int x0 = std::max(0, static_cast<int>(std::floor(center_x - extent_x)));
    const int y0 = std::max(0, static_cast<int>(std::floor(center_y - extent_y)));
    const int x1 = std::min(static_cast<int>(depth.width),
                            static_cast<int>(std::ceil(center_x + extent_x)));
    const int y1 = std::min(static_cast<int>(depth.height),
                            static_cast<int>(std::ceil(center_y + extent_y)));
    if (x1 <= x0 || y1 <= y0) return false;

    const std::size_t roi_area = static_cast<std::size_t>(x1 - x0) * (y1 - y0);
    std::vector<float> distances;
    std::vector<float> xs, ys;
    distances.reserve(roi_area);
    xs.reserve(roi_area);
    ys.reserve(roi_area);
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const float z = read_depth(depth, x, y);
            if (std::isfinite(z) && z > 0.3F && z < 40.0F) {
                distances.push_back(z);
                xs.push_back(static_cast<float>(x));
                ys.push_back(static_cast<float>(y));
            }
        }
    }
    if (distances.size() < 6 || distances.size() < 0.6 * roi_area) return false;
    const double z = median(distances);
    const double spread = percentile(distances, 0.9) - percentile(distances, 0.1);
    if (spread > std::max(0.3, z * 0.08)) return false;
    const double u = median(xs), v = median(ys);
    const double fx = p[0], fy = p[5], cx = p[2], cy = p[6];
    const double x = ((u - cx) * z - p[3]) / fx;
    const double y = ((v - cy) * z - p[7]) / fy;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;

    const double sigma_z = std::max(0.03 + 0.0015 * z * z, spread / 2);
    const std::array<std::array<double, 3>, 3> jacobian{{
        {{z / fx, 0, (u - cx) / fx}},
        {{0, z / fy, (v - cy) / fy}},
        {{0, 0, 1}}
    }};
    const std::array<double, 3> variance{{4.0, 4.0, sigma_z * sigma_z}};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double value = row == col ? 0.025 * 0.025 : 0;
            for (int k = 0; k < 3; ++k) {
                value += jacobian[row][k] * variance[k] * jacobian[col][k];
            }
            detection.position_covariance[row * 3 + col] = value;
        }
    }
    detection.position.x = x;
    detection.position.y = y;
    detection.position.z = z;
    detection.position_valid = true;
    return true;
}

class StereoAdapterCpp : public rclcpp::Node {
public:
    StereoAdapterCpp() : Node("stereo_detection_adapter_cpp") {
        const auto boxes_topic = declare_parameter<std::string>("boxes_topic", "/camera/yolo/cones");
        const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/left/depth_registered");
        const auto info_topic = declare_parameter<std::string>("info_topic", "/camera/left/camera_info");
        const auto output_topic = declare_parameter<std::string>("output_topic", "/perception/camera/cones");
        const auto output_info_topic = declare_parameter<std::string>(
            "output_info_topic", "/perception/camera/camera_info");
        results_ = create_publisher<DetectionArray>(output_topic, 10);
        info_publisher_ = create_publisher<CameraInfo>(output_info_topic, 10);
        boxes_sub_ = create_subscription<DetectionArray>(boxes_topic, rclcpp::SensorDataQoS(),
            [this](DetectionArray::ConstSharedPtr message) {
                boxes_.push_back({std::move(message), std::chrono::steady_clock::now()});
                if (boxes_.size() > 20) boxes_.pop_front();
            });
        depth_sub_ = create_subscription<Image>(depth_topic, rclcpp::SensorDataQoS(),
            [this](Image::ConstSharedPtr message) {
                depths_.push_back(std::move(message));
                if (depths_.size() > 20) depths_.pop_front();
            });
        info_sub_ = create_subscription<CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
            [this](CameraInfo::ConstSharedPtr message) {
                info_ = std::move(message);
                info_publisher_->publish(*info_);
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { process(); });
    }

private:
    struct QueuedBoxes {
        DetectionArray::ConstSharedPtr message;
        std::chrono::steady_clock::time_point queued;
    };

    void process() {
        while (!boxes_.empty()) {
            const auto &source = boxes_.front();
            Image::ConstSharedPtr nearest;
            std::int64_t nearest_delta = 10000001;
            for (const auto &depth : depths_) {
                if (depth->header.frame_id != source.message->header.frame_id) continue;
                const auto delta = std::llabs(stamp_ns(depth->header) - stamp_ns(source.message->header));
                if (delta <= 10000000 && delta < nearest_delta) {
                    nearest = depth;
                    nearest_delta = delta;
                }
            }
            if (!nearest && std::chrono::steady_clock::now() - source.queued <
                    std::chrono::milliseconds(50)) break;
            DetectionArray result = *source.message;
            boxes_.pop_front();
            for (auto &detection : result.detections) detection.position_valid = false;
            if (nearest && info_ && info_->header.frame_id == result.header.frame_id &&
                info_->width == nearest->width && info_->height == nearest->height &&
                depth_format_valid(*nearest)) {
                for (auto &detection : result.detections) estimate_position(*nearest, *info_, detection);
            }
            results_->publish(std::move(result));
        }
    }

    std::deque<QueuedBoxes> boxes_;
    std::deque<Image::ConstSharedPtr> depths_;
    CameraInfo::ConstSharedPtr info_;
    rclcpp::Publisher<DetectionArray>::SharedPtr results_;
    rclcpp::Publisher<CameraInfo>::SharedPtr info_publisher_;
    rclcpp::Subscription<DetectionArray>::SharedPtr boxes_sub_;
    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoAdapterCpp>());
    rclcpp::shutdown();
    return 0;
}

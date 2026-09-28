#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <wuta_msgs/msg/camera_cone_detection.hpp>
#include <wuta_msgs/msg/camera_cone_detection_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

void cuda_check(cudaError_t code, const char *action) {
    if (code != cudaSuccess) {
        throw std::runtime_error(std::string(action) + ": " + cudaGetErrorString(code));
    }
}

std::size_t volume(const nvinfer1::Dims &shape) {
    std::size_t result = 1;
    for (int i = 0; i < shape.nbDims; ++i) {
        if (shape.d[i] <= 0) {
            throw std::runtime_error("LW-DETR engine must have fixed, positive tensor shapes");
        }
        result *= static_cast<std::size_t>(shape.d[i]);
    }
    return result;
}

std::size_t bytes_per_element(nvinfer1::DataType type) {
    if (type == nvinfer1::DataType::kFLOAT) return sizeof(float);
    if (type == nvinfer1::DataType::kHALF) return sizeof(__half);
    throw std::runtime_error("LW-DETR output must be FP32 or FP16");
}

float sigmoid(float value) {
    value = std::clamp(value, -80.0F, 80.0F);
    return 1.0F / (1.0F + std::exp(-value));
}

struct Detection {
    std::array<float, 4> box;
    std::array<float, 4> probabilities;
    float confidence;
    int class_id;
};

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char *message) noexcept override {
        if (severity <= Severity::kWARNING) {
            fprintf(stderr, "[TensorRT] %s\n", message);
        }
    }
};

class LwDetrEngine {
public:
    LwDetrEngine(const std::string &path, int device_id, int expected_width,
                 int expected_height, int red_color)
        : device_id_(device_id), red_color_(red_color) {
        cuda_check(cudaSetDevice(device_id_), "Select CUDA device");
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) throw std::runtime_error("Cannot open LW-DETR engine: " + path);
        const auto length = file.tellg();
        if (length <= 0) throw std::runtime_error("LW-DETR engine file is empty");
        std::vector<char> bytes(static_cast<std::size_t>(length));
        file.seekg(0);
        file.read(bytes.data(), static_cast<std::streamsize>(length));
        if (!file) throw std::runtime_error("Could not read LW-DETR engine");
        runtime_.reset(nvinfer1::createInferRuntime(logger_));
        if (!runtime_) throw std::runtime_error("Cannot create TensorRT runtime");
        engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!engine_) throw std::runtime_error("Cannot deserialize LW-DETR engine");
        context_.reset(engine_->createExecutionContext());
        if (!context_) throw std::runtime_error("Cannot create LW-DETR execution context");

        for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
            const std::string name = engine_->getIOTensorName(i);
            if (engine_->getTensorFormat(name.c_str()) != nvinfer1::TensorFormat::kLINEAR ||
                engine_->getTensorLocation(name.c_str()) != nvinfer1::TensorLocation::kDEVICE) {
                throw std::runtime_error("LW-DETR engine requires linear CUDA I/O tensors");
            }
            if (engine_->getTensorIOMode(name.c_str()) == nvinfer1::TensorIOMode::kINPUT) {
                if (!input_name_.empty()) throw std::runtime_error("Expected one LW-DETR input");
                input_name_ = name;
                input_shape_ = engine_->getTensorShape(name.c_str());
                if (engine_->getTensorDataType(name.c_str()) != nvinfer1::DataType::kFLOAT) {
                    throw std::runtime_error("Expected FP32 LW-DETR input binding");
                }
            } else {
                Output output;
                output.name = name;
                output.shape = engine_->getTensorShape(name.c_str());
                output.count = volume(output.shape);
                output.type = engine_->getTensorDataType(name.c_str());
                output.bytes = output.count * bytes_per_element(output.type);
                if (output.shape.d[output.shape.nbDims - 1] == 4) {
                    if (boxes_index_ >= 0) throw std::runtime_error("Duplicate box output");
                    boxes_index_ = static_cast<int>(outputs_.size());
                } else if (output.shape.d[output.shape.nbDims - 1] == 3) {
                    if (logits_index_ >= 0) throw std::runtime_error("Duplicate logits output");
                    logits_index_ = static_cast<int>(outputs_.size());
                } else {
                    throw std::runtime_error("Unexpected LW-DETR output shape");
                }
                outputs_.push_back(output);
            }
        }
        if (input_shape_.nbDims != 4 || input_shape_.d[0] != 1 || input_shape_.d[1] != 3 ||
            input_name_.empty() || boxes_index_ < 0 || logits_index_ < 0 || outputs_.size() != 2) {
            throw std::runtime_error("Expected [1,3,H,W] input and boxes/logits outputs");
        }
        height_ = input_shape_.d[2];
        width_ = input_shape_.d[3];
        if ((expected_width > 0 && expected_width != width_) ||
            (expected_height > 0 && expected_height != height_)) {
            throw std::runtime_error("Configured model dimensions do not match LW-DETR engine");
        }
        if (outputs_[boxes_index_].count != 300 * 4 ||
            outputs_[logits_index_].count != 300 * 3) {
            throw std::runtime_error("Expected 300 boxes and three class logits per query");
        }
        cuda_check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "Create CUDA stream");
        const std::size_t input_bytes = volume(input_shape_) * sizeof(float);
        cuda_check(cudaHostAlloc(reinterpret_cast<void **>(&host_input_), input_bytes,
                                 cudaHostAllocDefault), "Allocate pinned input");
        cuda_check(cudaMalloc(&device_input_, input_bytes), "Allocate device input");
        if (!context_->setTensorAddress(input_name_.c_str(), device_input_)) {
            throw std::runtime_error("Bind LW-DETR input tensor failed");
        }
        for (auto &output : outputs_) {
            cuda_check(cudaHostAlloc(&output.host, output.bytes, cudaHostAllocDefault),
                       "Allocate pinned output");
            cuda_check(cudaMalloc(&output.device, output.bytes), "Allocate device output");
            if (!context_->setTensorAddress(output.name.c_str(), output.device)) {
                throw std::runtime_error("Bind LW-DETR output tensor failed");
            }
        }
    }

    ~LwDetrEngine() {
        if (stream_) cudaStreamSynchronize(stream_);
        for (auto &output : outputs_) {
            if (output.device) cudaFree(output.device);
            if (output.host) cudaFreeHost(output.host);
        }
        if (device_input_) cudaFree(device_input_);
        if (host_input_) cudaFreeHost(host_input_);
        if (stream_) cudaStreamDestroy(stream_);
    }

    int width() const { return width_; }
    int height() const { return height_; }

    std::vector<Detection> predict(const cv::Mat &image, float confidence, float iou_threshold) {
        if (image.empty() || image.type() != CV_8UC3) {
            throw std::runtime_error("Expected nonempty BGR8 camera image");
        }
        cuda_check(cudaSetDevice(device_id_), "Select CUDA device on inference thread");
        const float scale = std::min(static_cast<float>(width_) / image.cols,
                                     static_cast<float>(height_) / image.rows);
        const int resized_width = static_cast<int>(std::lround(image.cols * scale));
        const int resized_height = static_cast<int>(std::lround(image.rows * scale));
        const int left = (width_ - resized_width) / 2;
        const int top = (height_ - resized_height) / 2;
        cv::Mat canvas(height_, width_, CV_8UC3, cv::Scalar(114, 114, 114));
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(resized_width, resized_height), 0, 0, cv::INTER_LINEAR);
        resized.copyTo(canvas(cv::Rect(left, top, resized_width, resized_height)));
        const std::size_t plane = static_cast<std::size_t>(width_) * height_;
        const std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
        const std::array<float, 3> stddev{0.229F, 0.224F, 0.225F};
        for (int y = 0; y < height_; ++y) {
            const auto *row = canvas.ptr<cv::Vec3b>(y);
            for (int x = 0; x < width_; ++x) {
                const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
                for (int channel = 0; channel < 3; ++channel) {
                    const float pixel = row[x][2 - channel] / 255.0F;
                    host_input_[channel * plane + index] = (pixel - mean[channel]) / stddev[channel];
                }
            }
        }
        cuda_check(cudaMemcpyAsync(device_input_, host_input_, plane * 3 * sizeof(float),
                                   cudaMemcpyHostToDevice, stream_), "Copy input to GPU");
        if (!context_->enqueueV3(stream_)) throw std::runtime_error("LW-DETR enqueueV3 failed");
        for (auto &output : outputs_) {
            cuda_check(cudaMemcpyAsync(output.host, output.device, output.bytes,
                                       cudaMemcpyDeviceToHost, stream_), "Copy output to CPU");
        }
        cuda_check(cudaStreamSynchronize(stream_), "Wait for LW-DETR inference");

        struct Candidate {
            float score;
            int query;
            int class_id;
        };
        const auto &logits = outputs_[logits_index_];
        const auto &boxes = outputs_[boxes_index_];
        std::vector<Candidate> candidates;
        candidates.reserve(900);
        for (int query = 0; query < 300; ++query) {
            for (int class_id = 0; class_id < 3; ++class_id) {
                const float score = sigmoid(value(logits, query * 3 + class_id));
                if (std::isfinite(score)) candidates.push_back({score, query, class_id});
            }
        }
        const auto by_score = [](const Candidate &a, const Candidate &b) {
            return a.score > b.score;
        };
        if (candidates.size() > 300) {
            std::nth_element(candidates.begin(), candidates.begin() + 300, candidates.end(), by_score);
            candidates.resize(300);
        }
        std::sort(candidates.begin(), candidates.end(), by_score);
        std::vector<Detection> selected;
        for (const auto &candidate : candidates) {
            if (candidate.score < confidence) break;
            const std::size_t base = static_cast<std::size_t>(candidate.query) * 4;
            const float cx = value(boxes, base), cy = value(boxes, base + 1);
            const float bw = value(boxes, base + 2), bh = value(boxes, base + 3);
            if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(bw) || !std::isfinite(bh)) continue;
            std::array<float, 4> rect{
                std::clamp(((cx - bw / 2) * width_ - left) / scale, 0.0F, static_cast<float>(image.cols)),
                std::clamp(((cy - bh / 2) * height_ - top) / scale, 0.0F, static_cast<float>(image.rows)),
                std::clamp(((cx + bw / 2) * width_ - left) / scale, 0.0F, static_cast<float>(image.cols)),
                std::clamp(((cy + bh / 2) * height_ - top) / scale, 0.0F, static_cast<float>(image.rows))};
            if (rect[2] <= rect[0] || rect[3] <= rect[1]) continue;
            bool suppressed = false;
            for (const auto &prior : selected) {
                const float x0 = std::max(rect[0], prior.box[0]);
                const float y0 = std::max(rect[1], prior.box[1]);
                const float x1 = std::min(rect[2], prior.box[2]);
                const float y1 = std::min(rect[3], prior.box[3]);
                const float intersection = std::max(0.0F, x1 - x0) * std::max(0.0F, y1 - y0);
                const float area = (rect[2] - rect[0]) * (rect[3] - rect[1]);
                const float other = (prior.box[2] - prior.box[0]) * (prior.box[3] - prior.box[1]);
                if (intersection / (area + other - intersection) > iou_threshold) {
                    suppressed = true;
                    break;
                }
            }
            if (suppressed) continue;
            std::array<float, 4> probabilities{0, 0, 0, 0};
            const int color = candidate.class_id == 0 ? red_color_ : candidate.class_id == 1 ? 2 : 1;
            probabilities[0] = 1.0F - candidate.score;
            if (color != 0) probabilities[color] = candidate.score;
            selected.push_back({rect, probabilities, candidate.score, candidate.class_id});
        }
        return selected;
    }

private:
    struct Output {
        std::string name;
        nvinfer1::Dims shape{};
        nvinfer1::DataType type{};
        std::size_t count{};
        std::size_t bytes{};
        void *host{};
        void *device{};
    };

    static float value(const Output &output, std::size_t index) {
        if (output.type == nvinfer1::DataType::kFLOAT) {
            return static_cast<const float *>(output.host)[index];
        }
        return __half2float(static_cast<const __half *>(output.host)[index]);
    }

    TrtLogger logger_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    std::string input_name_;
    nvinfer1::Dims input_shape_{};
    std::vector<Output> outputs_;
    int boxes_index_{-1};
    int logits_index_{-1};
    int device_id_;
    int red_color_;
    int width_{};
    int height_{};
    float *host_input_{};
    void *device_input_{};
    cudaStream_t stream_{};
};

cv::Mat image_bgr(const sensor_msgs::msg::Image &message) {
    int channels = 0;
    int conversion = -1;
    if (message.encoding == "bgr8") channels = 3;
    else if (message.encoding == "rgb8") { channels = 3; conversion = cv::COLOR_RGB2BGR; }
    else if (message.encoding == "bgra8") { channels = 4; conversion = cv::COLOR_BGRA2BGR; }
    else if (message.encoding == "rgba8") { channels = 4; conversion = cv::COLOR_RGBA2BGR; }
    else throw std::runtime_error("Unsupported camera encoding: " + message.encoding);
    if (message.width == 0 || message.height == 0 ||
        message.step < message.width * channels ||
        message.data.size() < static_cast<std::size_t>(message.step) * message.height) {
        throw std::runtime_error("Invalid camera image dimensions or buffer");
    }
    cv::Mat view(message.height, message.width, CV_MAKETYPE(CV_8U, channels),
                 const_cast<std::uint8_t *>(message.data.data()), message.step);
    if (conversion < 0) return view;
    cv::Mat bgr;
    cv::cvtColor(view, bgr, conversion);
    return bgr;
}

cv::Mat annotated_image(const cv::Mat &source, const std::vector<Detection> &detections,
                        double milliseconds) {
    cv::Mat result = source.clone();
    const std::array<cv::Scalar, 4> colors{
        cv::Scalar(180, 180, 180), cv::Scalar(255, 80, 0),
        cv::Scalar(0, 255, 255), cv::Scalar(0, 140, 255)};
    const std::array<std::string, 4> names{"unknown", "blue", "yellow", "orange"};
    cv::rectangle(result, cv::Rect(0, 0, result.cols, std::min(result.rows, 35)),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    std::ostringstream status;
    status << "LW-DETR TensorRT CUDA | cones: " << detections.size() << " | "
           << static_cast<int>(std::lround(milliseconds)) << " ms";
    cv::putText(result, status.str(), cv::Point(8, 25), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    for (const auto &detection : detections) {
        const int color = static_cast<int>(std::distance(detection.probabilities.begin(),
            std::max_element(detection.probabilities.begin(), detection.probabilities.end())));
        const cv::Point start(std::lround(detection.box[0]), std::lround(detection.box[1]));
        const cv::Point end(std::lround(detection.box[2]), std::lround(detection.box[3]));
        cv::rectangle(result, start, end, colors[color], 2);
        std::ostringstream label;
        label.precision(2);
        label << names[color] << " " << std::fixed << detection.confidence;
        const cv::Point origin(start.x, std::max(18, start.y - 6));
        cv::putText(result, label.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        cv::putText(result, label.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    colors[color], 2, cv::LINE_AA);
    }
    return result;
}

class LwDetrNode : public rclcpp::Node {
public:
    LwDetrNode() : Node("lwdetr_tensorrt_node") {
        const auto model_path = declare_parameter<std::string>("model_path", "");
        const auto image_topic = declare_parameter<std::string>(
            "image_topic", "/zed/zed_node/rgb/image_rect_color");
        const auto output_topic = declare_parameter<std::string>("output_topic", "/camera/yolo/cones");
        const auto annotated_topic = declare_parameter<std::string>(
            "annotated_topic", "/camera/yolo/image_annotated");
        publish_annotated_ = declare_parameter<bool>("publish_annotated_image", true);
        confidence_ = declare_parameter<double>("confidence_threshold", 0.5);
        iou_ = declare_parameter<double>("nms_iou_threshold", 0.45);
        const int gpu_id = declare_parameter<int>("gpu_device_id", 0);
        const int red_color = declare_parameter<int>("red_color", 3);
        const int expected_width = declare_parameter<int>("model_input_width", 0);
        const int expected_height = declare_parameter<int>("model_input_height", 0);
        const int threads = declare_parameter<int>("inference_threads", 4);
        const auto device = declare_parameter<std::string>("device", "cuda");
        if (device != "cuda" || gpu_id < 0 || (red_color != 0 && red_color != 3) ||
            !std::isfinite(confidence_) || confidence_ <= 0 || confidence_ >= 1 ||
            !std::isfinite(iou_) || iou_ <= 0 || iou_ >= 1 || threads < 1 ||
            expected_width < 0 || expected_height < 0 ||
            ((expected_width == 0) != (expected_height == 0))) {
            throw std::runtime_error("Invalid LW-DETR camera node parameters");
        }
        cv::setNumThreads(threads);
        engine_ = std::make_unique<LwDetrEngine>(model_path, gpu_id, expected_width,
                                                  expected_height, red_color);
        results_ = create_publisher<wuta_msgs::msg::CameraConeDetectionArray>(output_topic, 10);
        annotated_ = create_publisher<sensor_msgs::msg::Image>(
            annotated_topic, rclcpp::SensorDataQoS());
        status_ = create_publisher<std_msgs::msg::String>("/perception/camera/yolo/status", 10);
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = std::move(image);
                }
                ready_.notify_one();
            });
        worker_ = std::thread([this]() { run(); });
        RCLCPP_INFO(get_logger(), "LW-DETR TensorRT C++ loaded: %dx%d GPU %d",
                    engine_->width(), engine_->height(), gpu_id);
    }

    ~LwDetrNode() override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    void run() {
        using Clock = std::chrono::steady_clock;
        for (;;) {
            sensor_msgs::msg::Image::ConstSharedPtr source;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this]() { return stopping_ || latest_; });
                if (stopping_) return;
                source = std::move(latest_);
            }
            const auto started = Clock::now();
            try {
                if (source->header.frame_id.empty()) throw std::runtime_error("Image frame_id is empty");
                const cv::Mat bgr = image_bgr(*source);
                const auto detections = engine_->predict(bgr, confidence_, iou_);
                const double milliseconds = std::chrono::duration<double, std::milli>(
                    Clock::now() - started).count();
                wuta_msgs::msg::CameraConeDetectionArray result;
                result.header = source->header;
                result.detections.reserve(detections.size());
                for (std::size_t i = 0; i < detections.size(); ++i) {
                    wuta_msgs::msg::CameraConeDetection detection;
                    detection.detection_id = static_cast<std::uint32_t>(i);
                    for (int j = 0; j < 4; ++j) {
                        detection.bbox_xyxy[j] = detections[i].box[j];
                        detection.color_probabilities[j] = detections[i].probabilities[j];
                    }
                    detection.confidence = detections[i].confidence;
                    detection.position_valid = false;
                    result.detections.push_back(std::move(detection));
                }
                results_->publish(std::move(result));
                if (publish_annotated_ && annotated_->get_subscription_count() > 0) {
                    const cv::Mat painted = annotated_image(bgr, detections, milliseconds);
                    sensor_msgs::msg::Image image;
                    image.header = source->header;
                    image.height = painted.rows;
                    image.width = painted.cols;
                    image.encoding = "bgr8";
                    image.is_bigendian = false;
                    image.step = painted.cols * 3;
                    image.data.assign(painted.datastart, painted.dataend);
                    annotated_->publish(std::move(image));
                }
                std::ostringstream details;
                details << "{\"detections\":" << detections.size()
                        << ",\"inference_ms\":" << milliseconds
                        << ",\"device\":\"cuda\",\"backend\":\"tensorrt-lwdetr-cpp\""
                        << ",\"providers\":[\"TensorRT:C++:cuda\"]"
                        << ",\"stamp_ns\":" <<
                    (static_cast<std::int64_t>(source->header.stamp.sec) * 1000000000LL +
                     source->header.stamp.nanosec) << "}";
                std_msgs::msg::String status;
                status.data = details.str();
                status_->publish(std::move(status));
            } catch (const std::exception &error) {
                RCLCPP_ERROR(get_logger(), "LW-DETR inference failed: %s", error.what());
            }
        }
    }

    std::unique_ptr<LwDetrEngine> engine_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<wuta_msgs::msg::CameraConeDetectionArray>::SharedPtr results_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable ready_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_;
    bool stopping_{false};
    bool publish_annotated_{true};
    double confidence_{0.5};
    double iou_{0.45};
};

}  // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<LwDetrNode>();
        rclcpp::spin(node);
        node.reset();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "LW-DETR TensorRT node startup failed: %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
}

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <wuta_msgs/msg/camera_cone_detection_array.hpp>
#include <wuta_msgs/msg/cone_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <iterator>
#include <iomanip>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using ConeArray = wuta_msgs::msg::ConeArray;
using CameraArray = wuta_msgs::msg::CameraConeDetectionArray;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using PointCloud = sensor_msgs::msg::PointCloud2;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Clock = std::chrono::steady_clock;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

std::int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Vector3 vector_of(const geometry_msgs::msg::Point &point) {
    return {point.x, point.y, point.z};
}

void assign_point(geometry_msgs::msg::Point &point, const Vector3 &value) {
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
}

bool finite(const Vector3 &value) { return value.allFinite(); }

struct Config {
    double sync_slop_sec, max_wait_sec, max_match_distance, mahalanobis_gate;
    double pixel_margin, ambiguity_margin, min_detection_confidence, min_color_probability;
    double lidar_sigma, reference_sigma, max_position_shift;
    bool fuse_positions, publish_unmatched_lidar, guided_clustering, debug_orange;
    double guided_voxel_size, guided_cluster_tolerance, guided_depth_tolerance;
    int guided_min_cluster_size, guided_max_cluster_size;
    double guided_max_width, guided_min_height, guided_max_height;
    int max_queue;
    std::string fixed_frame;
};

struct Observation {
    const wuta_msgs::msg::CameraConeDetection *detection;
    Vector3 point = Vector3::Zero();
    Matrix3 covariance = Matrix3::Zero();
    bool has_point = false;
};

struct AssociationStats {
    int mutual_matches{0};
    int hungarian_rows{0};
};

Eigen::Isometry3d transform_of(const geometry_msgs::msg::Transform &transform) {
    const auto &rotation = transform.rotation;
    Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    const Vector3 translation(transform.translation.x, transform.translation.y, transform.translation.z);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9 || !finite(translation)) {
        throw std::runtime_error("Invalid camera from LiDAR transform");
    }
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = quaternion.normalized().toRotationMatrix();
    result.translation() = translation;
    return result;
}

bool covariance_of(const std::array<double, 9> &values, double extra_sigma, Matrix3 &result) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) result(row, col) = values[row * 3 + col];
    }
    if (!result.allFinite()) return false;
    for (int row = 0; row < 3; ++row) {
        for (int col = row + 1; col < 3; ++col) {
            if (std::abs(result(row, col) - result(col, row)) >
                1e-8 + 1e-5 * std::abs(result(col, row))) return false;
        }
    }
    Eigen::LLT<Matrix3> test(result);
    if (test.info() != Eigen::Success || (test.matrixL().toDenseMatrix().diagonal().array() <= 0).any()) {
        return false;
    }
    result.diagonal().array() += extra_sigma * extra_sigma;
    return true;
}

bool project(const Vector3 &point, const std::array<double, 12> &projection,
             Eigen::Vector2d &pixel) {
    if (!finite(point) || point.z() <= 0) return false;
    const double homogeneous_x = projection[0] * point.x() + projection[1] * point.y() +
                                  projection[2] * point.z() + projection[3];
    const double homogeneous_y = projection[4] * point.x() + projection[5] * point.y() +
                                  projection[6] * point.z() + projection[7];
    const double homogeneous_z = projection[8] * point.x() + projection[9] * point.y() +
                                  projection[10] * point.z() + projection[11];
    if (!std::isfinite(homogeneous_z) || homogeneous_z <= 1e-6) return false;
    pixel = {homogeneous_x / homogeneous_z, homogeneous_y / homogeneous_z};
    return pixel.allFinite();
}

// Rectangular Hungarian assignment. One dummy column per LiDAR cone allows no match.
std::vector<std::pair<int, int>> assign_matches(const std::vector<std::vector<double>> &cost) {
    const int rows = static_cast<int>(cost.size());
    if (!rows) return {};
    const int detections = static_cast<int>(cost.front().size());
    const int columns = detections + rows;
    std::vector<double> u(rows + 1), v(columns + 1);
    std::vector<int> p(columns + 1), way(columns + 1);
    for (int row = 1; row <= rows; ++row) {
        p[0] = row;
        int col0 = 0;
        std::vector<double> minv(columns + 1, kInfinity);
        std::vector<bool> used(columns + 1, false);
        do {
            used[col0] = true;
            const int active = p[col0];
            double delta = kInfinity;
            int next = 0;
            for (int col = 1; col <= columns; ++col) {
                if (used[col]) continue;
                const double edge = col <= detections ?
                    (std::isfinite(cost[active - 1][col - 1]) ? cost[active - 1][col - 1] : 1e6) : 10.0;
                const double reduced = edge - u[active] - v[col];
                if (reduced < minv[col]) { minv[col] = reduced; way[col] = col0; }
                if (minv[col] < delta) { delta = minv[col]; next = col; }
            }
            for (int col = 0; col <= columns; ++col) {
                if (used[col]) { u[p[col]] += delta; v[col] -= delta; }
                else minv[col] -= delta;
            }
            col0 = next;
        } while (p[col0] != 0);
        do {
            const int previous = way[col0];
            p[col0] = p[previous];
            col0 = previous;
        } while (col0 != 0);
    }
    std::vector<std::pair<int, int>> matches;
    for (int col = 1; col <= detections; ++col) {
        const int row = p[col] - 1;
        if (row >= 0 && std::isfinite(cost[row][col - 1])) matches.emplace_back(row, col - 1);
    }
    std::sort(matches.begin(), matches.end());
    return matches;
}

std::vector<std::pair<int, int>> associate(const std::vector<Vector3> &points,
                                           const std::vector<Observation> &observations,
                                           const std::array<double, 12> &projection,
                                           const Config &config,
                                           AssociationStats *stats = nullptr) {
    if (stats) *stats = {};
    std::vector<std::vector<double>> cost(points.size(),
                                          std::vector<double>(observations.size(), kInfinity));
    for (std::size_t j = 0; j < observations.size(); ++j) {
        const auto &observation = observations[j];
        const auto &box = observation.detection->bbox_xyxy;
        if (!std::all_of(box.begin(), box.end(), [](double value) { return std::isfinite(value); }) ||
            box[2] <= box[0] || box[3] <= box[1]) continue;
        for (std::size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector2d pixel;
            if (!project(points[i], projection, pixel) ||
                pixel.x() < box[0] - config.pixel_margin || pixel.x() > box[2] + config.pixel_margin ||
                pixel.y() < box[1] - config.pixel_margin || pixel.y() > box[3] + config.pixel_margin) continue;
            const Eigen::Vector2d center((box[0] + box[2]) / 2, (box[1] + box[3]) / 2);
            const Eigen::Vector2d size(box[2] - box[0], box[3] - box[1]);
            const double image_cost = ((pixel - center).array() / size.array()).matrix().norm();
            if (!observation.has_point) { cost[i][j] = image_cost; continue; }
            const Vector3 residual = points[i] - observation.point;
            if (residual.norm() > config.max_match_distance) continue;
            Matrix3 covariance = observation.covariance;
            covariance.diagonal().array() += config.lidar_sigma * config.lidar_sigma;
            const Eigen::LDLT<Matrix3> solve(covariance);
            if (solve.info() != Eigen::Success) continue;
            const double distance = residual.dot(solve.solve(residual));
            if (std::isfinite(distance) && distance <= config.mahalanobis_gate) {
                cost[i][j] = distance / config.mahalanobis_gate + 0.25 * image_cost;
            }
        }
    }
    std::vector<bool> rejected_rows(cost.size(), false);
    std::vector<bool> rejected_cols(observations.size(), false);
    for (std::size_t row = 0; row < cost.size(); ++row) {
        std::vector<double> finite_costs;
        for (double value : cost[row]) if (std::isfinite(value)) finite_costs.push_back(value);
        if (finite_costs.size() > 1) {
            std::nth_element(finite_costs.begin(), finite_costs.begin() + 1, finite_costs.end());
            const double second = finite_costs[1];
            const double first = *std::min_element(finite_costs.begin(), finite_costs.end());
            if (second - first < config.ambiguity_margin) rejected_rows[row] = true;
        }
    }
    if (!cost.empty()) {
        for (std::size_t col = 0; col < cost.front().size(); ++col) {
            std::vector<double> finite_costs;
            for (const auto &row : cost) if (std::isfinite(row[col])) finite_costs.push_back(row[col]);
            if (finite_costs.size() > 1) {
                std::nth_element(finite_costs.begin(), finite_costs.begin() + 1, finite_costs.end());
                const double second = finite_costs[1];
                const double first = *std::min_element(finite_costs.begin(), finite_costs.end());
                if (second - first < config.ambiguity_margin) rejected_cols[col] = true;
            }
        }
    }
    for (std::size_t row = 0; row < cost.size(); ++row) {
        if (rejected_rows[row]) std::fill(cost[row].begin(), cost[row].end(), kInfinity);
        else for (std::size_t col = 0; col < cost[row].size(); ++col) {
            if (rejected_cols[col]) cost[row][col] = kInfinity;
        }
    }
    // Resolve isolated one-to-one edges directly. Any row/column with another
    // feasible candidate remains in the conflict subset for Hungarian.
    std::vector<int> row_best(cost.size(), -1);
    std::vector<int> col_best(observations.size(), -1);
    std::vector<int> row_degree(cost.size(), 0);
    std::vector<int> col_degree(observations.size(), 0);
    for (std::size_t row = 0; row < cost.size(); ++row) {
        for (std::size_t col = 0; col < observations.size(); ++col) {
            if (!std::isfinite(cost[row][col])) continue;
            ++row_degree[row];
            ++col_degree[col];
            if (row_best[row] < 0 || cost[row][col] < cost[row][row_best[row]]) {
                row_best[row] = static_cast<int>(col);
            }
            if (col_best[col] < 0 || cost[row][col] < cost[col_best[col]][col]) {
                col_best[col] = static_cast<int>(row);
            }
        }
    }

    std::vector<std::pair<int, int>> matches;
    std::vector<bool> used_rows(cost.size(), false);
    std::vector<bool> used_cols(observations.size(), false);
    for (std::size_t row = 0; row < row_best.size(); ++row) {
        const int col = row_best[row];
        if (col >= 0 && row_degree[row] == 1 && col_degree[col] == 1 &&
            col_best[col] == static_cast<int>(row)) {
            matches.emplace_back(static_cast<int>(row), col);
            used_rows[row] = true;
            used_cols[col] = true;
        }
    }
    if (stats) stats->mutual_matches = static_cast<int>(matches.size());

    std::vector<int> remaining_rows;
    std::vector<int> remaining_cols;
    for (std::size_t row = 0; row < cost.size(); ++row) {
        if (!used_rows[row]) remaining_rows.push_back(static_cast<int>(row));
    }
    for (std::size_t col = 0; col < observations.size(); ++col) {
        if (!used_cols[col]) remaining_cols.push_back(static_cast<int>(col));
    }
    std::vector<int> active_cols;
    for (const int col : remaining_cols) {
        const bool has_edge = std::any_of(remaining_rows.begin(), remaining_rows.end(),
            [&](int row) { return std::isfinite(cost[row][col]); });
        if (has_edge) active_cols.push_back(col);
    }
    std::vector<int> active_rows;
    for (const int row : remaining_rows) {
        const bool has_edge = std::any_of(active_cols.begin(), active_cols.end(),
            [&](int col) { return std::isfinite(cost[row][col]); });
        if (has_edge) active_rows.push_back(row);
    }
    if (!active_rows.empty() && !active_cols.empty()) {
        std::vector<std::vector<double>> residual(active_rows.size(),
            std::vector<double>(active_cols.size(), kInfinity));
        for (std::size_t row = 0; row < active_rows.size(); ++row) {
            for (std::size_t col = 0; col < active_cols.size(); ++col) {
                residual[row][col] = cost[active_rows[row]][active_cols[col]];
            }
        }
        if (stats) stats->hungarian_rows = static_cast<int>(active_rows.size());
        for (const auto &[row, col] : assign_matches(residual)) {
            matches.emplace_back(active_rows[row], active_cols[col]);
        }
    }
    std::sort(matches.begin(), matches.end());
    return matches;
}

bool read_cloud_coordinate(const PointCloud &cloud, std::size_t offset, std::uint8_t datatype,
                           double &value) {
    const auto &data = cloud.data;
    const std::size_t count = datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 :
                              datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8 : 0;
    if (!count || offset > data.size() || count > data.size() - offset) return false;
    std::uint64_t bits = 0;
    if (cloud.is_bigendian) {
        for (std::size_t i = 0; i < count; ++i) bits = (bits << 8) | data[offset + i];
    } else {
        for (std::size_t i = count; i > 0; --i) bits = (bits << 8) | data[offset + i - 1];
    }
    if (count == 4) {
        const std::uint32_t small = static_cast<std::uint32_t>(bits);
        float number;
        std::memcpy(&number, &small, sizeof(number));
        value = number;
    } else {
        std::memcpy(&value, &bits, sizeof(value));
    }
    return std::isfinite(value);
}

struct Voxel { Vector3 total = Vector3::Zero(); int count = 0; };
using GridKey = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
struct GridHash {
    std::size_t operator()(const GridKey &key) const {
        std::size_t seed = 0;
        for (const auto value : {std::get<0>(key), std::get<1>(key), std::get<2>(key)}) {
            seed ^= std::hash<std::int64_t>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

bool guided_cluster(const PointCloud &cloud, const Eigen::Isometry3d &camera_from_lidar,
                    const std::array<double, 12> &projection, const Observation &observation,
                    const Config &config, Vector3 &center, int &selected,
                    double pixel_margin,
                    std::size_t &raw_points) {
    const auto &box = observation.detection->bbox_xyxy;
    std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, Voxel> voxels;
    selected = 0;
    raw_points = 0;
    struct Field { std::uint32_t offset = 0; std::uint8_t datatype = 0; bool found = false; };
    std::array<Field, 3> fields;
    const std::array<std::string, 3> names{{"x", "y", "z"}};
    for (const auto &field : cloud.fields) {
        for (int axis = 0; axis < 3; ++axis) {
            if (field.name == names[axis]) fields[axis] = {field.offset, field.datatype, true};
        }
    }
    for (const auto &field : fields) {
        if (!field.found || (field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
                             field.datatype != sensor_msgs::msg::PointField::FLOAT64) ||
            field.offset + (field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8) >
                cloud.point_step) return false;
    }
    if (cloud.point_step == 0 || cloud.row_step < cloud.width * cloud.point_step ||
        cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height) return false;
    const bool fast_xyz = !cloud.is_bigendian && std::all_of(fields.begin(), fields.end(),
        [](const Field &field) { return field.datatype == sensor_msgs::msg::PointField::FLOAT32; });
    for (std::uint32_t y = 0; y < cloud.height; ++y) {
        for (std::uint32_t x = 0; x < cloud.width; ++x) {
            const std::size_t base = static_cast<std::size_t>(y) * cloud.row_step +
                                     static_cast<std::size_t>(x) * cloud.point_step;
            Vector3 point;
            bool valid = true;
            if (fast_xyz) {
                for (int axis = 0; axis < 3; ++axis) {
                    float coordinate;
                    std::memcpy(&coordinate, cloud.data.data() + base + fields[axis].offset,
                                sizeof(coordinate));
                    point[axis] = coordinate;
                    valid &= std::isfinite(coordinate);
                }
            } else {
                for (int axis = 0; axis < 3; ++axis) {
                    double coordinate = 0;
                    valid &= read_cloud_coordinate(cloud, base + fields[axis].offset,
                                                   fields[axis].datatype, coordinate);
                    point[axis] = coordinate;
                }
            }
            if (!valid) continue;
            ++raw_points;
            const Vector3 camera = camera_from_lidar * point;
            if (camera.z() <= 0.3 ||
                std::abs(camera.z() - observation.point.z()) > config.guided_depth_tolerance) continue;
            Eigen::Vector2d pixel;
            if (!project(camera, projection, pixel) ||
                pixel.x() < box[0] - pixel_margin || pixel.x() > box[2] + pixel_margin ||
                pixel.y() < box[1] - pixel_margin || pixel.y() > box[3] + pixel_margin) continue;
            ++selected;
            const auto key = std::make_tuple(
                static_cast<std::int64_t>(std::floor(point.x() / config.guided_voxel_size)),
                static_cast<std::int64_t>(std::floor(point.y() / config.guided_voxel_size)),
                static_cast<std::int64_t>(std::floor(point.z() / config.guided_voxel_size)));
            auto &voxel = voxels[key];
            voxel.total += point;
            ++voxel.count;
        }
    }
    if (selected < config.guided_min_cluster_size ||
        static_cast<int>(voxels.size()) < config.guided_min_cluster_size) return false;
    std::vector<Vector3> centroids;
    centroids.reserve(voxels.size());
    for (const auto &entry : voxels) centroids.push_back(entry.second.total / entry.second.count);
    std::vector<int> parent(centroids.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&parent](int index) {
        while (parent[index] != index) {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        return index;
    };
    const double tolerance = config.guided_cluster_tolerance;
    std::unordered_map<GridKey, std::vector<int>, GridHash> grid;
    std::vector<GridKey> keys;
    keys.reserve(centroids.size());
    for (std::size_t i = 0; i < centroids.size(); ++i) {
        const auto &point = centroids[i];
        const GridKey key{
            static_cast<std::int64_t>(std::floor(point.x() / tolerance)),
            static_cast<std::int64_t>(std::floor(point.y() / tolerance)),
            static_cast<std::int64_t>(std::floor(point.z() / tolerance))};
        keys.push_back(key);
        grid[key].push_back(i);
    }
    for (std::size_t i = 0; i < centroids.size(); ++i) {
        const auto &[gx, gy, gz] = keys[i];
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const auto found = grid.find({gx + dx, gy + dy, gz + dz});
                    if (found == grid.end()) continue;
                    for (int j : found->second) {
                        if (j <= static_cast<int>(i) ||
                            (centroids[i] - centroids[j]).squaredNorm() > tolerance * tolerance) continue;
                        const int first = root(static_cast<int>(i));
                        const int second = root(j);
                        if (first != second) parent[second] = first;
                    }
                }
            }
        }
    }
    std::map<int, std::vector<int>> groups;
    for (std::size_t i = 0; i < centroids.size(); ++i) groups[root(static_cast<int>(i))].push_back(i);
    double best_score = kInfinity;
    int best_size = 0;
    bool found = false;
    for (const auto &group : groups) {
        const auto &indices = group.second;
        if (static_cast<int>(indices.size()) < config.guided_min_cluster_size ||
            static_cast<int>(indices.size()) > config.guided_max_cluster_size) continue;
        Vector3 minimum = centroids[indices[0]], maximum = minimum, sum = Vector3::Zero();
        for (int index : indices) {
            minimum = minimum.cwiseMin(centroids[index]);
            maximum = maximum.cwiseMax(centroids[index]);
            sum += centroids[index];
        }
        const Vector3 dimensions = maximum - minimum;
        if (dimensions.x() >= config.guided_max_width ||
            dimensions.y() >= config.guided_max_width ||
            dimensions.z() <= config.guided_min_height ||
            dimensions.z() >= config.guided_max_height) continue;
        const Vector3 candidate = sum / indices.size();
        const double score = (camera_from_lidar * candidate - observation.point).norm();
        if (score < best_score || (score == best_score && static_cast<int>(indices.size()) > best_size)) {
            best_score = score;
            best_size = indices.size();
            center = candidate;
            found = true;
        }
    }
    return found;
}

class DetectionFusionCpp : public rclcpp::Node {
public:
    DetectionFusionCpp() : Node("detection_fusion_node") {
        const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/perception/lidar/cones_raw");
        const auto camera_topic = declare_parameter<std::string>("camera_topic", "/perception/camera/cones");
        const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/perception/camera/camera_info");
        const auto cloud_topic = declare_parameter<std::string>("pointcloud_topic", "/rslidar_points");
        const auto output_topic = declare_parameter<std::string>("output_topic", "/perception/fused/cones");
        const auto status_topic = declare_parameter<std::string>("status_topic", "/perception/fusion/status");
        const auto debug_topic = declare_parameter<std::string>("debug_orange_topic", "/perception/debug/orange_cones");
        const auto debug_status_topic = declare_parameter<std::string>("debug_orange_status_topic", "/perception/debug/orange_status");
        const auto debug_marker_topic = declare_parameter<std::string>("debug_orange_marker_topic", "/perception/debug/orange_markers");
        config_.fixed_frame = declare_parameter<std::string>("fixed_frame", "odom");
        config_.sync_slop_sec = declare_parameter<double>("sync_slop_sec", 0.03);
        config_.max_wait_sec = declare_parameter<double>("max_wait_sec", 0.10);
        config_.max_queue = declare_parameter<int>("max_queue", 20);
        config_.max_match_distance = declare_parameter<double>("max_match_distance", 0.8);
        config_.mahalanobis_gate = declare_parameter<double>("mahalanobis_gate", 11.345);
        config_.pixel_margin = declare_parameter<double>("pixel_margin", 8.0);
        config_.ambiguity_margin = declare_parameter<double>("ambiguity_margin", 0.15);
        config_.min_detection_confidence = declare_parameter<double>("min_detection_confidence", 0.5);
        config_.min_color_probability = declare_parameter<double>("min_color_probability", 0.8);
        config_.lidar_sigma = declare_parameter<double>("lidar_sigma", 0.12);
        config_.reference_sigma = declare_parameter<double>("reference_sigma", 0.15);
        config_.max_position_shift = declare_parameter<double>("max_position_shift", 0.25);
        config_.fuse_positions = declare_parameter<bool>("fuse_positions", true);
        config_.publish_unmatched_lidar = declare_parameter<bool>("publish_unmatched_lidar", true);
        config_.guided_clustering = declare_parameter<bool>("guided_clustering", false);
        config_.debug_orange = declare_parameter<bool>("debug_orange", false);
        debug_sync_slop_sec_ = declare_parameter<double>("debug_sync_slop_sec", 0.06);
        debug_pixel_margin_ = declare_parameter<double>("debug_pixel_margin", 8.0);
        config_.guided_voxel_size = declare_parameter<double>("guided_voxel_size", 0.05);
        config_.guided_cluster_tolerance = declare_parameter<double>("guided_cluster_tolerance", 0.15);
        config_.guided_depth_tolerance = declare_parameter<double>("guided_depth_tolerance", 0.4);
        config_.guided_min_cluster_size = declare_parameter<int>("guided_min_cluster_size", 5);
        config_.guided_max_cluster_size = declare_parameter<int>("guided_max_cluster_size", 200);
        config_.guided_max_width = declare_parameter<double>("guided_max_width", 0.5);
        config_.guided_min_height = declare_parameter<double>("guided_min_height", 0.08);
        config_.guided_max_height = declare_parameter<double>("guided_max_height", 0.7);
        for (double value : {config_.sync_slop_sec, config_.max_wait_sec, config_.max_match_distance,
                             config_.mahalanobis_gate, config_.lidar_sigma, config_.reference_sigma,
                             config_.max_position_shift, config_.guided_voxel_size,
                             config_.guided_cluster_tolerance, config_.guided_depth_tolerance,
                             config_.guided_max_width, config_.guided_min_height,
                             config_.guided_max_height}) {
            if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Invalid positive fusion parameter");
        }
        if (config_.fixed_frame.empty() || config_.max_queue < 1 ||
            config_.guided_min_cluster_size < 1 ||
            config_.guided_max_cluster_size < config_.guided_min_cluster_size ||
            !std::isfinite(config_.pixel_margin) || config_.pixel_margin < 0 ||
            !std::isfinite(config_.ambiguity_margin) || config_.ambiguity_margin < 0 ||
            !std::isfinite(config_.min_detection_confidence) ||
            !std::isfinite(config_.min_color_probability) ||
            config_.min_detection_confidence < 0 || config_.min_detection_confidence > 1 ||
            config_.min_color_probability < 0 || config_.min_color_probability > 1 ||
            config_.guided_max_height <= config_.guided_min_height ||
            !std::isfinite(debug_sync_slop_sec_) || debug_sync_slop_sec_ <= 0 ||
            !std::isfinite(debug_pixel_margin_) || debug_pixel_margin_ < 0 ||
            (config_.debug_orange && (debug_topic.empty() || debug_status_topic.empty() ||
                                      debug_marker_topic.empty()))) {
            throw std::runtime_error("Invalid fusion node parameters");
        }
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        publisher_ = create_publisher<ConeArray>(output_topic, 10);
        status_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
        if (config_.debug_orange) {
            debug_publisher_ = create_publisher<ConeArray>(debug_topic, 10);
            debug_status_ = create_publisher<std_msgs::msg::String>(debug_status_topic, 10);
            debug_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(debug_marker_topic, 10);
        }
        info_sub_ = create_subscription<CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
            [this](CameraInfo::ConstSharedPtr message) {
                if (message->header.frame_id.empty() || !std::all_of(message->p.begin(), message->p.end(),
                    [](double value) { return std::isfinite(value); }) ||
                    message->p[0] <= 0 || message->p[5] <= 0) return;
                info_ = std::move(message);
                process();
            });
        camera_sub_ = create_subscription<CameraArray>(camera_topic, rclcpp::SensorDataQoS(),
            [this](CameraArray::ConstSharedPtr message) {
                const auto stamp = stamp_ns(message->header.stamp);
                if (message->header.frame_id.empty() || stamp <= last_camera_stamp_) return;
                last_camera_stamp_ = stamp;
                cameras_.push_back(std::move(message));
                if (static_cast<int>(cameras_.size()) > config_.max_queue) cameras_.pop_front();
                if (config_.debug_orange && !clouds_.empty() &&
                    Clock::now() - last_cloud_received_ < std::chrono::milliseconds(100)) {
                    publish_debug_orange(*clouds_.back());
                }
                // A pending LiDAR scan may now have its closest image. Try it
                // immediately instead of waiting for the 10 ms timer tick.
                process();
            });
        lidar_sub_ = create_subscription<ConeArray>(lidar_topic, rclcpp::SensorDataQoS(),
            [this](ConeArray::ConstSharedPtr message) {
                const auto stamp = stamp_ns(message->header.stamp);
                if (message->header.frame_id.empty() || stamp <= last_lidar_stamp_) return;
                last_lidar_stamp_ = stamp;
                if (static_cast<int>(pending_.size()) >= config_.max_queue) {
                    emit(*pending_.front().message, nullptr, nullptr, nullptr, "queue_overflow");
                    pending_.pop_front();
                }
                pending_.push_back({std::move(message), Clock::now()});
                process();
            });
        cloud_sub_ = create_subscription<PointCloud>(cloud_topic, rclcpp::SensorDataQoS(),
            [this](PointCloud::ConstSharedPtr message) {
                if (message->header.frame_id.empty()) return;
                clouds_.push_back(std::move(message));
                if (static_cast<int>(clouds_.size()) > config_.max_queue) clouds_.pop_front();
                last_cloud_received_ = Clock::now();
                if (config_.debug_orange) publish_debug_orange(*clouds_.back());
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { process(); });
    }

private:
    struct Pending {
        ConeArray::ConstSharedPtr message;
        Clock::time_point queued;
    };

    void publish_debug_markers(const ConeArray &cones) {
        visualization_msgs::msg::MarkerArray array;
        visualization_msgs::msg::Marker clear;
        clear.header = cones.header;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        array.markers.push_back(clear);
        for (std::size_t i = 0; i < cones.cones.size(); ++i) {
            const auto &point = cones.cones[i].position;
            visualization_msgs::msg::Marker marker;
            marker.header = cones.header;
            marker.ns = "live_orange";
            marker.id = static_cast<int>(i * 2);
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = point;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.28;
            marker.scale.y = 0.28;
            marker.scale.z = 0.45;
            marker.color.r = 1.0F;
            marker.color.g = 0.45F;
            marker.color.a = 0.9F;
            marker.lifetime.sec = 0;
            marker.lifetime.nanosec = 250000000;
            array.markers.push_back(marker);
            marker.ns = "live_orange_xyz";
            marker.id = static_cast<int>(i * 2 + 1);
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.pose.position.z += 0.42;
            marker.scale.z = 0.17;
            std::ostringstream label;
            label << std::fixed << std::setprecision(2) << "x=" << point.x
                  << " y=" << point.y << " z=" << point.z << " m";
            marker.text = label.str();
            array.markers.push_back(marker);
        }
        debug_markers_->publish(std::move(array));
    }

    void publish_debug_orange(const PointCloud &cloud) {
        const auto cloud_stamp = stamp_ns(cloud.header.stamp);
        if (cloud_stamp <= last_debug_stamp_ || !info_) return;
        CameraArray::ConstSharedPtr camera;
        std::int64_t closest = std::numeric_limits<std::int64_t>::max();
        for (const auto &candidate : cameras_) {
            if (candidate->header.frame_id != info_->header.frame_id) continue;
            const auto delta = std::llabs(stamp_ns(candidate->header.stamp) - cloud_stamp);
            if (delta <= debug_sync_slop_sec_ * 1e9 && delta < closest) {
                camera = candidate;
                closest = delta;
            }
        }
        if (!camera) return;
        Eigen::Isometry3d camera_from_lidar;
        try {
            const auto transform = tf_buffer_->lookupTransform(
                camera->header.frame_id, cloud.header.frame_id,
                rclcpp::Time(cloud.header.stamp), rclcpp::Duration::from_nanoseconds(0));
            camera_from_lidar = transform_of(transform.transform);
        } catch (const tf2::TransformException &) {
            return;
        } catch (const std::runtime_error &) {
            return;
        }

        Config relaxed = config_;
        relaxed.guided_depth_tolerance = std::max(relaxed.guided_depth_tolerance, 0.6);
        relaxed.guided_min_cluster_size = std::min(relaxed.guided_min_cluster_size, 3);
        relaxed.guided_max_cluster_size = std::max(relaxed.guided_max_cluster_size, 300);
        relaxed.guided_max_width = std::max(relaxed.guided_max_width, 0.65);
        relaxed.guided_min_height = std::min(relaxed.guided_min_height, 0.05);
        relaxed.guided_max_height = std::max(relaxed.guided_max_height, 0.85);
        ConeArray output;
        output.header = cloud.header;
        int candidates = 0;
        int selected_points = 0;
        const auto started = Clock::now();
        for (const auto &detection : camera->detections) {
            const auto &probabilities = detection.color_probabilities;
            const double total = std::accumulate(probabilities.begin(), probabilities.end(), 0.0);
            if (!std::isfinite(detection.confidence) ||
                detection.confidence < config_.min_detection_confidence ||
                !std::all_of(probabilities.begin(), probabilities.end(),
                    [](float value) { return std::isfinite(value) && value >= 0; }) ||
                total <= 0 || probabilities[3] != *std::max_element(probabilities.begin(), probabilities.end()) ||
                probabilities[3] / total < config_.min_color_probability ||
                !detection.position_valid) continue;
            Observation observation;
            observation.detection = &detection;
            observation.point = vector_of(detection.position);
            if (!finite(observation.point) || observation.point.z() <= 0) continue;
            ++candidates;
            Vector3 center;
            int selected = 0;
            std::size_t raw_points = 0;
            const bool found = guided_cluster(cloud, camera_from_lidar, info_->p,
                observation, relaxed, center, selected, debug_pixel_margin_, raw_points);
            selected_points += selected;
            if (!found || std::any_of(output.cones.begin(), output.cones.end(),
                [&center](const auto &cone) { return (vector_of(cone.position) - center).norm() < 0.3; })) continue;
            wuta_msgs::msg::Cone cone;
            assign_point(cone.position, center);
            cone.color = wuta_msgs::msg::Cone::COLOR_ORANGE;
            cone.confidence = detection.confidence;
            output.cones.push_back(cone);
        }
        const double cluster_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
        std::ostringstream details;
        const auto output_ns = get_clock()->now().nanoseconds();
        const auto camera_stamp = stamp_ns(camera->header.stamp);
        details << "{\"stamp_ns\":" << cloud_stamp
                << ",\"source\":\"local_cluster\""
                << ",\"camera_delta_ms\":" << (stamp_ns(camera->header.stamp) - cloud_stamp) / 1e6
                << ",\"orange_candidates\":" << candidates
                << ",\"orange_cones\":" << output.cones.size()
                << ",\"selected_points\":" << selected_points
                << ",\"cluster_ms\":" << cluster_ms
                << ",\"lidar_to_output_ms\":" << (output_ns - cloud_stamp) / 1e6
                << ",\"camera_to_output_ms\":" << (output_ns - camera_stamp) / 1e6
                << ",\"end_to_end_ms\":" << (output_ns - std::min(cloud_stamp, camera_stamp)) / 1e6
                << ",\"positions\":[";
        for (std::size_t i = 0; i < output.cones.size(); ++i) {
            if (i) details << ',';
            const auto &point = output.cones[i].position;
            details << '[' << point.x << ',' << point.y << ',' << point.z << ']';
        }
        details << "]}";
        last_debug_stamp_ = cloud_stamp;
        if (!output.cones.empty()) last_debug_local_stamp_ = cloud_stamp;
        publish_debug_markers(output);
        debug_publisher_->publish(std::move(output));
        std_msgs::msg::String status;
        status.data = details.str();
        debug_status_->publish(std::move(status));
    }

    void process() {
        while (!pending_.empty()) {
            const auto &item = pending_.front();
            const auto lidar_stamp = stamp_ns(item.message->header.stamp);
            CameraArray::ConstSharedPtr camera;
            std::int64_t closest = std::numeric_limits<std::int64_t>::max();
            for (const auto &candidate : cameras_) {
                const auto delta = std::llabs(stamp_ns(candidate->header.stamp) - lidar_stamp);
                if (delta <= config_.sync_slop_sec * 1e9 && delta < closest) {
                    camera = candidate;
                    closest = delta;
                }
            }
            std::string reason = "camera_timeout";
            if (camera && info_) {
                if (info_->header.frame_id == camera->header.frame_id) {
                    try {
                        const auto transform = tf_buffer_->lookupTransform(
                            camera->header.frame_id, rclcpp::Time(camera->header.stamp),
                            item.message->header.frame_id, rclcpp::Time(item.message->header.stamp),
                            config_.fixed_frame, rclcpp::Duration::from_nanoseconds(0));
                        const auto matrix = transform_of(transform.transform);
                        PointCloud::ConstSharedPtr cloud;
                        std::int64_t cloud_delta = 1000001;
                        for (const auto &candidate : clouds_) {
                            if (candidate->header.frame_id != item.message->header.frame_id) continue;
                            const auto delta = std::llabs(stamp_ns(candidate->header.stamp) - lidar_stamp);
                            if (delta <= 1000000 && delta < cloud_delta) {
                                cloud = candidate;
                                cloud_delta = delta;
                            }
                        }
                        emit(*item.message, camera.get(), &matrix, cloud.get(), "fused");
                        pending_.pop_front();
                        while (!cameras_.empty() &&
                               stamp_ns(cameras_.front()->header.stamp) <= stamp_ns(camera->header.stamp)) {
                            cameras_.pop_front();
                        }
                        continue;
                    } catch (const tf2::TransformException &) {
                        reason = "transform_unavailable";
                    } catch (const std::runtime_error &) {
                        reason = "transform_unavailable";
                    }
                } else reason = "camera_info_frame_mismatch";
            } else if (camera) reason = "camera_info_missing";
            if (Clock::now() - item.queued < std::chrono::duration<double>(config_.max_wait_sec)) break;
            emit(*item.message, nullptr, nullptr, nullptr, reason);
            pending_.pop_front();
        }
    }

    void emit(const ConeArray &lidar, const CameraArray *camera, const Eigen::Isometry3d *matrix,
              const PointCloud *cloud, const std::string &reason) {
        ConeArray output;
        output.header = lidar.header;
        for (const auto &raw : lidar.cones) {
            if (!finite(vector_of(raw.position))) continue;
            auto cone = raw;
            cone.color = 0;
            output.cones.push_back(cone);
        }
        const int raw_count = output.cones.size();
        std::vector<std::pair<int, int>> matches;
        std::vector<Observation> observations;
        std::vector<Vector3> lidar_points;
        AssociationStats association_stats;
        double association_ms = 0.0;
        double cloud_decode_ms = 0.0;
        double guided_cluster_ms = 0.0;
        int guided_attempts = 0;
        int guided_selected_points = 0;
        std::size_t guided_raw_points = 0;
        double camera_delta_ms = std::numeric_limits<double>::quiet_NaN();
        int guided = 0, colored = 0;
        if (camera && matrix && info_) {
            camera_delta_ms = (stamp_ns(camera->header.stamp) - stamp_ns(lidar.header.stamp)) / 1e6;
            lidar_points.reserve(output.cones.size());
            std::vector<Vector3> projected;
            projected.reserve(output.cones.size());
            for (const auto &cone : output.cones) {
                lidar_points.push_back(vector_of(cone.position));
                projected.push_back(*matrix * lidar_points.back());
            }
            for (const auto &detection : camera->detections) {
                if (!std::isfinite(detection.confidence) ||
                    detection.confidence < config_.min_detection_confidence) continue;
                Observation observation;
                observation.detection = &detection;
                if (detection.position_valid &&
                    covariance_of(detection.position_covariance, config_.reference_sigma,
                                  observation.covariance)) {
                    observation.point = vector_of(detection.position);
                    observation.has_point = finite(observation.point) && observation.point.z() > 0;
                }
                observations.push_back(observation);
            }
            const auto association_started = Clock::now();
            matches = associate(projected, observations, info_->p, config_, &association_stats);
            association_ms = std::chrono::duration<double, std::milli>(
                Clock::now() - association_started).count();
            if (config_.guided_clustering && cloud) {
                for (std::size_t j = 0; j < observations.size(); ++j) {
                    const auto &observation = observations[j];
                    if (!observation.has_point || std::any_of(matches.begin(), matches.end(),
                        [j](const auto &match) { return match.second == static_cast<int>(j); })) continue;
                    Vector3 center;
                    int selected = 0;
                    std::size_t raw_points = 0;
                    const auto guided_started = Clock::now();
                    const bool found = guided_cluster(*cloud, *matrix, info_->p,
                        observation, config_, center, selected, 0.0, raw_points);
                    guided_cluster_ms += std::chrono::duration<double, std::milli>(
                        Clock::now() - guided_started).count();
                    ++guided_attempts;
                    guided_selected_points += selected;
                    guided_raw_points += raw_points;
                    if (found) {
                        wuta_msgs::msg::Cone cone;
                        assign_point(cone.position, center);
                        cone.color = 0;
                        cone.confidence = observation.detection->confidence;
                        output.cones.push_back(cone);
                        matches.emplace_back(static_cast<int>(output.cones.size() - 1), j);
                        ++guided;
                    }
                }
            }
            const auto inverse = matrix->inverse();
            for (const auto &[i, j] : matches) {
                const auto &observation = observations[j];
                const auto &probabilities = observation.detection->color_probabilities;
                double sum = 0;
                bool valid = true;
                for (const float probability : probabilities) {
                    valid &= std::isfinite(probability) && probability >= 0;
                    sum += probability;
                }
                if (valid && sum > 0) {
                    const int color = static_cast<int>(std::distance(probabilities.begin(),
                        std::max_element(probabilities.begin(), probabilities.end())));
                    if (color > 0 && probabilities[color] / sum >= config_.min_color_probability) {
                        output.cones[i].color = color;
                        ++colored;
                    }
                }
                if (i >= raw_count || !observation.has_point || !config_.fuse_positions) continue;
                const Vector3 stereo_lidar = inverse * observation.point;
                const Matrix3 rotated = inverse.linear() * observation.covariance * inverse.linear().transpose();
                const Eigen::Matrix2d camera_covariance = rotated.topLeftCorner<2, 2>();
                Eigen::LDLT<Eigen::Matrix2d> solve(camera_covariance);
                if (solve.info() != Eigen::Success) continue;
                const Eigen::Matrix2d precision_camera = solve.solve(Eigen::Matrix2d::Identity());
                const Eigen::Matrix2d precision_lidar = Eigen::Matrix2d::Identity() /
                                                          (config_.lidar_sigma * config_.lidar_sigma);
                const Eigen::Vector2d fused = (precision_camera + precision_lidar).ldlt().solve(
                    precision_camera * stereo_lidar.head<2>() +
                    precision_lidar * lidar_points[i].head<2>());
                if (fused.allFinite() && (fused - lidar_points[i].head<2>()).norm() <=
                    config_.max_position_shift) {
                    output.cones[i].position.x = fused.x();
                    output.cones[i].position.y = fused.y();
                }
            }
        }
        int matched_raw = 0;
        for (const auto &match : matches) if (match.first < raw_count) ++matched_raw;
        if (!config_.publish_unmatched_lidar) {
            std::vector<wuta_msgs::msg::Cone> filtered;
            for (std::size_t i = 0; i < output.cones.size(); ++i) {
                if (std::any_of(matches.begin(), matches.end(),
                    [i](const auto &match) { return match.first == static_cast<int>(i); })) {
                    filtered.push_back(output.cones[i]);
                }
            }
            output.cones = std::move(filtered);
        }
        const int published = output.cones.size();
        const auto output_stamp = stamp_ns(output.header.stamp);
        if (config_.debug_orange && output_stamp >= last_debug_stamp_ &&
            output_stamp != last_debug_local_stamp_) {
            ConeArray current;
            current.header = output.header;
            for (const auto &cone : output.cones) {
                if (cone.color == wuta_msgs::msg::Cone::COLOR_ORANGE) current.cones.push_back(cone);
            }
            if (!current.cones.empty()) {
                const auto now_ns = get_clock()->now().nanoseconds();
                const auto camera_stamp = camera ? stamp_ns(camera->header.stamp) : output_stamp;
                std::ostringstream debug_details;
                debug_details << "{\"stamp_ns\":" << output_stamp
                              << ",\"source\":\"fusion_fallback\""
                              << ",\"orange_candidates\":" << current.cones.size()
                              << ",\"orange_cones\":" << current.cones.size()
                              << ",\"selected_points\":0,\"cluster_ms\":0"
                              << ",\"camera_delta_ms\":" << (camera_stamp - output_stamp) / 1e6
                              << ",\"lidar_to_output_ms\":" << (now_ns - output_stamp) / 1e6
                              << ",\"camera_to_output_ms\":" << (now_ns - camera_stamp) / 1e6
                              << ",\"end_to_end_ms\":" <<
                                  (now_ns - std::min(output_stamp, camera_stamp)) / 1e6
                              << ",\"positions\":[";
                for (std::size_t i = 0; i < current.cones.size(); ++i) {
                    if (i) debug_details << ',';
                    const auto &point = current.cones[i].position;
                    debug_details << '[' << point.x << ',' << point.y << ',' << point.z << ']';
                }
                debug_details << "]}";
                last_debug_stamp_ = output_stamp;
                publish_debug_markers(current);
                debug_publisher_->publish(std::move(current));
                std_msgs::msg::String debug_status;
                debug_status.data = debug_details.str();
                debug_status_->publish(std::move(debug_status));
            }
        }
        publisher_->publish(std::move(output));
        std::ostringstream details;
        details << "{\"stamp_ns\":" << stamp_ns(lidar.header.stamp) << ",\"reason\":\""
                << reason << "\",\"lidar_cones\":" << raw_count << ",\"matches\":"
                << matches.size() << ",\"colored\":" << colored << ",\"published_cones\":"
                << published << ",\"unmatched_filtered\":" << raw_count - matched_raw
                << ",\"guided_clusters\":" << guided << ",\"camera_delta_ms\":";
        if (std::isfinite(camera_delta_ms)) details << camera_delta_ms;
        else details << "null";
        details << ",\"association_ms\":" << association_ms
                << ",\"end_to_end_ms\":" <<
                    (get_clock()->now().nanoseconds() - (camera_delta_ms < 0 && camera ?
                        stamp_ns(camera->header.stamp) : stamp_ns(lidar.header.stamp))) / 1e6
                << ",\"cloud_decode_ms\":" << cloud_decode_ms
                << ",\"guided_cluster_ms\":" << guided_cluster_ms
                << ",\"guided_attempts\":" << guided_attempts
                << ",\"guided_selected_points\":" << guided_selected_points
                << ",\"guided_raw_points\":" << guided_raw_points
                << ",\"mutual_matches\":" << association_stats.mutual_matches
                << ",\"hungarian_rows\":" << association_stats.hungarian_rows << "}";
        std_msgs::msg::String status;
        status.data = details.str();
        status_->publish(std::move(status));
    }

    Config config_{};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<ConeArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Publisher<ConeArray>::SharedPtr debug_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_status_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_;
    rclcpp::Subscription<ConeArray>::SharedPtr lidar_sub_;
    rclcpp::Subscription<CameraArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<PointCloud>::SharedPtr cloud_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<Pending> pending_;
    std::deque<CameraArray::ConstSharedPtr> cameras_;
    std::deque<PointCloud::ConstSharedPtr> clouds_;
    CameraInfo::ConstSharedPtr info_;
    std::int64_t last_lidar_stamp_{-1};
    std::int64_t last_camera_stamp_{-1};
    std::int64_t last_debug_stamp_{-1};
    std::int64_t last_debug_local_stamp_{-1};
    Clock::time_point last_cloud_received_{};
    double debug_sync_slop_sec_{0.06};
    double debug_pixel_margin_{8.0};
};

}  // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<DetectionFusionCpp>());
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "C++ fusion node failed: %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
}

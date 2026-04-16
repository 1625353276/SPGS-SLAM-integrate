/**
 * Multi-Plane Detector Implementation
 */

#include "include/multi_plane_detector.h"
#include "ORB-SLAM3/include/MapPoint.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <numeric>

namespace SPGS {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float rad2deg(float rad) { return rad * 180.0f / kPi; }
float deg2rad(float deg) { return deg * kPi / 180.0f; }

float clampedAcos(float value) {
    return std::acos(std::max(-1.0f, std::min(1.0f, value)));
}

} // namespace

// ============================================================================
// Constructor
// ============================================================================

MultiPlaneDetector::MultiPlaneDetector(int image_w, int image_h,
                                       float fx, float fy, float cx, float cy,
                                       const MultiPlaneConfig& config)
    : config_(config)
    , img_w_(image_w)
    , img_h_(image_h)
    , fx_(fx)
    , fy_(fy)
    , cx_(cx)
    , cy_(cy)
{
}

// ============================================================================
// Main Update
// ============================================================================

void MultiPlaneDetector::update(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                                const Sophus::SE3f& T_cw)
{
    last_T_cw_ = T_cw;
    pose_valid_ = true;

    total_map_points_ = static_cast<int>(map_points.size());

    // Extract visible world points
    std::vector<Eigen::Vector3f> visible_points = extractVisiblePoints(map_points, T_cw);

    if (visible_points.empty()) {
        // No visible points, degrade all planes
        for (auto& plane : planes_) {
            plane.frames_since_update++;
            updatePlaneStatus(plane, 0);
        }
        removeLostPlanes();
        return;
    }

    // Track existing planes: count inliers for each
    std::vector<int> point_assignment(visible_points.size(), -1);
    std::vector<int> plane_inlier_counts(planes_.size(), 0);

    for (size_t i = 0; i < visible_points.size(); ++i) {
        const auto& pt = visible_points[i];
        float min_dist = std::numeric_limits<float>::max();
        int best_plane = -1;

        for (size_t j = 0; j < planes_.size(); ++j) {
            if (planes_[j].status == PlaneStatus::Lost) continue;
            float dist = planes_[j].distanceToPoint(pt);
            if (dist < min_dist && dist < config_.inlier_threshold_meters) {
                min_dist = dist;
                best_plane = static_cast<int>(j);
            }
        }

        if (best_plane >= 0) {
            point_assignment[i] = best_plane;
            plane_inlier_counts[best_plane]++;
        }
    }

    // Update existing plane statuses
    for (size_t j = 0; j < planes_.size(); ++j) {
        updatePlaneStatus(planes_[j], plane_inlier_counts[j]);
    }

    // Collect unassigned points
    std::vector<Eigen::Vector3f> unassigned_points;
    for (size_t i = 0; i < visible_points.size(); ++i) {
        if (point_assignment[i] < 0) {
            unassigned_points.push_back(visible_points[i]);
        }
    }

    // Try to detect new planes from unassigned points
    while (unassigned_points.size() >= static_cast<size_t>(config_.min_points_per_plane) &&
           static_cast<int>(planes_.size()) < config_.max_planes) {

        Plane new_plane;
        std::vector<int> inlier_indices;

        if (!fitPlaneRANSAC(unassigned_points, new_plane, inlier_indices)) {
            break;
        }

        // Classify and initialize
        new_plane.type = classifyPlane(new_plane.normal, new_plane.center);
        new_plane.status = PlaneStatus::Tentative;
        new_plane.id = next_plane_id_++;
        new_plane.stability_score = 0.0f;
        new_plane.stable_frames = 0;
        new_plane.frames_since_update = 0;

        planes_.push_back(new_plane);

        // Remove inliers from unassigned
        std::vector<Eigen::Vector3f> remaining;
        for (size_t i = 0; i < unassigned_points.size(); ++i) {
            bool is_inlier = false;
            for (int idx : inlier_indices) {
                if (static_cast<int>(i) == idx) {
                    is_inlier = true;
                    break;
                }
            }
            if (!is_inlier) {
                remaining.push_back(unassigned_points[i]);
            }
        }
        unassigned_points = std::move(remaining);
    }

    // Try to merge similar planes
    mergeSimilarPlanes();

    // Remove lost planes
    removeLostPlanes();

    // Count assigned points
    assigned_map_points_ = static_cast<int>(visible_points.size() - unassigned_points.size());
}

// ============================================================================
// Query Methods
// ============================================================================

const Plane* MultiPlaneDetector::findPlaneAtPosition(const Eigen::Vector3f& world_pos,
                                                      float max_distance) const
{
    const Plane* best = nullptr;
    float min_dist = max_distance;

    for (const auto& plane : planes_) {
        if (plane.status == PlaneStatus::Lost) continue;
        float dist = plane.distanceToPoint(world_pos);
        if (dist < min_dist) {
            // Also check if point is within extent
            Eigen::Vector2f uv = plane.worldToUV(world_pos);
            if (plane.isUVInExtent(uv, 1.5f)) {
                min_dist = dist;
                best = &plane;
            }
        }
    }

    return best;
}

const Plane* MultiPlaneDetector::findPlaneById(int id) const
{
    for (const auto& plane : planes_) {
        if (plane.id == id) return &plane;
    }
    return nullptr;
}

const Plane* MultiPlaneDetector::raycastToPlane(double px, double py,
                                                 Eigen::Vector3f& intersection_world) const
{
    if (!pose_valid_) return nullptr;

    // Build ray in world space
    Eigen::Vector3f ray_cam(
        (static_cast<float>(px) - cx_) / fx_,
        (static_cast<float>(py) - cy_) / fy_,
        1.0f);
    ray_cam.normalize();

    const Sophus::SE3f T_wc = last_T_cw_.inverse();
    const Eigen::Vector3f cam_pos = T_wc.translation();
    const Eigen::Vector3f ray_world = T_wc.rotationMatrix() * ray_cam;

    // Find closest intersection
    const Plane* best = nullptr;
    float best_t = std::numeric_limits<float>::max();

    for (const auto& plane : planes_) {
        if (plane.status == PlaneStatus::Lost) continue;

        const float denom = plane.normal.dot(ray_world);
        if (std::abs(denom) < 1e-5f) continue;

        const float t = plane.normal.dot(plane.center - cam_pos) / denom;
        if (t <= 0.0f || t >= best_t) continue;

        const Eigen::Vector3f hit_pt = cam_pos + t * ray_world;
        const Eigen::Vector2f uv = plane.worldToUV(hit_pt);

        if (plane.isUVInExtent(uv, 1.25f)) {
            best_t = t;
            best = &plane;
            intersection_world = hit_pt;
        }
    }

    return best;
}

std::string MultiPlaneDetector::getStatusString() const
{
    int locked = 0, tracking = 0, tentative = 0;
    for (const auto& p : planes_) {
        if (p.status == PlaneStatus::Locked) locked++;
        else if (p.status == PlaneStatus::Tracking) tracking++;
        else if (p.status == PlaneStatus::Tentative) tentative++;
    }
    return std::to_string(planes_.size()) + " planes (L:" + std::to_string(locked) +
           " T:" + std::to_string(tracking) + " ?:" + std::to_string(tentative) + ")";
}

// ============================================================================
// Internal Methods
// ============================================================================

std::vector<Eigen::Vector3f> MultiPlaneDetector::extractVisiblePoints(
    const std::vector<ORB_SLAM3::MapPoint*>& map_points,
    const Sophus::SE3f& T_cw) const
{
    std::vector<Eigen::Vector3f> points;
    points.reserve(map_points.size());

    const Eigen::Matrix3f R_cw = T_cw.rotationMatrix();
    const Eigen::Vector3f t_cw = T_cw.translation();

    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;

        const Eigen::Vector3f p_world = mp->GetWorldPos();
        const Eigen::Vector3f p_cam = R_cw * p_world + t_cw;

        // Must be in front of camera
        if (p_cam.z() <= 0.15f) continue;

        // Must project into image
        const float u = fx_ * p_cam.x() / p_cam.z() + cx_;
        const float v = fy_ * p_cam.y() / p_cam.z() + cy_;
        if (u < 0.0f || u >= img_w_ || v < 0.0f || v >= img_h_) continue;

        points.push_back(p_world);
    }

    return points;
}

bool MultiPlaneDetector::fitPlaneRANSAC(const std::vector<Eigen::Vector3f>& points,
                                        Plane& plane_out,
                                        std::vector<int>& inlier_indices_out) const
{
    const int n = static_cast<int>(points.size());
    if (n < config_.min_points_per_plane) return false;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, n - 1);

    int best_inlier_count = 0;
    float best_residual = std::numeric_limits<float>::max();
    Eigen::Vector3f best_normal = Eigen::Vector3f(0, 1, 0);
    float best_d = 0.0f;

    for (int iter = 0; iter < config_.ransac_iterations; ++iter) {
        // Sample 3 random points
        int i0 = dist(rng), i1 = dist(rng), i2 = dist(rng);
        if (i0 == i1 || i0 == i2 || i1 == i2) continue;

        const Eigen::Vector3f& p0 = points[i0];
        const Eigen::Vector3f& p1 = points[i1];
        const Eigen::Vector3f& p2 = points[i2];

        // Compute plane normal
        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);
        const float n_norm = n.norm();
        if (n_norm < 1e-6f) continue;
        n /= n_norm;

        // Plane equation: n.dot(p) = d
        const float d = n.dot(p0);

        // Count inliers
        int inliers = 0;
        float residual_sum = 0.0f;
        for (const auto& p : points) {
            const float dist = std::abs(n.dot(p) - d);
            if (dist < config_.inlier_threshold_meters) {
                inliers++;
                residual_sum += dist;
            }
        }

        if (inliers > best_inlier_count ||
            (inliers == best_inlier_count && residual_sum < best_residual)) {
            best_inlier_count = inliers;
            best_residual = residual_sum;
            best_normal = n;
            best_d = d;
        }
    }

    if (best_inlier_count < config_.min_points_per_plane) return false;

    // Collect inlier indices
    inlier_indices_out.clear();
    for (int i = 0; i < n; ++i) {
        if (std::abs(best_normal.dot(points[i]) - best_d) < config_.inlier_threshold_meters) {
            inlier_indices_out.push_back(i);
        }
    }

    // Refine plane
    return refinePlane(points, inlier_indices_out, plane_out);
}

bool MultiPlaneDetector::refinePlane(const std::vector<Eigen::Vector3f>& points,
                                     const std::vector<int>& inlier_indices,
                                     Plane& plane_out) const
{
    if (inlier_indices.size() < static_cast<size_t>(config_.min_points_per_plane)) {
        return false;
    }

    // Compute centroid
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int idx : inlier_indices) {
        centroid += points[idx];
    }
    centroid /= static_cast<float>(inlier_indices.size());

    // Compute covariance
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int idx : inlier_indices) {
        const Eigen::Vector3f q = points[idx] - centroid;
        cov += q * q.transpose();
    }

    // PCA: eigenvalues sorted in ascending order
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) return false;

    // Normal = eigenvector with smallest eigenvalue
    plane_out.normal = solver.eigenvectors().col(0).normalized();
    plane_out.center = centroid;
    plane_out.d = plane_out.normal.dot(centroid);

    // Build tangent axes
    buildPlaneAxes(plane_out);

    // Compute extent and residuals
    std::vector<float> us, vs;
    us.reserve(inlier_indices.size());
    vs.reserve(inlier_indices.size());

    for (int idx : inlier_indices) {
        Eigen::Vector2f uv = plane_out.worldToUV(points[idx]);
        us.push_back(uv.x());
        vs.push_back(uv.y());
    }

    // Percentile extent
    auto percentileAbs = [](std::vector<float>& values) -> float {
        if (values.empty()) return 0.0f;
        for (auto& v : values) v = std::abs(v);
        size_t idx = std::min(values.size() - 1, (values.size() * 9) / 10);
        std::nth_element(values.begin(), values.begin() + idx, values.end());
        return values[idx];
    };

    plane_out.extent_u = std::max(0.05f, percentileAbs(us));
    plane_out.extent_v = std::max(0.05f, percentileAbs(vs));
    plane_out.inlier_count = static_cast<int>(inlier_indices.size());

    return true;
}

PlaneType MultiPlaneDetector::classifyPlane(const Eigen::Vector3f& normal,
                                            const Eigen::Vector3f& center) const
{
    // Check if wall (normal is mostly horizontal)
    if (std::abs(normal.y()) < config_.wall_normal_y_threshold) {
        return PlaneType::Wall;
    }

    // Check if ceiling (normal points downward)
    if (normal.y() < -config_.wall_normal_y_threshold) {
        return PlaneType::Ceiling;
    }

    // Horizontal plane (floor or table)
    if (center.y() < config_.floor_max_height) {
        return PlaneType::Floor;
    } else if (center.y() > config_.table_min_height) {
        return PlaneType::Table;
    }

    return PlaneType::Slope;
}

void MultiPlaneDetector::updatePlaneStatus(Plane& plane, int current_inlier_count)
{
    plane.inlier_count = current_inlier_count;
    plane.frames_since_update++;

    if (current_inlier_count >= config_.min_points_per_plane) {
        plane.frames_since_update = 0;

        if (plane.status == PlaneStatus::Tentative) {
            plane.stable_frames++;
            plane.stability_score = std::min(1.0f,
                static_cast<float>(plane.stable_frames) / config_.lock_stable_frames);

            if (plane.stable_frames >= config_.lock_stable_frames) {
                plane.status = PlaneStatus::Locked;
                plane.stability_score = 1.0f;
            }
        } else if (plane.status == PlaneStatus::Tracking) {
            plane.stable_frames++;
            if (plane.stable_frames >= config_.lock_stable_frames) {
                plane.status = PlaneStatus::Locked;
            }
        } else if (plane.status == PlaneStatus::Lost) {
            // Re-activated
            plane.status = PlaneStatus::Tracking;
            plane.stable_frames = 1;
        }
    } else {
        // Not enough inliers
        if (plane.status == PlaneStatus::Locked) {
            plane.status = PlaneStatus::Tracking;
            plane.stable_frames = 0;
        } else if (plane.status == PlaneStatus::Tracking) {
            plane.stable_frames = 0;
        }

        if (plane.frames_since_update > config_.max_lost_frames) {
            plane.status = PlaneStatus::Lost;
        }
    }
}

void MultiPlaneDetector::mergeSimilarPlanes()
{
    const float max_angle_rad = deg2rad(config_.merge_max_angle_deg);

    for (size_t i = 0; i < planes_.size(); ++i) {
        if (planes_[i].status == PlaneStatus::Lost) continue;

        for (size_t j = i + 1; j < planes_.size(); ++j) {
            if (planes_[j].status == PlaneStatus::Lost) continue;

            Plane& p1 = planes_[i];
            Plane& p2 = planes_[j];

            // Check angle between normals
            const float angle = clampedAcos(std::abs(p1.normal.dot(p2.normal)));
            if (angle > max_angle_rad) continue;

            // Check distance between planes
            const float dist = std::abs(p1.d - p2.d);
            if (dist > config_.merge_max_distance_meters) continue;

            // Check overlap in UV space
            // TODO: Implement proper overlap check

            // Merge: keep the one with more inliers
            if (p2.inlier_count > p1.inlier_count) {
                std::swap(p1, p2);
            }

            // Transfer inliers from p2 to p1
            p1.neighbor_planes.insert(p2.neighbor_planes.begin(), p2.neighbor_planes.end());
            p2.status = PlaneStatus::Lost;
        }
    }
}

void MultiPlaneDetector::removeLostPlanes()
{
    planes_.erase(
        std::remove_if(planes_.begin(), planes_.end(),
            [](const Plane& p) { return p.status == PlaneStatus::Lost; }),
        planes_.end());
}

void MultiPlaneDetector::buildPlaneAxes(Plane& plane)
{
    // Build a tangent vector (axis_u) perpendicular to normal
    Eigen::Vector3f tangent = (std::abs(plane.normal.y()) < 0.9f)
        ? Eigen::Vector3f::UnitY()
        : Eigen::Vector3f::UnitX();
    tangent = tangent - tangent.dot(plane.normal) * plane.normal;
    if (tangent.norm() < 1e-6f) {
        tangent = Eigen::Vector3f::UnitZ();
        tangent = tangent - tangent.dot(plane.normal) * plane.normal;
    }
    plane.axis_u = tangent.normalized();

    // Second tangent = normal × axis_u
    plane.axis_v = plane.normal.cross(plane.axis_u).normalized();
}

} // namespace SPGS

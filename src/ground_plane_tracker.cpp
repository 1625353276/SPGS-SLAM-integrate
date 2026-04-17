#include "include/ground_plane_tracker.h"

#include "ORB-SLAM3/include/MapPoint.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace SPGS {

namespace {

constexpr int   kMinCandidatePoints = 40;
constexpr int   kMinInliers = 24;
constexpr int   kRansacIters = 200;
constexpr float kInlierThresholdMeters = 0.03f;
constexpr float kMaxMeanResidualMeters = 0.02f;
constexpr float kMinViewDot = 0.20f;
constexpr int   kLockStableFrames = 5;
constexpr int   kMaxDegradedFrames = 10;
constexpr float kMaxStableAngleRad = 15.0f * 3.1415926535f / 180.0f;
constexpr float kMaxStableCenterDist = 0.20f;
constexpr float kMaxStableExtentRatioDelta = 0.60f;
constexpr float kReferenceNormalLerp = 0.08f;
constexpr float kReferenceCenterLerp = 0.10f;
constexpr float kReferenceExtentLerp = 0.12f;

Eigen::Vector3f BuildPlaneAxisU(const Eigen::Vector3f& normal)
{
    Eigen::Vector3f tangent = (std::abs(normal.y()) < 0.9f)
        ? Eigen::Vector3f::UnitY()
        : Eigen::Vector3f::UnitX();
    tangent = tangent - tangent.dot(normal) * normal;
    if (tangent.norm() < 1e-6f) {
        tangent = Eigen::Vector3f::UnitZ();
        tangent = tangent - tangent.dot(normal) * normal;
    }
    return tangent.normalized();
}

float ClampedAcos(float value)
{
    return std::acos(std::max(-1.0f, std::min(1.0f, value)));
}

} // namespace

GroundPlaneTracker::GroundPlaneTracker(int image_w, int image_h, float fx, float fy, float cx, float cy)
    : img_w_(image_w)
    , img_h_(image_h)
    , fx_(fx)
    , fy_(fy)
    , cx_(cx)
    , cy_(cy)
{
}

void GroundPlaneTracker::update(const Sophus::SE3f& T_cw,
                                const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    last_pose_cw_ = T_cw;
    last_pose_valid_ = true;

    GroundPlaneState candidate;
    if (!estimateDominantPlane(map_points, T_cw, candidate)) {
        updateStatusNoCandidate();
        return;
    }

    candidate.valid = true;
    candidate.locked = false;
    candidate.status = GroundPlaneStatus::Tracking;

    if (!plane_state_.locked) {
        plane_state_ = candidate;
        plane_state_.status = GroundPlaneStatus::Tracking;

        if (!last_candidate_valid_ || isPlaneStableWith(last_candidate_, candidate)) {
            stable_candidate_frames_++;
        } else {
            stable_candidate_frames_ = 1;
        }

        last_candidate_ = candidate;
        last_candidate_valid_ = true;

        plane_state_.stability_score =
            std::min(1.0f, static_cast<float>(stable_candidate_frames_) / static_cast<float>(kLockStableFrames));

        if (stable_candidate_frames_ >= kLockStableFrames) {
            plane_state_.locked = true;
            plane_state_.status = GroundPlaneStatus::Locked;
            plane_state_.stability_score = 1.0f;
            degraded_frames_ = 0;
        }
        return;
    }

    if (isPlaneStableWith(plane_state_, candidate)) {
        smoothUpdateReferencePlane(candidate);
        plane_state_.locked = true;
        plane_state_.status = GroundPlaneStatus::Locked;
        plane_state_.stability_score = 1.0f;
        plane_state_.inlier_count = candidate.inlier_count;
        plane_state_.mean_residual = candidate.mean_residual;
        degraded_frames_ = 0;
        last_candidate_ = candidate;
        last_candidate_valid_ = true;
        stable_candidate_frames_ = kLockStableFrames;
        return;
    }

    degraded_frames_++;
    plane_state_.status = GroundPlaneStatus::Degraded;
    plane_state_.valid = true;
    plane_state_.locked = true;
    plane_state_.stability_score =
        std::max(0.0f, 1.0f - static_cast<float>(degraded_frames_) / static_cast<float>(kMaxDegradedFrames));

    if (degraded_frames_ > kMaxDegradedFrames) {
        plane_state_ = GroundPlaneState();
        plane_state_.status = GroundPlaneStatus::Unavailable;
        last_candidate_valid_ = false;
        stable_candidate_frames_ = 0;
        degraded_frames_ = 0;
    }
}

std::string GroundPlaneTracker::getStatusString() const
{
    switch (plane_state_.status) {
    case GroundPlaneStatus::Tracking: return "Tracking";
    case GroundPlaneStatus::Locked: return "Locked";
    case GroundPlaneStatus::Degraded: return "Degraded";
    case GroundPlaneStatus::Unavailable:
    default:
        return "Unavailable";
    }
}

bool GroundPlaneTracker::projectScreenPointToPlane(double px, double py, Eigen::Vector3f& world_pt) const
{
    if (!plane_state_.locked || !last_pose_valid_) return false;

    Eigen::Vector3f ray_cam(
        (static_cast<float>(px) - cx_) / fx_,
        (static_cast<float>(py) - cy_) / fy_,
        1.0f);
    ray_cam.normalize();

    const Sophus::SE3f T_wc = last_pose_cw_.inverse();
    const Eigen::Vector3f cam_pos = T_wc.translation();
    const Eigen::Vector3f ray_world = T_wc.rotationMatrix() * ray_cam;

    const float denom = plane_state_.normal.dot(ray_world);
    if (std::abs(denom) < 1e-5f) return false;

    const float t = plane_state_.normal.dot(plane_state_.center - cam_pos) / denom;
    if (t <= 0.0f) return false;

    world_pt = cam_pos + t * ray_world;

    Eigen::Vector2f uv;
    if (!worldToPlaneUV(world_pt, uv)) return false;
    if (std::abs(uv.x()) > plane_state_.extent_u * 1.25f ||
        std::abs(uv.y()) > plane_state_.extent_v * 1.25f) {
        return false;
    }

    return true;
}

bool GroundPlaneTracker::worldToPlaneUV(const Eigen::Vector3f& world_pt, Eigen::Vector2f& uv) const
{
    if (!plane_state_.valid) return false;
    const Eigen::Vector3f delta = world_pt - plane_state_.center;
    uv.x() = delta.dot(plane_state_.axis_u);
    uv.y() = delta.dot(plane_state_.axis_v);
    return true;
}

Eigen::Vector3f GroundPlaneTracker::planeUVToWorld(const Eigen::Vector2f& uv, float height_offset) const
{
    return plane_state_.center +
           uv.x() * plane_state_.axis_u +
           uv.y() * plane_state_.axis_v +
           height_offset * plane_state_.normal;
}

bool GroundPlaneTracker::estimateDominantPlane(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                                               const Sophus::SE3f& T_cw,
                                               GroundPlaneState& candidate) const
{
    std::vector<Eigen::Vector3f> points_world;
    points_world.reserve(map_points.size());

    const Eigen::Matrix3f R_cw = T_cw.rotationMatrix();
    const Eigen::Vector3f t_cw = T_cw.translation();
    const Eigen::Vector3f cam_pos = T_cw.inverse().translation();

    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;
        const Eigen::Vector3f p_world = mp->GetWorldPos();
        const Eigen::Vector3f p_cam = R_cw * p_world + t_cw;
        if (p_cam.z() <= 0.15f) continue;

        const float u = fx_ * p_cam.x() / p_cam.z() + cx_;
        const float v = fy_ * p_cam.y() / p_cam.z() + cy_;
        if (u < 0.0f || u >= img_w_ || v < 0.0f || v >= img_h_) continue;

        points_world.push_back(p_world);
    }

    if (static_cast<int>(points_world.size()) < kMinCandidatePoints) return false;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, static_cast<int>(points_world.size()) - 1);

    int best_count = 0;
    float best_residual = std::numeric_limits<float>::max();
    Eigen::Vector3f best_n = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    float best_d = 0.0f;

    for (int iter = 0; iter < kRansacIters; ++iter) {
        int i0 = dist(rng), i1 = dist(rng), i2 = dist(rng);
        if (i0 == i1 || i0 == i2 || i1 == i2) continue;

        const Eigen::Vector3f& p0 = points_world[i0];
        const Eigen::Vector3f& p1 = points_world[i1];
        const Eigen::Vector3f& p2 = points_world[i2];

        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);
        const float n_norm = n.norm();
        if (n_norm < 1e-6f) continue;
        n /= n_norm;

        const float d = n.dot(p0);
        int inliers = 0;
        float residual_sum = 0.0f;

        for (const auto& p : points_world) {
            const float residual = std::abs(n.dot(p) - d);
            if (residual < kInlierThresholdMeters) {
                inliers++;
                residual_sum += residual;
            }
        }

        if (inliers < kMinInliers) continue;
        const float mean_residual = residual_sum / static_cast<float>(inliers);

        if (inliers > best_count || (inliers == best_count && mean_residual < best_residual)) {
            best_count = inliers;
            best_residual = mean_residual;
            best_n = n;
            best_d = d;
        }
    }

    if (best_count < kMinInliers) return false;
    if (!refinePlane(points_world, best_n, best_d, candidate)) return false;

    Eigen::Vector3f to_cam = cam_pos - candidate.center;
    if (candidate.normal.dot(to_cam) < 0.0f) {
        candidate.normal = -candidate.normal;
        candidate.axis_v = -candidate.axis_v;
    }

    const Eigen::Vector3f cam_dir = (candidate.center - cam_pos).normalized();
    if (std::abs(candidate.normal.dot(cam_dir)) < kMinViewDot) return false;
    if (candidate.inlier_count < kMinInliers) return false;
    if (candidate.mean_residual > kMaxMeanResidualMeters) return false;

    return true;
}

bool GroundPlaneTracker::refinePlane(const std::vector<Eigen::Vector3f>& candidates,
                                     const Eigen::Vector3f& seed_normal,
                                     float seed_d,
                                     GroundPlaneState& plane) const
{
    std::vector<Eigen::Vector3f> inliers;
    inliers.reserve(candidates.size());
    for (const auto& p : candidates) {
        if (std::abs(seed_normal.dot(p) - seed_d) < kInlierThresholdMeters) {
            inliers.push_back(p);
        }
    }

    if (static_cast<int>(inliers.size()) < kMinInliers) return false;

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : inliers) centroid += p;
    centroid /= static_cast<float>(inliers.size());

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : inliers) {
        const Eigen::Vector3f q = p - centroid;
        cov += q * q.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) return false;

    Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
    Eigen::Vector3f axis_u = solver.eigenvectors().col(2).normalized();
    Eigen::Vector3f axis_v = normal.cross(axis_u).normalized();
    if (axis_v.norm() < 1e-6f) {
        axis_u = BuildPlaneAxisU(normal);
        axis_v = normal.cross(axis_u).normalized();
    }

    float residual_sum = 0.0f;
    std::vector<float> us;
    std::vector<float> vs;
    us.reserve(inliers.size());
    vs.reserve(inliers.size());

    for (const auto& p : inliers) {
        residual_sum += std::abs(normal.dot(p - centroid));
        const Eigen::Vector3f d = p - centroid;
        us.push_back(d.dot(axis_u));
        vs.push_back(d.dot(axis_v));
    }

    auto percentileAbs = [](std::vector<float>& values) -> float {
        if (values.empty()) return 0.0f;
        for (auto& value : values) value = std::abs(value);
        const size_t idx = std::min(values.size() - 1, (values.size() * 9) / 10);
        std::nth_element(values.begin(), values.begin() + idx, values.end());
        return values[idx];
    };

    plane.center = centroid;
    plane.normal = normal;
    plane.axis_u = axis_u;
    plane.axis_v = axis_v;
    plane.inlier_count = static_cast<int>(inliers.size());
    plane.mean_residual = residual_sum / static_cast<float>(inliers.size());
    plane.extent_u = std::max(0.05f, percentileAbs(us));
    plane.extent_v = std::max(0.05f, percentileAbs(vs));
    plane.stability_score = 0.0f;
    plane.valid = true;
    return true;
}

bool GroundPlaneTracker::isPlaneStableWith(const GroundPlaneState& reference,
                                           const GroundPlaneState& candidate) const
{
    if (!reference.valid || !candidate.valid) return false;

    const float angle = ClampedAcos(std::abs(reference.normal.dot(candidate.normal)));
    const float center_dist = (reference.center - candidate.center).norm();
    const float ratio_u = std::abs(reference.extent_u - candidate.extent_u) / std::max(reference.extent_u, 0.05f);
    const float ratio_v = std::abs(reference.extent_v - candidate.extent_v) / std::max(reference.extent_v, 0.05f);

    return angle < kMaxStableAngleRad &&
           center_dist < kMaxStableCenterDist &&
           ratio_u < kMaxStableExtentRatioDelta &&
           ratio_v < kMaxStableExtentRatioDelta;
}

void GroundPlaneTracker::smoothUpdateReferencePlane(const GroundPlaneState& candidate)
{
    if (!plane_state_.valid) {
        plane_state_ = candidate;
        return;
    }

    Eigen::Vector3f candidate_normal = candidate.normal;
    if (plane_state_.normal.dot(candidate_normal) < 0.0f) {
        candidate_normal = -candidate_normal;
    }

    Eigen::Vector3f blended_normal =
        (1.0f - kReferenceNormalLerp) * plane_state_.normal +
        kReferenceNormalLerp * candidate_normal;
    if (blended_normal.norm() < 1e-6f) {
        blended_normal = candidate_normal;
    } else {
        blended_normal.normalize();
    }

    Eigen::Vector3f blended_center =
        (1.0f - kReferenceCenterLerp) * plane_state_.center +
        kReferenceCenterLerp * candidate.center;

    Eigen::Vector3f axis_u = candidate.axis_u;
    axis_u = axis_u - axis_u.dot(blended_normal) * blended_normal;
    if (axis_u.norm() < 1e-6f) {
        axis_u = BuildPlaneAxisU(blended_normal);
    } else {
        axis_u.normalize();
    }
    Eigen::Vector3f axis_v = blended_normal.cross(axis_u).normalized();

    plane_state_.normal = blended_normal;
    plane_state_.center = blended_center;
    plane_state_.axis_u = axis_u;
    plane_state_.axis_v = axis_v;
    plane_state_.extent_u =
        (1.0f - kReferenceExtentLerp) * plane_state_.extent_u +
        kReferenceExtentLerp * candidate.extent_u;
    plane_state_.extent_v =
        (1.0f - kReferenceExtentLerp) * plane_state_.extent_v +
        kReferenceExtentLerp * candidate.extent_v;
}

void GroundPlaneTracker::updateStatusNoCandidate()
{
    if (plane_state_.locked) {
        degraded_frames_++;
        plane_state_.status = GroundPlaneStatus::Degraded;
        plane_state_.stability_score =
            std::max(0.0f, 1.0f - static_cast<float>(degraded_frames_) / static_cast<float>(kMaxDegradedFrames));
        if (degraded_frames_ > kMaxDegradedFrames) {
            plane_state_ = GroundPlaneState();
            plane_state_.status = GroundPlaneStatus::Unavailable;
            stable_candidate_frames_ = 0;
            degraded_frames_ = 0;
            last_candidate_valid_ = false;
        }
        return;
    }

    plane_state_ = GroundPlaneState();
    plane_state_.status = GroundPlaneStatus::Unavailable;
    stable_candidate_frames_ = 0;
    degraded_frames_ = 0;
    last_candidate_valid_ = false;
}

} // namespace SPGS

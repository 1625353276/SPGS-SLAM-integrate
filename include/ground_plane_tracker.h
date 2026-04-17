#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <string>
#include <vector>

namespace ORB_SLAM3 {
class MapPoint;
}

namespace SPGS {

enum class GroundPlaneStatus
{
    Unavailable = 0,
    Tracking,
    Locked,
    Degraded
};

struct GroundPlaneState
{
    bool valid = false;
    bool locked = false;
    GroundPlaneStatus status = GroundPlaneStatus::Unavailable;

    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis_u = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f axis_v = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

    float extent_u = 0.0f;
    float extent_v = 0.0f;
    int inlier_count = 0;
    float mean_residual = 0.0f;
    float stability_score = 0.0f;
};

class GroundPlaneTracker
{
public:
    GroundPlaneTracker(int image_w, int image_h, float fx, float fy, float cx, float cy);

    void update(const Sophus::SE3f& T_cw,
                const std::vector<ORB_SLAM3::MapPoint*>& map_points);

    const GroundPlaneState& getState() const { return plane_state_; }
    bool hasLockedPlane() const { return plane_state_.locked; }
    std::string getStatusString() const;

    bool projectScreenPointToPlane(double px, double py, Eigen::Vector3f& world_pt) const;
    bool worldToPlaneUV(const Eigen::Vector3f& world_pt, Eigen::Vector2f& uv) const;
    Eigen::Vector3f planeUVToWorld(const Eigen::Vector2f& uv, float height_offset = 0.0f) const;

private:
    bool estimateDominantPlane(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                               const Sophus::SE3f& T_cw,
                               GroundPlaneState& candidate) const;
    bool refinePlane(const std::vector<Eigen::Vector3f>& candidates,
                     const Eigen::Vector3f& seed_normal,
                     float seed_d,
                     GroundPlaneState& plane) const;
    bool isPlaneStableWith(const GroundPlaneState& reference,
                           const GroundPlaneState& candidate) const;
    void smoothUpdateReferencePlane(const GroundPlaneState& candidate);
    void updateStatusNoCandidate();

private:
    int img_w_;
    int img_h_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;

    Sophus::SE3f last_pose_cw_;
    bool last_pose_valid_ = false;

    GroundPlaneState plane_state_;
    GroundPlaneState last_candidate_;
    bool last_candidate_valid_ = false;

    int stable_candidate_frames_ = 0;
    int degraded_frames_ = 0;
};

} // namespace SPGS

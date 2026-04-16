/**
 * Plane structure for multi-plane AR system
 */

#pragma once

#include <Eigen/Core>
#include <set>
#include <string>

namespace ORB_SLAM3 {
class MapPoint;
}

namespace SPGS {

enum class PlaneType
{
    Floor,      // Horizontal, normal ≈ (0, 1, 0), near ground level
    Table,      // Horizontal, normal ≈ (0, 1, 0), above floor
    Wall,       // Vertical, normal ≈ (±1, 0, 0) or (0, 0, ±1)
    Slope,      // Tilted surface
    Ceiling,    // Horizontal, normal ≈ (0, -1, 0)
    Unknown
};

enum class PlaneStatus
{
    Tentative,  // Just detected, not stable
    Tracking,   // Being tracked
    Locked,     // Stable and locked
    Lost        // No longer visible
};

struct Plane
{
    int id = -1;
    PlaneType type = PlaneType::Unknown;
    PlaneStatus status = PlaneStatus::Tentative;

    // Plane geometry
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis_u = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // First tangent
    Eigen::Vector3f axis_v = Eigen::Vector3f(0.0f, 0.0f, 1.0f);  // Second tangent
    float extent_u = 0.0f;  // Half-extent along axis_u
    float extent_v = 0.0f;  // Half-extent along axis_v

    // Plane equation: normal.dot(point) = d
    float d = 0.0f;

    // Associated map points
    std::set<ORB_SLAM3::MapPoint*> map_points;

    // Connectivity
    std::set<int> neighbor_planes;

    // Tracking state
    float stability_score = 0.0f;
    int inlier_count = 0;
    int frames_since_update = 0;
    int stable_frames = 0;

    // Helper functions
    bool valid() const { return id >= 0 && inlier_count > 0; }

    // Project world point to plane UV coordinates
    Eigen::Vector2f worldToUV(const Eigen::Vector3f& world_pt) const {
        Eigen::Vector3f delta = world_pt - center;
        return Eigen::Vector2f(delta.dot(axis_u), delta.dot(axis_v));
    }

    // Convert UV coordinates back to world position
    Eigen::Vector3f uvToWorld(const Eigen::Vector2f& uv, float height_offset = 0.0f) const {
        return center + uv.x() * axis_u + uv.y() * axis_v + height_offset * normal;
    }

    // Distance from point to plane
    float distanceToPoint(const Eigen::Vector3f& pt) const {
        return std::abs(normal.dot(pt) - d);
    }

    // Check if UV is within plane extent
    bool isUVInExtent(const Eigen::Vector2f& uv, float margin = 1.0f) const {
        return std::abs(uv.x()) <= extent_u * margin &&
               std::abs(uv.y()) <= extent_v * margin;
    }

    // Get type as string
    std::string typeString() const {
        switch (type) {
            case PlaneType::Floor:   return "Floor";
            case PlaneType::Table:   return "Table";
            case PlaneType::Wall:    return "Wall";
            case PlaneType::Slope:   return "Slope";
            case PlaneType::Ceiling: return "Ceiling";
            default: return "Unknown";
        }
    }

    // Get status as string
    std::string statusString() const {
        switch (status) {
            case PlaneStatus::Tentative: return "Tentative";
            case PlaneStatus::Tracking:  return "Tracking";
            case PlaneStatus::Locked:    return "Locked";
            case PlaneStatus::Lost:      return "Lost";
            default: return "Unknown";
        }
    }
};

} // namespace SPGS

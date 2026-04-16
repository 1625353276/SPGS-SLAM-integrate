/**
 * Multi-Plane Detector for AR System
 *
 * Detects and tracks multiple planes from SLAM MapPoints.
 * Supports floor, table, wall, and slope detection.
 */

#pragma once

#include "include/plane.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector>
#include <memory>

namespace ORB_SLAM3 {
class MapPoint;
}

namespace SPGS {

struct MultiPlaneConfig
{
    // Detection parameters
    int min_points_per_plane = 30;
    int max_planes = 10;
    int ransac_iterations = 200;
    float inlier_threshold_meters = 0.03f;

    // Classification thresholds
    float floor_max_height = 0.3f;         // Max center height for floor
    float table_min_height = 0.4f;         // Min center height for table
    float wall_normal_y_threshold = 0.3f;  // |normal.y| < this = wall

    // Stability
    int lock_stable_frames = 5;
    int max_lost_frames = 10;

    // Plane merging
    float merge_max_angle_deg = 10.0f;
    float merge_max_distance_meters = 0.1f;
};

class MultiPlaneDetector
{
public:
    explicit MultiPlaneDetector(int image_w, int image_h,
                                 float fx, float fy, float cx, float cy,
                                 const MultiPlaneConfig& config = MultiPlaneConfig());

    /**
     * Update plane detection with current MapPoints and camera pose
     * @param map_points All tracked map points from SLAM
     * @param T_cw Camera pose (world to camera)
     */
    void update(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                const Sophus::SE3f& T_cw);

    /**
     * Get all detected planes
     */
    const std::vector<Plane>& getPlanes() const { return planes_; }

    /**
     * Get mutable planes (for visualization)
     */
    std::vector<Plane>& getPlanesMutable() { return planes_; }

    /**
     * Find the plane that a world point is closest to
     * @param world_pos Position in world coordinates
     * @param max_distance Maximum distance to consider
     * @return Pointer to plane, or nullptr if none found
     */
    const Plane* findPlaneAtPosition(const Eigen::Vector3f& world_pos,
                                      float max_distance = 0.1f) const;

    /**
     * Find plane by ID
     */
    const Plane* findPlaneById(int id) const;

    /**
     * Get plane at screen position (raycast)
     * @param px Screen x coordinate
     * @param py Screen y coordinate
     * @param intersection_world Output intersection point
     * @return Pointer to intersected plane, or nullptr
     */
    const Plane* raycastToPlane(double px, double py,
                                 Eigen::Vector3f& intersection_world) const;

    /**
     * Get debug/status info
     */
    int getTotalMapPoints() const { return total_map_points_; }
    int getAssignedMapPoints() const { return assigned_map_points_; }
    std::string getStatusString() const;

private:
    // Configuration
    MultiPlaneConfig config_;
    int img_w_, img_h_;
    float fx_, fy_, cx_, cy_;

    // Camera pose for raycasting
    Sophus::SE3f last_T_cw_;
    bool pose_valid_ = false;

    // Detected planes
    std::vector<Plane> planes_;
    int next_plane_id_ = 1;

    // Statistics
    int total_map_points_ = 0;
    int assigned_map_points_ = 0;

    // --- Internal methods ---

    /**
     * Extract visible points from MapPoints
     */
    std::vector<Eigen::Vector3f> extractVisiblePoints(
        const std::vector<ORB_SLAM3::MapPoint*>& map_points,
        const Sophus::SE3f& T_cw) const;

    /**
     * Try to fit a plane to a set of points using RANSAC
     * @return true if a valid plane was found
     */
    bool fitPlaneRANSAC(const std::vector<Eigen::Vector3f>& points,
                        Plane& plane_out,
                        std::vector<int>& inlier_indices_out) const;

    /**
     * Refine plane using all inliers (PCA)
     */
    bool refinePlane(const std::vector<Eigen::Vector3f>& points,
                     const std::vector<int>& inlier_indices,
                     Plane& plane_out) const;

    /**
     * Classify plane type based on normal and position
     */
    PlaneType classifyPlane(const Eigen::Vector3f& normal,
                            const Eigen::Vector3f& center) const;

    /**
     * Update plane status based on tracking
     */
    void updatePlaneStatus(Plane& plane, int current_inlier_count);

    /**
     * Try to merge similar planes
     */
    void mergeSimilarPlanes();

    /**
     * Remove lost planes
     */
    void removeLostPlanes();

    /**
     * Build plane axes from normal
     */
    static void buildPlaneAxes(Plane& plane);
};

} // namespace SPGS

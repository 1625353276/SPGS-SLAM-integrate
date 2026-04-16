# Multi-Plane AR System Design

## Overview

This document describes the architecture for a multi-plane detection and navigation system that allows virtual objects to move across different surfaces (floors, tables, walls, slopes) with automatic orientation adaptation.

## Current System Limitations

1. **Single Plane Only**: `GroundPlaneTracker` tracks only one dominant plane
2. **No Plane Persistence**: Once locked, the plane is only updated, not expanded
3. **No Obstacle Awareness**: Points above the plane are simply ignored
4. **No Cross-Plane Navigation**: Objects cannot move between different surfaces

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        AR Viewer                                 │
│  (Modified to use MultiPlaneManager instead of GroundPlaneTracker)│
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    MultiPlaneManager                             │
│  - Manages all detected planes                                   │
│  - Handles plane switching for objects                           │
│  - Provides cross-plane navigation                               │
└─────────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│  PlaneDetector  │ │  PlaneGraph     │ │  CrossPlaneNav  │
│  - RANSAC       │ │  - Connectivity │ │  - Path planning│
│  - Clustering   │ │  - Transitions  │ │  - Transitions  │
└─────────────────┘ └─────────────────┘ └─────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Plane (struct)                              │
│  - id, normal, center, extent                                    │
│  - axis_u, axis_v (local 2D coordinate system)                   │
│  - map_points (points belonging to this plane)                   │
│  - type (Floor, Table, Wall, Slope, Unknown)                     │
│  - status (Tracking, Locked, Lost)                               │
└─────────────────────────────────────────────────────────────────┘
```

## Component Design

### 1. Plane Structure

```cpp
enum class PlaneType {
    Floor,      // Horizontal, normal ≈ (0, 1, 0)
    Table,      // Horizontal, above floor level
    Wall,       // Vertical, normal ≈ (±1, 0, 0) or (0, 0, ±1)
    Slope,      // Tilted surface
    Ceiling,    // Horizontal, normal ≈ (0, -1, 0)
    Unknown
};

enum class PlaneStatus {
    Tentative,  // Just detected, not stable
    Tracking,   // Being tracked
    Locked,     // Stable and locked
    Lost        // No longer visible
};

struct Plane {
    int id;
    PlaneType type;
    PlaneStatus status;

    Eigen::Vector3f normal;
    Eigen::Vector3f center;
    Eigen::Vector3f axis_u, axis_v;
    float extent_u, extent_v;

    std::set<ORB_SLAM3::MapPoint*> map_points;
    std::set<int> neighbor_planes;  // Connected plane IDs

    float stability_score;
    int frames_since_update;
};
```

### 2. MultiPlaneDetector

**Purpose**: Detect and maintain multiple planes from MapPoints

**Algorithm**:
1. **Initial Detection**:
   - Use RANSAC to find the dominant plane
   - Remove inliers and repeat for additional planes
   - Continue until not enough points remain

2. **Plane Classification**:
   - Floor: normal.y > 0.8, center near ground level
   - Table: normal.y > 0.8, center above floor + threshold
   - Wall: |normal.y| < 0.3
   - Slope: otherwise

3. **Incremental Update**:
   - New MapPoints are assigned to nearest compatible plane
   - Outliers may form new tentative planes
   - Planes are merged if they become connected

```cpp
class MultiPlaneDetector {
public:
    void update(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                const Sophus::SE3f& T_cw);

    const std::vector<Plane>& getPlanes() const;
    Plane* findPlaneAtPosition(const Eigen::Vector3f& world_pos);

private:
    std::vector<Plane> planes_;
    int next_plane_id_ = 1;

    bool tryFitPlane(const std::vector<Eigen::Vector3f>& points, Plane& plane);
    PlaneType classifyPlane(const Eigen::Vector3f& normal, const Eigen::Vector3f& center);
    void assignPointToPlane(ORB_SLAM3::MapPoint* mp, Plane& plane);
    void mergeOverlappingPlanes();
};
```

### 3. PlaneGraph

**Purpose**: Manage connectivity between planes for cross-plane navigation

```cpp
struct PlaneTransition {
    int from_plane_id;
    int to_plane_id;
    Eigen::Vector3f transition_point;  // Where planes meet
    float transition_cost;             // Cost to cross (height diff, angle, etc.)
    TransitionType type;               // Step, Slope, Wall
};

class PlaneGraph {
public:
    void addPlane(int plane_id);
    void removePlane(int plane_id);
    void addTransition(const PlaneTransition& transition);
    void updateConnectivity();

    std::vector<int> findPathBetweenPlanes(int from_id, int to_id);

private:
    std::map<int, std::vector<PlaneTransition>> adjacency_;
};
```

### 4. VirtualObject Enhancement

**Purpose**: Track which plane an object is on and handle transitions

```cpp
struct VirtualObject {
    // ... existing fields ...

    // Multi-plane support
    int current_plane_id = -1;
    Eigen::Vector2f plane_uv;           // Position on current plane
    float height_offset;                // Above/below plane surface
    float yaw_on_plane;                 // Orientation relative to plane

    bool transitioning = false;         // Currently moving between planes
    int target_plane_id = -1;
};
```

### 5. Cross-Plane Navigation

**Purpose**: Plan paths that may cross multiple planes

```cpp
struct CrossPlanePath {
    struct Segment {
        int plane_id;
        std::vector<Eigen::Vector2f> waypoints_uv;
        Eigen::Vector3f entry_point;    // World position to enter this plane
        Eigen::Vector3f exit_point;     // World position to exit this plane
    };

    std::vector<Segment> segments;
    float total_cost;
};

class CrossPlaneNavigator {
public:
    CrossPlanePath planPath(
        const VirtualObject& object,
        const Eigen::Vector3f& target_world,
        const MultiPlaneDetector& detector,
        const PlaneGraph& graph);

private:
    PathResult planOnSinglePlane(
        const Plane& plane,
        const Eigen::Vector2f& start_uv,
        const Eigen::Vector2f& goal_uv);
};
```

## Object Orientation System

### Principle
Object's Y-axis (up) should always align with the plane's normal.

### Implementation

```cpp
Sophus::SE3f computeObjectPoseOnPlane(
    const Plane& plane,
    const Eigen::Vector2f& uv,
    float height_offset,
    float yaw_relative)
{
    // Position on plane
    Eigen::Vector3f position =
        plane.center +
        uv.x() * plane.axis_u +
        uv.y() * plane.axis_v +
        height_offset * plane.normal;

    // Up direction = plane normal
    Eigen::Vector3f up = plane.normal.normalized();

    // Forward direction = rotated by yaw around normal
    Eigen::Vector3f forward =
        std::sin(yaw_relative) * plane.axis_u +
        std::cos(yaw_relative) * plane.axis_v;
    forward = forward - forward.dot(up) * up;
    forward.normalize();

    // Right = up × forward
    Eigen::Vector3f right = up.cross(forward).normalized();

    // Build rotation matrix
    Eigen::Matrix3f R;
    R.col(0) = right;
    R.col(1) = up;
    R.col(2) = forward;

    return Sophus::SE3f(Eigen::Quaternionf(R), position);
}
```

## User Interaction Flow

### Placing Object
1. User clicks on screen
2. Raycast from camera through click point
3. Find intersected plane (may be floor, table, wall, etc.)
4. Place object on that plane with correct orientation
5. Store plane_id and plane_uv in VirtualObject

### Moving Object
1. User clicks destination
2. Find target plane
3. Plan cross-plane path (if on different planes)
4. Animate object along path
5. Update orientation when crossing plane boundaries

### Automatic Behavior
1. When object is on a locked plane, it follows the plane's updates
2. If plane becomes Lost, object may transfer to nearby plane
3. If no valid plane, object becomes "floating"

## File Structure

```
include/
├── multi_plane_detector.h
├── plane_graph.h
├── cross_plane_navigator.h
└── plane.h

src/
├── multi_plane_detector.cpp
├── plane_graph.cpp
├── cross_plane_navigator.cpp
└── ar_viewer.cpp  (modified)
```

## Implementation Phases

### Phase 1: Basic Multi-Plane Detection
- Implement `Plane` structure
- Implement `MultiPlaneDetector` with basic RANSAC
- Classify planes by type
- Visualize multiple planes in viewer

### Phase 2: Plane Assignment for Objects
- Modify `VirtualObject` to track plane_id
- Implement orientation adaptation
- Update `moveObjectToNearestMapPoint` to find appropriate plane

### Phase 3: Single-Plane Navigation
- Update navigation to work on arbitrary plane (not just ground)
- Plane-local 2D coordinate system
- A* pathfinding on the plane

### Phase 4: Cross-Plane Navigation
- Implement `PlaneGraph` for connectivity
- Detect plane transitions/overlaps
- Plan paths across multiple planes
- Smooth orientation transitions

### Phase 5: Refinement
- Plane merging and splitting
- Handling dynamic scenes
- Performance optimization
- UI controls for plane visualization

## Configuration Parameters

```yaml
# multi_plane.yaml
MultiPlane:
  min_points_per_plane: 30
  max_planes: 10
  ransac_iterations: 200
  inlier_threshold_meters: 0.03

  plane_merge:
    max_angle_deg: 10.0
    max_distance_meters: 0.1
    min_overlap_ratio: 0.3

  classification:
    floor_max_height: 0.3
    table_min_height: 0.4
    wall_normal_y_threshold: 0.3

  navigation:
    transition_cost_per_meter: 1.0
    transition_cost_height_diff: 2.0
    transition_cost_angle_diff: 1.5
```

## Visualization

In the AR viewer, add ImGui controls:
- Show/hide individual planes (checkboxes)
- Color code planes by type
- Draw plane boundaries and normals
- Show current plane for selected object
- Debug: show plane connectivity graph

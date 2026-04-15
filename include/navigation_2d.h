#pragma once

#include <Eigen/Core>

#include <vector>

namespace SPGS {

struct NavGrid
{
    float resolution = 0.05f;
    int width = 0;
    int height = 0;
    Eigen::Vector2f origin_uv = Eigen::Vector2f::Zero();
    std::vector<unsigned char> cells; // 0=blocked, 1=walkable

    bool valid() const { return width > 0 && height > 0 && cells.size() == static_cast<size_t>(width * height); }
    bool inBounds(int x, int y) const { return x >= 0 && x < width && y >= 0 && y < height; }
    int index(int x, int y) const { return y * width + x; }
    bool isWalkable(int x, int y) const { return inBounds(x, y) && cells[index(x, y)] != 0; }
};

struct PathResult
{
    bool success = false;
    std::vector<Eigen::Vector2f> waypoints_uv;
};

struct NavGridBuildParams
{
    float resolution = 0.05f;
    int padding_cells = 8;
    int support_threshold = 1;
    int hole_fill_neighbors = 5;
};

NavGrid BuildWalkableGridFromPoints(const std::vector<Eigen::Vector2f>& points_uv,
                                    const Eigen::Vector2f& start_uv,
                                    const Eigen::Vector2f& goal_uv,
                                    const NavGridBuildParams& params = NavGridBuildParams());

PathResult PlanPathAStar(const NavGrid& grid,
                         const Eigen::Vector2f& start_uv,
                         const Eigen::Vector2f& goal_uv);

bool UVToCell(const NavGrid& grid, const Eigen::Vector2f& uv, int& x, int& y);
Eigen::Vector2f CellToUV(const NavGrid& grid, int x, int y);

} // namespace SPGS

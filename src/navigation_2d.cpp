#include "include/navigation_2d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace SPGS {

namespace {

struct Node
{
    int idx = -1;
    float f = std::numeric_limits<float>::infinity();
};

struct NodeCompare
{
    bool operator()(const Node& a, const Node& b) const
    {
        return a.f > b.f;
    }
};

bool HasLineOfSight(const NavGrid& grid, int x0, int y0, int x1, int y1)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;
    while (true) {
        if (!grid.isWalkable(x, y)) return false;
        if (x == x1 && y == y1) return true;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

std::vector<int> CompressCollinearCells(const NavGrid& grid, const std::vector<int>& path_cells)
{
    if (path_cells.size() <= 2) return path_cells;

    std::vector<int> compressed;
    compressed.reserve(path_cells.size());
    compressed.push_back(path_cells.front());

    int prev_dx = 0;
    int prev_dy = 0;
    for (size_t i = 1; i < path_cells.size(); ++i) {
        const int prev = path_cells[i - 1];
        const int curr = path_cells[i];
        const int dx = (curr % grid.width) - (prev % grid.width);
        const int dy = (curr / grid.width) - (prev / grid.width);

        if (i == 1) {
            prev_dx = dx;
            prev_dy = dy;
            continue;
        }

        if (dx != prev_dx || dy != prev_dy) {
            compressed.push_back(path_cells[i - 1]);
            prev_dx = dx;
            prev_dy = dy;
        }
    }

    compressed.push_back(path_cells.back());
    return compressed;
}

std::vector<int> ShortcutCells(const NavGrid& grid, const std::vector<int>& path_cells)
{
    if (path_cells.size() <= 2) return path_cells;

    std::vector<int> simplified;
    simplified.reserve(path_cells.size());

    size_t anchor = 0;
    simplified.push_back(path_cells[anchor]);
    while (anchor < path_cells.size() - 1) {
        size_t best = anchor + 1;
        const int ax = path_cells[anchor] % grid.width;
        const int ay = path_cells[anchor] / grid.width;

        for (size_t candidate = anchor + 2; candidate < path_cells.size(); ++candidate) {
            const int cx = path_cells[candidate] % grid.width;
            const int cy = path_cells[candidate] / grid.width;
            if (!HasLineOfSight(grid, ax, ay, cx, cy)) break;
            best = candidate;
        }

        simplified.push_back(path_cells[best]);
        anchor = best;
    }

    return simplified;
}

int CountWalkableNeighbors(const NavGrid& grid, int x, int y)
{
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (grid.isWalkable(x + dx, y + dy)) {
                count++;
            }
        }
    }
    return count;
}

void FillSmallHoles(NavGrid& grid, int required_neighbors)
{
    std::vector<unsigned char> updated = grid.cells;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (grid.isWalkable(x, y)) continue;
            if (CountWalkableNeighbors(grid, x, y) >= required_neighbors) {
                updated[grid.index(x, y)] = 1;
            }
        }
    }
    grid.cells.swap(updated);
}

void KeepStartConnectedComponent(NavGrid& grid, int sx, int sy)
{
    if (!grid.isWalkable(sx, sy)) return;

    std::vector<unsigned char> kept(static_cast<size_t>(grid.width * grid.height), 0);
    std::queue<int> q;
    const int start_idx = grid.index(sx, sy);
    kept[start_idx] = 1;
    q.push(start_idx);

    while (!q.empty()) {
        const int idx = q.front();
        q.pop();
        const int x = idx % grid.width;
        const int y = idx / grid.width;

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const int nx = x + dx;
                const int ny = y + dy;
                if (!grid.isWalkable(nx, ny)) continue;
                const int nidx = grid.index(nx, ny);
                if (kept[nidx]) continue;
                kept[nidx] = 1;
                q.push(nidx);
            }
        }
    }

    grid.cells.swap(kept);
}

} // namespace

bool UVToCell(const NavGrid& grid, const Eigen::Vector2f& uv, int& x, int& y)
{
    if (!grid.valid()) return false;
    x = static_cast<int>(std::floor((uv.x() - grid.origin_uv.x()) / grid.resolution));
    y = static_cast<int>(std::floor((uv.y() - grid.origin_uv.y()) / grid.resolution));
    return grid.inBounds(x, y);
}

Eigen::Vector2f CellToUV(const NavGrid& grid, int x, int y)
{
    return Eigen::Vector2f(
        grid.origin_uv.x() + (static_cast<float>(x) + 0.5f) * grid.resolution,
        grid.origin_uv.y() + (static_cast<float>(y) + 0.5f) * grid.resolution);
}

NavGrid BuildWalkableGridFromPoints(const std::vector<Eigen::Vector2f>& points_uv,
                                    const Eigen::Vector2f& start_uv,
                                    const Eigen::Vector2f& goal_uv,
                                    const NavGridBuildParams& params)
{
    NavGrid grid;
    const float resolution = params.resolution;
    const int padding_cells = params.padding_cells;
    const int support_threshold = std::max(1, params.support_threshold);
    const int hole_fill_neighbors = std::clamp(params.hole_fill_neighbors, 0, 8);

    grid.resolution = resolution;

    if (points_uv.empty()) return grid;

    Eigen::Vector2f min_uv = start_uv.cwiseMin(goal_uv);
    Eigen::Vector2f max_uv = start_uv.cwiseMax(goal_uv);
    for (const auto& uv : points_uv) {
        min_uv = min_uv.cwiseMin(uv);
        max_uv = max_uv.cwiseMax(uv);
    }

    const Eigen::Vector2f padding = Eigen::Vector2f::Constant(resolution * static_cast<float>(padding_cells));
    min_uv -= padding;
    max_uv += padding;

    grid.origin_uv = min_uv;
    grid.width = std::max(1, static_cast<int>(std::ceil((max_uv.x() - min_uv.x()) / resolution)));
    grid.height = std::max(1, static_cast<int>(std::ceil((max_uv.y() - min_uv.y()) / resolution)));
    grid.cells.assign(static_cast<size_t>(grid.width * grid.height), 0);

    std::vector<int> support_counts(static_cast<size_t>(grid.width * grid.height), 0);

    for (const auto& uv : points_uv) {
        int x = 0, y = 0;
        if (!UVToCell(grid, uv, x, y)) continue;
        support_counts[grid.index(x, y)]++;
    }

    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const int idx = grid.index(x, y);
            if (support_counts[idx] >= support_threshold) {
                grid.cells[idx] = 1;
            }
        }
    }

    int sx = 0, sy = 0, gx = 0, gy = 0;
    if (UVToCell(grid, start_uv, sx, sy)) grid.cells[grid.index(sx, sy)] = 1;
    if (UVToCell(grid, goal_uv, gx, gy)) grid.cells[grid.index(gx, gy)] = 1;

    FillSmallHoles(grid, hole_fill_neighbors);
    if (UVToCell(grid, start_uv, sx, sy)) grid.cells[grid.index(sx, sy)] = 1;
    if (UVToCell(grid, goal_uv, gx, gy)) grid.cells[grid.index(gx, gy)] = 1;

    if (UVToCell(grid, start_uv, sx, sy)) {
        KeepStartConnectedComponent(grid, sx, sy);
    }
    if (UVToCell(grid, start_uv, sx, sy)) grid.cells[grid.index(sx, sy)] = 1;
    if (UVToCell(grid, goal_uv, gx, gy)) grid.cells[grid.index(gx, gy)] = 1;

    return grid;
}

PathResult PlanPathAStar(const NavGrid& grid,
                         const Eigen::Vector2f& start_uv,
                         const Eigen::Vector2f& goal_uv)
{
    PathResult result;
    if (!grid.valid()) return result;

    int sx = 0, sy = 0, gx = 0, gy = 0;
    if (!UVToCell(grid, start_uv, sx, sy)) return result;
    if (!UVToCell(grid, goal_uv, gx, gy)) return result;
    if (!grid.isWalkable(sx, sy) || !grid.isWalkable(gx, gy)) return result;

    const int start_idx = grid.index(sx, sy);
    const int goal_idx = grid.index(gx, gy);
    const int total = grid.width * grid.height;

    std::vector<float> g_score(static_cast<size_t>(total), std::numeric_limits<float>::infinity());
    std::vector<int> parent(static_cast<size_t>(total), -1);
    std::priority_queue<Node, std::vector<Node>, NodeCompare> open;

    auto heuristic = [&](int x, int y) {
        float dx = static_cast<float>(gx - x);
        float dy = static_cast<float>(gy - y);
        return std::sqrt(dx * dx + dy * dy);
    };

    g_score[start_idx] = 0.0f;
    open.push(Node{start_idx, heuristic(sx, sy)});

    const int dxs[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dys[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (current.idx == goal_idx) break;

        int cx = current.idx % grid.width;
        int cy = current.idx / grid.width;

        for (int i = 0; i < 8; ++i) {
            int nx = cx + dxs[i];
            int ny = cy + dys[i];
            if (!grid.isWalkable(nx, ny)) continue;

            const int nidx = grid.index(nx, ny);
            const float step_cost = (dxs[i] == 0 || dys[i] == 0) ? 1.0f : 1.41421356f;
            const float tentative = g_score[current.idx] + step_cost;
            if (tentative >= g_score[nidx]) continue;

            g_score[nidx] = tentative;
            parent[nidx] = current.idx;
            open.push(Node{nidx, tentative + heuristic(nx, ny)});
        }
    }

    if (parent[goal_idx] == -1 && start_idx != goal_idx) return result;

    std::vector<int> path_cells;
    for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
        path_cells.push_back(idx);
        if (idx == start_idx) break;
    }
    if (path_cells.empty() || path_cells.back() != start_idx) return result;

    std::reverse(path_cells.begin(), path_cells.end());
    std::vector<int> simplified_cells = CompressCollinearCells(grid, path_cells);
    simplified_cells = ShortcutCells(grid, simplified_cells);

    result.waypoints_uv.reserve(simplified_cells.size());
    for (int idx : simplified_cells) {
        int x = idx % grid.width;
        int y = idx / grid.width;
        result.waypoints_uv.push_back(CellToUV(grid, x, y));
    }

    if (!result.waypoints_uv.empty()) {
        result.waypoints_uv.front() = start_uv;
        result.waypoints_uv.back() = goal_uv;
    }

    result.success = !result.waypoints_uv.empty();
    return result;
}

} // namespace SPGS

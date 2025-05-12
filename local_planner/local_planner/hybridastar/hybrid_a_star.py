import numpy as np
import math
import heapq

class HybridAStar:
    def __init__(self, grid_size, resolution, num_angles=36, step_size=1, obstacle_threshold=80):
        self.grid_size   = grid_size
        self.resolution  = resolution
        self.num_angles  = num_angles
        self.step_size   = step_size
        self.delta_theta = 2 * math.pi / num_angles
        self.obstacle_threshold = obstacle_threshold

    def plan(self, start, goal, costmap):
        size = costmap.shape[0]
        ox = start[0] - self.grid_size / 2
        oy = start[1] - self.grid_size / 2

        def world_to_idx(x, o):
            return int((x - o) / self.resolution)

        sx = np.clip(world_to_idx(start[0], ox), 0, size - 1)
        sy = np.clip(world_to_idx(start[1], oy), 0, size - 1)
        stheta = int(start[2] / self.delta_theta) % self.num_angles

        gx = np.clip(world_to_idx(goal[0], ox), 0, size - 1)
        gy = np.clip(world_to_idx(goal[1], oy), 0, size - 1)

        open_list = []
        closed = dict()

        start_node = Node(sx, sy, stheta, 0.0, 0.0, None)
        start_node.f = self.heuristic(sx, sy, gx, gy)
        heapq.heappush(open_list, (start_node.f, start_node))

        while open_list:
            _, current = heapq.heappop(open_list)
            key = (current.x, current.y, current.theta)
            if key in closed:
                continue
            closed[key] = current.g

            if (current.x, current.y) == (gx, gy):
                return self._reconstruct_path(current, ox, oy)

            for turn in [-1, 0, 1]:
                new_theta = (current.theta + turn) % self.num_angles
                yaw = new_theta * self.delta_theta

                nx_f = current.x + math.cos(yaw) * self.step_size
                ny_f = current.y + math.sin(yaw) * self.step_size
                nx = int(round(nx_f))
                ny = int(round(ny_f))

                if not (0 <= nx < size and 0 <= ny < size):
                    continue
                if costmap[ny, nx] >= self.obstacle_threshold:
                    continue

                new_g = current.g + self.resolution * self.step_size
                h     = self.heuristic(nx, ny, gx, gy)
                new_f = new_g + h

                nkey = (nx, ny, new_theta)
                if nkey in closed and closed[nkey] <= new_g:
                    continue

                neighbor = Node(nx, ny, new_theta, new_g, new_f, current)
                heapq.heappush(open_list, (neighbor.f, neighbor))

        return []

    def heuristic(self, x, y, gx, gy):
        return math.hypot(gx - x, gy - y) * self.resolution

    def _reconstruct_path(self, node, ox, oy):
        path = []
        while node:
            wx = ox + node.x * self.resolution
            wy = oy + node.y * self.resolution
            wt = node.theta * self.delta_theta
            path.append((wx, wy, wt))
            node = node.parent
        return list(reversed(path))

class Node:
    def __init__(self, x, y, theta, g, f, parent):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.f = f
        self.parent = parent

    def __lt__(self, other):  # for heapq
        return self.f < other.f
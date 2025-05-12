import numpy as np
import math
import heapq

class HybridAStar:
    def __init__(self, grid_size, resolution, num_angles=36, step_size=1, obstacle_threshold=1):
        self.grid_size = grid_size
        self.resolution = resolution
        self.num_angles = num_angles
        self.step_size = step_size
        self.delta_theta = 2 * math.pi / num_angles
        self.obstacle_threshold = obstacle_threshold

    def plan(self, start, goal, costmap):
        size_y, size_x = costmap.shape
        ox = start[0] - self.grid_size / 2
        oy = start[1] - self.grid_size / 2

        def world_to_idx(x, o):
            return int((x - o) / self.resolution)

        sx = np.clip(world_to_idx(start[0], ox), 0, size_x - 1)
        sy = np.clip(world_to_idx(start[1], oy), 0, size_y - 1)
        stheta = int(start[2] / self.delta_theta) % self.num_angles

        gx = np.clip(world_to_idx(goal[0], ox), 0, size_x - 1)
        gy = np.clip(world_to_idx(goal[1], oy), 0, size_y - 1)

        goal_yaw = goal[2]
        goal_theta = int(goal_yaw / self.delta_theta) % self.num_angles

        open_list = []
        closed = dict()

        start_node = Node(sx, sy, stheta, 0.0, 0.0, None)
        start_node.f = self.heuristic(sx, sy, stheta, gx, gy, goal_theta)
        heapq.heappush(open_list, (start_node.f, start_node))

        first_step = True

        while open_list:
            _, current = heapq.heappop(open_list)
            key = (current.x, current.y, current.theta)
            if key in closed:
                continue
            closed[key] = current.g

            if (current.x, current.y) == (gx, gy):
                return self._reconstruct_path(current, ox, oy)

            # 처음 한 스텝은 로봇 yaw 방향 유지
            turns = [0] if first_step else [-1, 0, 1]
            first_step = False

            for turn in turns:
                new_theta = (current.theta + turn) % self.num_angles
                yaw = new_theta * self.delta_theta

                nx_f = current.x + math.cos(yaw) * self.step_size
                ny_f = current.y + math.sin(yaw) * self.step_size
                nx = int(round(nx_f))
                ny = int(round(ny_f))

                if not (0 <= nx < size_x and 0 <= ny < size_y):
                    continue
                if self.is_collision(nx, ny, costmap):
                    continue

                new_g = current.g + self.resolution * self.step_size
                h = self.heuristic(nx, ny, new_theta, gx, gy, goal_theta)
                new_f = new_g + h

                nkey = (nx, ny, new_theta)
                if nkey in closed and closed[nkey] <= new_g:
                    continue

                neighbor = Node(nx, ny, new_theta, new_g, new_f, current)
                heapq.heappush(open_list, (neighbor.f, neighbor))

        return []

    def is_collision(self, x, y, costmap):
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx = x + dx
                ny = y + dy
                if 0 <= nx < costmap.shape[1] and 0 <= ny < costmap.shape[0]:
                    if costmap[ny, nx] >= self.obstacle_threshold:
                        return True
        return False

    def heuristic(self, x, y, theta, gx, gy, goal_theta):
        dist_cost = math.hypot(gx - x, gy - y) * self.resolution
        dtheta = min(abs(theta - goal_theta), self.num_angles - abs(theta - goal_theta))
        yaw_cost = (dtheta / self.num_angles) * 2.0  # 가중치 조절 가능
        return dist_cost + yaw_cost

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

    def __lt__(self, other):
        return self.f < other.f

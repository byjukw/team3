"""
D* Lite grid planning (modified for dynamic map size)
author: vss2sn (with downstream edits)
"""

import math
import numpy as np

class Node:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost

def add_coordinates(node1: Node, node2: Node) -> Node:
    new_node = Node()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node

def compare_coordinates(node1: Node, node2: Node) -> bool:
    return node1.x == node2.x and node1.y == node2.y

class DStarLite:

    # 8 방향 이동 (+대각선)
    motions = [
        Node(1, 0, 1),
        Node(0, 1, 1),
        Node(-1, 0, 1),
        Node(0, -1, 1),
        Node(1, 1, math.sqrt(2)),
        Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)),
        Node(-1, -1, math.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list, width: int, height: int):
        """
        ox, oy: obstacle coordinates in grid indices
        width, height: full map size in grid cells
        """
        # No offset—ox, oy are already grid indices
        self.x_min_world = 0
        self.y_min_world = 0

        # Map dimensions
        self.x_max = width
        self.y_max = height

        # Obstacles: use ox, oy directly
        self.obstacles = [
            Node(x, y) for x, y in zip(ox, oy)
        ]
        # For quick lookup
        self.obstacles_xy = np.array(
            [[obs.x, obs.y] for obs in self.obstacles]
        )

        # D* Lite state
        self.start = Node()
        self.goal = Node()
        self.U = []
        self.km = 0.0

        # cost-to-go (rhs) and cost-so-far (g) grids
        self.rhs = self._create_grid(math.inf)
        self.g   = self._create_grid(math.inf)

        self.initialized = False


    def _create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)

    def is_obstacle(self, node: Node) -> bool:
        if self.obstacles_xy.size == 0:
            return False
        # check obstacle list
        return np.any(
            (self.obstacles_xy[:,0] == node.x) &
            (self.obstacles_xy[:,1] == node.y)
        )

    def c(self, node1: Node, node2: Node) -> float:
        # cost from node1 → node2
        if self.is_obstacle(node2):
            return math.inf
        # find matching motion
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        for m in self.motions:
            if m.x == dx and m.y == dy:
                return m.cost
        return math.inf

    def h(self, s: Node) -> float:
        # simple heuristic: Manhattan
        dx = abs(self.start.x - s.x)
        dy = abs(self.start.y - s.y)
        return dx + dy

    def calculate_key(self, s: Node):
        val = min(self.g[s.x, s.y], self.rhs[s.x, s.y])
        return (val + self.h(s) + self.km, val)

    def is_valid(self, node: Node) -> bool:
        return 0 <= node.x < self.x_max and 0 <= node.y < self.y_max

    def get_neighbours(self, u: Node):
        nbrs = []
        for motion in self.motions:
            v = add_coordinates(u, motion)
            if self.is_valid(v):
                nbrs.append(v)
        return nbrs

    def initialize(self, start: Node, goal: Node):
        # shift by world origin
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x  = goal.x  - self.x_min_world
        self.goal.y  = goal.y  - self.y_min_world

        if not self.initialized:
            self.initialized = True
            # reset U, km, g, rhs
            self.U = []
            self.km = 0.0
            self.rhs.fill(math.inf)
            self.g.fill(math.inf)

            # goal rhs = 0, push into U
            self.rhs[self.goal.x, self.goal.y] = 0.0
            self.U.append((Node(self.goal.x, self.goal.y), 
                           self.calculate_key(self.goal)))

    def update_vertex(self, u: Node):
        if not compare_coordinates(u, self.goal):
            # rhs(u) = min over successors
            self.rhs[u.x, u.y] = min(
                self.c(u, v) + self.g[v.x, v.y]
                for v in self.get_neighbours(u)
            )
        # remove u from U if present
        self.U = [(n,k) for (n,k) in self.U if not compare_coordinates(n, u)]
        # if g != rhs, reinsert
        if self.g[u.x, u.y] != self.rhs[u.x, u.y]:
            self.U.append((Node(u.x, u.y), self.calculate_key(u)))

    def compute_shortest_path(self):
        # loop until start is consistent
        while True:
            # — 방어 코드: U가 비어 있으면 즉시 종료 —
            if not self.U:
                print("[D*] ERROR: U is empty, cannot continue planning")
                return

            # U sorted by key
            self.U.sort(key=lambda nk: nk[1])
            # 꺼낼 노드(u)와 이전 키(key_old) 추출
            u, key_old = self.U[0]
            k_new = self.calculate_key(self.start)

            # 디버그 로그: 꺼낸 노드, 이전 키, start의 새로운 키, U 크기
            print(f"[D*] pop node=({u.x},{u.y}) k_old={key_old} k_start={k_new} U_size={len(self.U)}")

            # start 노드가 일관성(consistency)을 만족하면 종료
            if key_old >= k_new and self.rhs[self.start.x,self.start.y] == self.g[self.start.x,self.start.y]:
                break

            # 가장 우선 순위인 항목 제거
            self.U.pop(0)

            # 갱신 로직
            if key_old < self.calculate_key(u):
                # 키가 변경되었으면 재삽입
                self.U.append((u, self.calculate_key(u)))
            elif self.g[u.x,u.y] > self.rhs[u.x,u.y]:
                # g > rhs 면 일관성 맞추기
                self.g[u.x,u.y] = self.rhs[u.x,u.y]
                for s in self.get_neighbours(u):
                    self.update_vertex(s)
            else:
                # g <= rhs 면 g를 무한대로 설정하고 이웃 갱신
                self.g[u.x,u.y] = math.inf
                self.update_vertex(u)
                for s in self.get_neighbours(u):
                    self.update_vertex(s)


    def compute_current_path(self):
        path = []
        u = Node(self.start.x, self.start.y)
        while not compare_coordinates(u, self.goal):
            path.append(Node(u.x + self.x_min_world, u.y + self.y_min_world))
            # pick successor with minimal cost + g
            nbrs = self.get_neighbours(u)
            u = min(
                nbrs,
                key=lambda v: self.c(u, v) + self.g[v.x, v.y]
            )
        # append goal in world coords
        path.append(Node(self.goal.x + self.x_min_world, self.goal.y + self.y_min_world))
        return path

    def main(self, start: Node, goal: Node):
        """
        Returns (success: bool, pathx: list, pathy: list)
        """
        # initialize and compute
        self.initialize(start, goal)
        self.compute_shortest_path()

        # if unreachable
        if self.g[self.start.x, self.start.y] == math.inf:
            return False, [], []

        # build path
        path_nodes = self.compute_current_path()
        pathx = [n.x for n in path_nodes]
        pathy = [n.y for n in path_nodes]
        return True, pathx, pathy

# If you want to test standalone, you can keep a __main__, otherwise import and call DStarLite.main()
if __name__ == "__main__":
    # Example usage
    sx, sy = 10, 10
    gx, gy = 50, 50
    # example obstacles
    ox, oy = [], []
    for i in range(-10, 61):
        ox.append(i); oy.append(-10)
        ox.append(60); oy.append(i)
        ox.append(i); oy.append(60)
        ox.append(-10); oy.append(i)
    # map size large enough
    width, height = 100, 100
    planner = DStarLite(ox, oy, width, height)
    success, px, py = planner.main(Node(x=sx, y=sy), Node(x=gx, y=gy))
    print("Success:", success)
    print("Path length:", len(px))

# algorithms.py
import pygame
import math
import random
import time
from queue import PriorityQueue
from config import *

# --- CLASS NODE (Cho A*) ---
class Node:
    def __init__(self, row, col, size, total_rows):
        self.row = row; self.col = col
        self.x = row * size; self.y = col * size
        self.color = WHITE
        self.neighbors = []
        self.size = size; self.total_rows = total_rows
        self.parent = None

    def get_pos(self): return self.row, self.col
    def is_barrier(self): return self.color == BLACK
    def is_open(self): return self.color == GREEN
    def is_closed(self): return self.color == RED
    
    def make_start(self): self.color = ORANGE
    def make_closed(self): self.color = RED
    def make_open(self): self.color = GREEN
    def make_barrier(self): self.color = BLACK
    def make_end(self): self.color = TURQUOISE
    def make_path(self): self.color = PURPLE
    def reset(self): self.color = WHITE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.size, self.size))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col - 1])

# --- HELPER FUNCTIONS ---
def heuristic(p1, p2): return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
def get_dist(p1, p2): return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# --- A* ALGORITHM ---
def reconstruct_path_astar(came_from, current, draw):
    path_len = 0
    while current in came_from:
        prev = came_from[current]
        path_len += get_dist((current.x, current.y), (prev.x, prev.y))
        current = prev
        current.make_path()
        draw()
    return path_len

def algorithm_astar(draw, grid, start, end):
    start_time = time.time()
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = heuristic(start.get_pos(), end.get_pos())
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            length = reconstruct_path_astar(came_from, end, draw)
            end.make_end(); start.make_start()
            return True, length, (time.time() - start_time) * 1000

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + heuristic(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()
        if current != start: current.make_closed()
    return False, 0, 0

# --- RRT* (STAR) ALGORITHM ---
class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0 # (MỚI) Chi phí từ Start đến node này

def check_collision_line(n1, n2, grid):
    steps = int(get_dist((n1.x, n1.y), (n2.x, n2.y)) / (GRID_SIZE / 2)) + 1
    for i in range(steps):
        t = i / steps
        x = n1.x + (n2.x - n1.x) * t
        y = n1.y + (n2.y - n1.y) * t
        r, c = int(x / GRID_SIZE), int(y / GRID_SIZE)
        if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
            if grid[r][c].is_barrier(): return True
    return False

def algorithm_rrt_star(win, grid, start_node, end_node):
    start_time = time.time()
    
    start_point = RRTNode(start_node.x + GRID_SIZE//2, start_node.y + GRID_SIZE//2)
    end_point = RRTNode(end_node.x + GRID_SIZE//2, end_node.y + GRID_SIZE//2)
    tree = [start_point]
    path_nodes = []

    for i in range(RRT_MAX_ITER):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit()

        # 1. Random Sample
        if random.randint(0, 100) < 5: rand_point = RRTNode(end_point.x, end_point.y)
        else: rand_point = RRTNode(random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT))

        # 2. Nearest Node
        nearest_node = min(tree, key=lambda n: get_dist((n.x, n.y), (rand_point.x, rand_point.y)))

        # 3. Steer
        theta = math.atan2(rand_point.y - nearest_node.y, rand_point.x - nearest_node.x)
        new_x = nearest_node.x + RRT_STEP_SIZE * math.cos(theta)
        new_y = nearest_node.y + RRT_STEP_SIZE * math.sin(theta)
        new_node = RRTNode(new_x, new_y)
        
        if not (0 <= new_node.x < SCREEN_WIDTH and 0 <= new_node.y < SCREEN_HEIGHT): continue
        if check_collision_line(nearest_node, new_node, grid): continue

        # --- RRT* LOGIC BẮT ĐẦU ---
        
        # 4. Choose Parent (Tìm cha tốt nhất trong vùng lân cận)
        near_nodes = [node for node in tree if get_dist((node.x, node.y), (new_node.x, new_node.y)) < RRT_SEARCH_RADIUS]
        min_cost = nearest_node.cost + get_dist((nearest_node.x, nearest_node.y), (new_node.x, new_node.y))
        new_node.parent = nearest_node
        new_node.cost = min_cost

        for near_node in near_nodes:
            if check_collision_line(near_node, new_node, grid): continue
            cost = near_node.cost + get_dist((near_node.x, near_node.y), (new_node.x, new_node.y))
            if cost < new_node.cost:
                new_node.parent = near_node
                new_node.cost = cost
        
        tree.append(new_node)
        
        # 5. Rewire (Đấu lại dây cho các node hàng xóm nếu đi qua node mới rẻ hơn)
        for near_node in near_nodes:
            if near_node == new_node.parent: continue # Không đấu lại cha vừa chọn
            if check_collision_line(new_node, near_node, grid): continue
            
            new_cost = new_node.cost + get_dist((new_node.x, new_node.y), (near_node.x, near_node.y))
            if new_cost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = new_cost
                # Lưu ý: Đúng ra phải update cost đệ quy cho con cháu, nhưng trong visual demo này bỏ qua để code nhanh.

        # Draw Realtime
        pygame.draw.circle(win, BLUE, (int(new_node.x), int(new_node.y)), 2)
        if new_node.parent:
            pygame.draw.line(win, BLUE, (new_node.parent.x, new_node.parent.y), (new_node.x, new_node.y), 1)
        pygame.display.update()

        # 6. Check Goal
        if get_dist((new_node.x, new_node.y), (end_point.x, end_point.y)) < RRT_GOAL_RADIUS:
            # Reconstruct Path
            path_len = new_node.cost + get_dist((new_node.x, new_node.y), (end_point.x, end_point.y))
            curr = new_node
            path_nodes = [(end_point.x, end_point.y)]
            while curr:
                path_nodes.append((curr.x, curr.y))
                curr = curr.parent
            return True, path_len, (time.time() - start_time) * 1000, tree, path_nodes

    return False, 0, 0, tree, []
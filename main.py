# main.py
import pygame
import matplotlib
# Sử dụng backend TkAgg để tương thích tốt với cửa sổ Pygame
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import sys
import numpy as np
from config import *
from algorithms import Node, algorithm_astar, algorithm_rrt_star

pygame.init()
WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Project II: A* vs RRT* (With Success Rate)")

# --- GLOBAL DATA HISTORY (Cấu trúc mới) ---
# Bây giờ ta lưu thêm số lần 'attempts' (thử) và 'successes' (thành công)
history = {
    "astar": {"times": [], "lengths": [], "attempts": 0, "successes": 0},
    "rrt_star": {"times": [], "lengths": [], "attempts": 0, "successes": 0}
}

plt.ion() 
# Tăng kích thước lên (15, 5) để chứa 3 biểu đồ
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
fig.canvas.manager.set_window_title('Metrics: Time vs Length vs Success Rate')

def update_charts():
    ax1.clear(); ax2.clear(); ax3.clear()
    
    algos = ['A*', 'RRT*']
    colors = ['#90EE90', '#87CEEB'] # Xanh lá vs Xanh dương
    
    # 1. TÍNH TOÁN DỮ LIỆU
    # Thời gian & Quãng đường trung bình (chỉ tính các lần thành công)
    avg_times = [
        np.mean(history["astar"]["times"]) if history["astar"]["times"] else 0,
        np.mean(history["rrt_star"]["times"]) if history["rrt_star"]["times"] else 0
    ]
    avg_lens = [
        np.mean(history["astar"]["lengths"]) if history["astar"]["lengths"] else 0,
        np.mean(history["rrt_star"]["lengths"]) if history["rrt_star"]["lengths"] else 0
    ]
    
    # Tỷ lệ thành công = (Thành công / Tổng thử) * 100
    success_rates = []
    for algo in ["astar", "rrt_star"]:
        attempts = history[algo]["attempts"]
        successes = history[algo]["successes"]
        rate = (successes / attempts * 100) if attempts > 0 else 0
        success_rates.append(rate)

    # 2. VẼ BIỂU ĐỒ 1: THỜI GIAN (Time)
    bars1 = ax1.bar(algos, avg_times, color=colors)
    ax1.set_title('Avg Time (ms) - Lower is Better')
    ax1.set_ylabel('ms')
    for bar in bars1:
        h = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2, h, f"{h:.1f}", ha='center', va='bottom')

    # 3. VẼ BIỂU ĐỒ 2: QUÃNG ĐƯỜNG (Length)
    bars2 = ax2.bar(algos, avg_lens, color=colors)
    ax2.set_title('Avg Path Length (px) - Lower is Better')
    ax2.set_ylabel('Pixels')
    for bar in bars2:
        h = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2, h, f"{int(h)}", ha='center', va='bottom')

    # 4. VẼ BIỂU ĐỒ 3: TỶ LỆ THÀNH CÔNG (Success Rate)
    # Nếu < 100% thì tô màu đỏ cảnh báo
    rate_colors = []
    for r, base_col in zip(success_rates, colors):
        rate_colors.append(base_col if r == 100 else '#FF6347') # Màu cà chua nếu fail

    bars3 = ax3.bar(algos, success_rates, color=rate_colors)
    ax3.set_title('Success Rate (%) - Higher is Better')
    ax3.set_ylabel('%')
    ax3.set_ylim(0, 110) # Cố định trục Y max là 110 để dễ nhìn
    for i, bar in enumerate(bars3):
        h = bar.get_height()
        total = history["astar" if i==0 else "rrt_star"]["attempts"]
        ax3.text(bar.get_x() + bar.get_width()/2, h, f"{h:.1f}%\n(n={total})", ha='center', va='bottom', fontweight='bold')

    plt.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()

def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid

def draw(win, grid, rows, width, rrt_tree, rrt_path):
    win.fill(WHITE)
    for row in grid:
        for node in row:
            node.draw(win)
    
    # Vẽ lại lưới mờ hơn chút để dễ nhìn đường
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, (220, 220, 220), (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, (220, 220, 220), (j * gap, 0), (j * gap, width))

    if rrt_tree:
        for node in rrt_tree:
            if node.parent:
                pygame.draw.line(win, BLUE, (node.x, node.y), (node.parent.x, node.parent.y), 1)
    if rrt_path and len(rrt_path) > 1:
        for i in range(len(rrt_path) - 1):
            pygame.draw.line(win, DARK_GREEN, rrt_path[i], rrt_path[i+1], 4)
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col

def reset_algo_visuals(grid):
    for row in grid:
        for node in row:
            if node.is_open() or node.is_closed() or node.color == PURPLE:
                node.reset()

def main():
    global history  # <--- THÊM DÒNG NÀY ĐỂ SỬA LỖI
    
    grid = make_grid(ROWS, SCREEN_WIDTH)
    start = None; end = None
    rrt_tree_result = []; rrt_path_result = []
    run = True

    update_charts() # Hiện biểu đồ trống ban đầu

    print("\n--- CONTROLS ---")
    print("[LEFT CLICK]: Vẽ Start -> End -> Wall")
    print("[SPACE]: Chạy A*")
    print("[R]: Chạy RRT*")
    print("[K]: Giữ Map (Xóa đường đi để chạy lại)")
    print("[C]: Xóa hết (Reset metrics)")

    while run:
        draw(WIN, grid, ROWS, SCREEN_WIDTH, rrt_tree_result, rrt_path_result)
        plt.pause(0.001)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: run = False

            if pygame.mouse.get_pressed()[0]: 
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, SCREEN_WIDTH)
                if row < ROWS and col < COLS:
                    node = grid[row][col]
                    if not start and node != end: start = node; start.make_start()
                    elif not end and node != start: end = node; end.make_end()
                    elif node != end and node != start: node.make_barrier()

            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, SCREEN_WIDTH)
                if row < ROWS and col < COLS:
                    node = grid[row][col]
                    node.reset()
                    if node == start: start = None
                    if node == end: end = None

            if event.type == pygame.KEYDOWN:
                # --- A* ---
                if event.key == pygame.K_SPACE and start and end:
                    rrt_tree_result = []; rrt_path_result = []
                    reset_algo_visuals(grid)
                    for row in grid:
                        for node in row: node.update_neighbors(grid)
                    
                    history["astar"]["attempts"] += 1
                    
                    found, length, t_exec = algorithm_astar(lambda: draw(WIN, grid, ROWS, SCREEN_WIDTH, [], []), grid, start, end)
                    
                    if found:
                        history["astar"]["successes"] += 1
                        history["astar"]["times"].append(t_exec)
                        history["astar"]["lengths"].append(length)
                        print(f"A* SUCCESS: {t_exec:.2f}ms")
                    else:
                        print("A* FAILED!")
                    
                    update_charts()

                # --- RRT* ---
                if event.key == pygame.K_r and start and end:
                    reset_algo_visuals(grid)
                    
                    history["rrt_star"]["attempts"] += 1
                    
                    found, length, t_exec, tree, path = algorithm_rrt_star(WIN, grid, start, end)
                    
                    if found:
                        history["rrt_star"]["successes"] += 1
                        history["rrt_star"]["times"].append(t_exec)
                        history["rrt_star"]["lengths"].append(length)
                        rrt_tree_result = tree
                        rrt_path_result = path
                        print(f"RRT* SUCCESS: {t_exec:.2f}ms")
                    else:
                        rrt_tree_result = tree 
                        rrt_path_result = []
                        print("RRT* FAILED!")

                    update_charts()

                # --- KEEP MAP ---
                if event.key == pygame.K_k:
                    reset_algo_visuals(grid)
                    rrt_tree_result = []; rrt_path_result = []
                    print("Map kept. Metrics preserved.")

                # --- CLEAR ALL ---
                if event.key == pygame.K_c:
                    start = None; end = None
                    grid = make_grid(ROWS, SCREEN_WIDTH)
                    rrt_tree_result = []; rrt_path_result = []
                    
                    # Reset toàn bộ lịch sử
                    history = {
                        "astar": {"times": [], "lengths": [], "attempts": 0, "successes": 0},
                        "rrt_star": {"times": [], "lengths": [], "attempts": 0, "successes": 0}
                    }
                    update_charts()
                    print("All cleared.")

    pygame.quit()
    plt.close()

if __name__ == "__main__":
    main()
# config.py
import pygame

# --- CẤU HÌNH MÀN HÌNH ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
GRID_SIZE = 20
ROWS = SCREEN_HEIGHT // GRID_SIZE
COLS = SCREEN_WIDTH // GRID_SIZE

# --- MÀU SẮC ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (220, 220, 220)
RED = (255, 100, 100)       # A* Closed
GREEN = (100, 255, 100)     # A* Open
ORANGE = (255, 165, 0)      # Start
TURQUOISE = (64, 224, 208)  # Goal
PURPLE = (128, 0, 128)      # Path A*
BLUE = (100, 149, 237)      # RRT* Tree
DARK_GREEN = (0, 100, 0)    # Path RRT*

# --- CẤU HÌNH RRT* (RRT-STAR) ---
RRT_STEP_SIZE = 30          # Bước nhảy
RRT_MAX_ITER = 5000         # Số vòng lặp tối đa
RRT_GOAL_RADIUS = 20        # Bán kính đích
RRT_SEARCH_RADIUS = 50      # (MỚI) Bán kính tìm hàng xóm để tối ưu nối dây
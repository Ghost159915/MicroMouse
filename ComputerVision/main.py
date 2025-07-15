import numpy as np
import maze_image_processing as mip
import graph_builder as gb
import pathfinder as pf
import visualize

# === CONFIG ===
image_path = "MAZE.jpg"
num_rows = 5
num_cols = 9
border_size = 7
cell_pixel_size = 40
start = (0, 0)
goal = (4, 8)
start_facing = 2
pts_src = np.array([
    [0, 0],
    [347, 0],
    [347, 194],
    [0, 194]
], dtype=np.float32)

# === Load and process image ===
width = num_cols * cell_pixel_size
height = num_rows * cell_pixel_size
warped = mip.load_and_warp_image(image_path, pts_src, width, height)
binary = mip.threshold_image(warped)
H, V = mip.detect_walls(binary, num_rows, num_cols, border_size)

# === Build Graph ===
graph = gb.build_graph(H, V, num_rows, num_cols)

# === Find Path and Commands ===
path = pf.bfs(graph, start, goal)
if not path:
    print("")
    exit()
commands = pf.path_to_commands(path, start_facing)

# === Show Maze ===
visualize.draw_maze_with_path(H, V, path, cell_pixel_size)

# === Output Commands ===
print(commands)

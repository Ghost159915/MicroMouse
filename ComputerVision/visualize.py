import numpy as np
import cv2

def draw_maze_with_path(H, V, path, cell_pixel_size):
    num_rows = H.shape[0] - 1
    num_cols = V.shape[1] - 1
    vis_height = (num_rows + 1) * cell_pixel_size
    vis_width = (num_cols + 1) * cell_pixel_size

    # Start with white background
    maze_img = np.ones((vis_height, vis_width), dtype=np.uint8) * 255

    # Draw black walls
    for i in range(num_rows + 1):
        for j in range(num_cols):
            if H[i, j]:
                y = i * cell_pixel_size
                x_start = j * cell_pixel_size
                x_end = (j + 1) * cell_pixel_size
                cv2.line(maze_img, (x_start, y), (x_end, y), color=0, thickness=2)

    for i in range(num_rows):
        for j in range(num_cols + 1):
            if V[i, j]:
                x = j * cell_pixel_size
                y_start = i * cell_pixel_size
                y_end = (i + 1) * cell_pixel_size
                cv2.line(maze_img, (x, y_start), (x, y_end), color=0, thickness=2)

    # Convert to color so we can draw in red
    maze_img = cv2.cvtColor(maze_img, cv2.COLOR_GRAY2BGR)

    # Draw the path as red arrows
    if path:
        for k in range(1, len(path)):
            prev = path[k - 1]
            curr = path[k]
            x1 = prev[1] * cell_pixel_size + cell_pixel_size // 2
            y1 = prev[0] * cell_pixel_size + cell_pixel_size // 2
            x2 = curr[1] * cell_pixel_size + cell_pixel_size // 2
            y2 = curr[0] * cell_pixel_size + cell_pixel_size // 2
            cv2.arrowedLine(maze_img, (x1, y1), (x2, y2), color=(0,0,255), thickness=2, tipLength=0.4)

    # Show window
    cv2.imshow("Detected Maze with Planned Path", maze_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

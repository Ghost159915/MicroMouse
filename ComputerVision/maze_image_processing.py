import cv2
import numpy as np

def load_and_warp_image(image_path, pts_src, width, height):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not load image: {image_path}")
    M = cv2.getPerspectiveTransform(pts_src, np.array([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1]
    ], dtype=np.float32))
    return cv2.warpPerspective(image, M, (width, height))

def threshold_image(image):
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    return cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

def detect_walls(binary, num_rows, num_cols, border_size):
    cell_height = binary.shape[0] // num_rows
    cell_width  = binary.shape[1] // num_cols
    H = np.zeros((num_rows + 1, num_cols), dtype=int)
    V = np.zeros((num_rows, num_cols + 1), dtype=int)

    for i in range(num_rows):
        for j in range(num_cols):
            y_start = i * cell_height
            y_end = (i + 1) * cell_height
            x_start = j * cell_width
            x_end = (j + 1) * cell_width
            cell = binary[y_start:y_end, x_start:x_end]
            if np.mean(cell[0:border_size, :]) < 150:
                H[i, j] = 1
            if np.mean(cell[-border_size:, :]) < 150:
                H[i + 1, j] = 1
            if np.mean(cell[:, 0:border_size]) < 150:
                V[i, j] = 1
            if np.mean(cell[:, -border_size:]) < 150:
                V[i, j + 1] = 1
    return H, V

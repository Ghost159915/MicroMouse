def build_graph(H, V, num_rows, num_cols):
    graph = {}
    for i in range(num_rows):
        for j in range(num_cols):
            node = (i, j)
            neighbors = []
            if i > 0 and H[i, j] == 0:
                neighbors.append((i - 1, j))
            if i < num_rows - 1 and H[i + 1, j] == 0:
                neighbors.append((i + 1, j))
            if j > 0 and V[i, j] == 0:
                neighbors.append((i, j - 1))
            if j < num_cols - 1 and V[i, j + 1] == 0:
                neighbors.append((i, j + 1))
            graph[node] = neighbors
    return graph

from collections import deque

def bfs(graph, start, goal):
    queue = deque([start])
    visited = {start}
    parent = {}
    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append(neighbor)
    if goal not in parent and start != goal:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

def path_to_commands(path, start_facing):
    directions = [(1,0), (0,1), (-1,0), (0,-1)]  # S, E, N, W
    dir_idx = start_facing
    commands = ""
    for k in range(1, len(path)):
        dy, dx = path[k][0] - path[k-1][0], path[k][1] - path[k-1][1]
        move_dir = (dy, dx)
        desired_idx = directions.index(move_dir)
        turn_count = (desired_idx - dir_idx) % 4
        if turn_count == 1:
            commands += "L"   # Swap R<->L here if desired
        elif turn_count == 2:
            commands += "RR"
        elif turn_count == 3:
            commands += "R"
        commands += "F"
        dir_idx = desired_idx
    return commands

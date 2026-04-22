import math
from collections import deque
import numpy as np


def occupancy_to_numpy(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    return np.array(map_msg.data, dtype=np.int16).reshape((height, width))


def grid_to_world(gx, gy, map_msg):
    origin = map_msg.info.origin.position
    res = map_msg.info.resolution
    wx = origin.x + (gx + 0.5) * res
    wy = origin.y + (gy + 0.5) * res
    return wx, wy


def world_to_grid(wx, wy, map_msg):
    origin = map_msg.info.origin.position
    res = map_msg.info.resolution
    gx = int((wx - origin.x) / res)
    gy = int((wy - origin.y) / res)
    return gx, gy


def extract_frontiers(map_msg):
    data = occupancy_to_numpy(map_msg)
    h, w = data.shape
    frontiers = []

    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if data[y, x] != 0:
                continue
            local = data[y - 1:y + 2, x - 1:x + 2]
            if np.any(local == -1):
                frontiers.append((x, y))

    return frontiers


def cluster_frontiers(frontier_cells):
    frontier_set = set(frontier_cells)
    visited = set()
    clusters = []

    for cell in frontier_cells:
        if cell in visited:
            continue

        q = deque([cell])
        visited.add(cell)
        cluster = []

        while q:
            cx, cy = q.popleft()
            cluster.append((cx, cy))

            for nx in range(cx - 1, cx + 2):
                for ny in range(cy - 1, cy + 2):
                    if (nx, ny) == (cx, cy):
                        continue
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        q.append((nx, ny))

        clusters.append(cluster)

    return clusters


def centroid_world(cluster, map_msg):
    xs = [p[0] for p in cluster]
    ys = [p[1] for p in cluster]
    gx = float(sum(xs)) / len(xs)
    gy = float(sum(ys)) / len(ys)
    return grid_to_world(gx, gy, map_msg)


def simplify_pose_list(poses, min_spacing=0.25):
    if not poses:
        return []

    result = [poses[0]]
    last = poses[0].pose.position

    for pose in poses[1:]:
        curr = pose.pose.position
        dist = math.hypot(curr.x - last.x, curr.y - last.y)
        if dist >= min_spacing:
            result.append(pose)
            last = curr

    if result[-1] != poses[-1]:
        result.append(poses[-1])

    return result
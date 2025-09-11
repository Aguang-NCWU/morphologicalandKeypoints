#!/usr/bin/env python3
import rospy
import cv2
import copy
import math
import os
import json
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from threading import Thread
from heapq import heappop, heappush

# ------------------ Global Variables ------------------
target_point = [0.0, 0.0]
need_planning = False
car_position = [0.0, 0.0]

# Get image size from ROS parameter server and calculate map ratio
image_size = rospy.get_param('image_size', 100)  # default = 100
# Map ratio - dynamically calculated based on image_size
# image_size = 50, 100, 200 correspond to map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0


# ------------------ Utility Functions ------------------
def to_polar_list(points, origin):
    """Convert a list of (x,y) points into (r, theta, idx) tuples (pure Python, avoid numpy dtype issues)."""
    out = []
    for idx, pt in enumerate(points):
        dx = float(pt[0]) - float(origin[0])
        dy = float(pt[1]) - float(origin[1])
        r = math.sqrt(dx * dx + dy * dy)
        theta = math.atan2(dy, dx)
        out.append((r, theta, idx))
    return out


def find_first_obstacle(A, B):
    """In polar coordinate sequence A, find the first edge covered by polar set B (return index or None)."""
    # A: list of tuples (r, theta, idx)
    # B: list of tuples (r, theta)
    for i in range(len(A) - 1):
        r1, t1, idx1 = A[i]
        r2, t2, idx2 = A[i + 1]
        rmin, rmax = min(r1, r2), max(r1, r2)
        tmin, tmax = min(t1, t2), max(t1, t2)
        rmin_idx = idx1 if r1 == rmin else idx2
        for (rB, tB) in B:
            if rB <= rmin and tmin <= tB <= tmax:
                return int(rmin_idx)
    return None


def polar_key_points(path, obstacle_coords):
    """Extract key points from a path based on polar coordinates and obstacle distribution."""
    if len(path) == 0:
        return []

    coords = [tuple(p) for p in path]
    origin = coords[0]
    key_points = [coords[0]]

    polar_coords = to_polar_list(coords, origin)
    obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, origin)]

    while True:
        obs_idx = find_first_obstacle(polar_coords, obstacle_polar)
        if obs_idx is not None and (obs_idx + 1) < len(coords):
            key_points.append(coords[obs_idx])
            coords = coords[obs_idx:]  # re-compute from this point
            polar_coords = to_polar_list(coords, coords[0])
            obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, coords[0])]
        else:
            break
    if coords:
        key_points.append(coords[-1])
    # Deduplicate while preserving order
    kp = []
    for p in key_points:
        if kp and kp[-1] == p:
            continue
        kp.append([int(p[0]), int(p[1])])
    return kp


# ------------------ JPS Main Logic ------------------
def JPS():
    global need_planning, target_point, car_position, image_size, map_ratio

    # Heuristic function - Chebyshev distance
    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)

    # Get neighbor directions
    def get_neighbors(pos, grid_map, grid_size):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for dx, dy in directions:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < grid_size and 0 <= ny < grid_size:
                if grid_map[ny, nx] == 0:
                    result.append((dx, dy))
        return result

    # Jump function - core of JPS
    def jump(current, dx, dy, goal, grid_map, grid_size):
        x, y = current
        x += dx
        y += dy
        if not (0 <= x < grid_size and 0 <= y < grid_size):
            return None
        if grid_map[y, x] == 1:
            return None
        if (x, y) == goal:
            return (x, y)

        # Forced neighbor check for diagonal moves
        if dx != 0 and dy != 0:
            if (grid_map[y - dy, x] == 1 and grid_map[y - dy, x - dx] == 0) or \
                    (grid_map[y, x - dx] == 1 and grid_map[y - dy, x - dx] == 0):
                return (x, y)
        else:
            # Horizontal move
            if dx != 0:
                if (grid_map[y + 1, x] == 1 and grid_map[y + 1, x - dx] == 0) or \
                        (grid_map[y - 1, x] == 1 and grid_map[y - 1, x - dx] == 0):
                    return (x, y)
            # Vertical move
            else:
                if (grid_map[y, x + 1] == 1 and grid_map[y - dy, x + 1] == 0) or \
                        (grid_map[y, x - 1] == 1 and grid_map[y - dy, x - 1] == 0):
                    return (x, y)

        # Continue jump along horizontal/vertical for diagonal move
        if dx != 0 and dy != 0:
            if jump((x, y), dx, 0, goal, grid_map, grid_size) or jump((x, y), 0, dy, goal, grid_map, grid_size):
                return (x, y)

        return jump((x, y), dx, dy, goal, grid_map, grid_size)

    # Node definition
    class Node:
        def __init__(self, position, g, h, parent=None):
            self.position = position
            self.g = g
            self.h = h
            self.f = g + h
            self.parent = parent

        def __lt__(self, other):
            return self.f < other.f

    # Read parameters (read once outside loop)
    try:
        resources_path = rospy.get_param('resourcesPath')
    except KeyError:
        rospy.logerr("Parameter 'resourcesPath' not found on param server.")
        return

    # Support both parameter names: image_name or image_base_name (just 'demo01')
    base_name = rospy.get_param('image_name', None)
    if base_name is None:
        base_name = rospy.get_param('image_base_name', 'demo01')
    image_name = base_name
    # If input is base name (e.g., demo01), append _100.png; if already .png, keep
    if not image_name.endswith('.png'):
        image_name = f"{image_name}_{image_size}.png"

    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: %s" % image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    if not os.path.exists(image_path):
        rospy.logerr("Map image not found: %s" % image_path)
        return

    img_orig = cv2.imread(image_path, 0)
    if img_orig is None:
        rospy.logerr("Failed to load map image (cv2 returned None): %s" % image_path)
        return

    # Morphological preprocessing
    kernel_size = 3 if image_size == 50 else 5
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    img = cv2.erode(img_orig, kernel, iterations=1)

    # Publisher
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    rate = rospy.Rate(10)
    route_list = []
    route_list_msg = String()

    while not rospy.is_shutdown():
        if need_planning:
            rospy.loginfo("Starting JPS path planning...")
            size = img.shape
            h = size[0]
            w = size[1]
            grid_size = image_size  # use dynamic image_size

            # Build grid map and obstacle list
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            obstacle_list = []
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        y_coord = h - iy - 1  # flip y
                        grid_map[y_coord, ix] = 1
                        obstacle_list.append([ix, y_coord])

            # Compute start and end points
            start_point = (int(round(car_position[0] + image_size / 2)), int(round(car_position[1] + image_size / 2)))
            end_point = (int(round(target_point[0] + image_size / 2)), int(round(target_point[1] + image_size / 2)))

            rospy.loginfo("Start (img coords): %s , End (img coords): %s" % (str(start_point), str(end_point)))

            # Bounds check
            def in_bounds(pt):
                return 0 <= pt[0] < grid_size and 0 <= pt[1] < grid_size

            if not in_bounds(start_point) or not in_bounds(end_point):
                rospy.logerr("Start or End out of map bounds. Aborting planning.")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Check if end is in obstacle
            if grid_map[end_point[1], end_point[0]] == 1:
                rospy.logerr("End point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # JPS algorithm
            open_list = []
            closed_set = set()

            start_node = Node(start_point, 0, heuristic(start_point, end_point))
            heappush(open_list, (start_node.f, start_node))
            path_found = False
            path = []

            max_iterations = 20000
            iterations = 0

            while open_list and not rospy.is_shutdown() and iterations < max_iterations:
                _, current = heappop(open_list)
                iterations += 1

                if current.position == end_point:
                    # Found path - backtrack
                    temp = current
                    while temp:
                        path.append(list(temp.position))
                        temp = temp.parent
                    path = path[::-1]
                    path_found = True
                    break

                closed_set.add(current.position)

                # Expand directions
                for dx, dy in get_neighbors(current.position, grid_map, grid_size):
                    jp = jump(current.position, dx, dy, end_point, grid_map, grid_size)
                    if jp and jp not in closed_set:
                        new_g = current.g + heuristic(current.position, jp)

                        # Check if already in open list
                        in_open = False
                        for _, node in open_list:
                            if node.position == jp:
                                in_open = True
                                if new_g < node.g:
                                    node.g = new_g
                                    node.f = new_g + node.h
                                    node.parent = current
                                break

                        if not in_open:
                            new_node = Node(jp, new_g, heuristic(jp, end_point), current)
                            heappush(open_list, (new_node.f, new_node))

            if iterations >= max_iterations:
                rospy.logwarn("JPS path planning exceeded maximum iterations (%d)!" % max_iterations)

            if not path_found:
                rospy.logerr("JPS failed to find a path to the target!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Extract key points (polar method)
            key_points = polar_key_points(path, obstacle_list)

            # Convert keypoints back to -image_size/2..+image_size/2 coordinate system
            route_list = []
            for kp in key_points:
                route_list.append([int(kp[0] - image_size / 2), int(kp[1] - image_size / 2)])

            rospy.loginfo("JPS path planning completed with %d points" % len(route_list))
            need_planning = False

        # Always publish (even empty)
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


# ------------------ Subscription Thread ------------------
def subData():
    global target_point, need_planning, car_position, map_ratio

    def getTargetPoint(msg_t):
        global target_point, need_planning
        if float(msg_t.x) != target_point[0] or float(msg_t.y) != target_point[1]:
            target_point[0] = float(msg_t.x)
            target_point[1] = float(msg_t.y)
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    def getCarPosition(msg_p):
        global car_position
        car_position = [float(msg_p.pose.pose.position.x) * map_ratio,
                        float(msg_p.pose.pose.position.y) * map_ratio]

    rospy.Subscriber("/target_point", Point, getTargetPoint)
    rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


# ------------------ Entry Point ------------------
if __name__ == "__main__":
    rospy.init_node("improved_jps_node")

    # Re-fetch parameters for image size and map ratio
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"JPS started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Start subscriber thread and JPS thread
    Thread(target=subData, daemon=True).start()
    Thread(target=JPS, daemon=True).start()
    rospy.spin()

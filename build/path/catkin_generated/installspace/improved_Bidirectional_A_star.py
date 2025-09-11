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

# Global variables
target_point = [0.0, 0.0]
need_planning = False
car_position = [0.0, 0.0]
image_size = rospy.get_param('image_size', 100)  # default value: 100
# Map scale - dynamically computed from image_size
# image_size = 50, 100, 200 → map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0


# ------------------ Utility functions ------------------
def to_polar_list(points, origin):
    """Convert a list of (x,y) points into a Python list of (r, theta, idx)
       using math (avoid numpy dtype issues)."""
    out = []
    for idx, pt in enumerate(points):
        dx = float(pt[0]) - float(origin[0])
        dy = float(pt[1]) - float(origin[1])
        r = math.sqrt(dx * dx + dy * dy)
        theta = math.atan2(dy, dx)
        out.append((r, theta, idx))
    return out


def find_first_obstacle(A, B):
    """In polar coordinate sequence A,
       find the first edge covered by polar set B (return index or None)."""
    # A: list of (r, theta, idx)
    # B: list of (r, theta)
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
    """Extract key points from a path (list of [x,y]) using polar coordinates."""
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
            coords = coords[obs_idx:]  # restart computation from this point
            polar_coords = to_polar_list(coords, coords[0])
            obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, coords[0])]
        else:
            break
    if coords:
        key_points.append(coords[-1])
    # Remove duplicates and return as list of [x,y]
    kp = []
    for p in key_points:
        if kp and kp[-1] == p:
            continue
        kp.append([int(p[0]), int(p[1])])
    return kp


# ------------------ Bidirectional A* main logic ------------------
def bidirectional_a_star():
    global need_planning, target_point, car_position, image_size, map_ratio

    # Node class
    class Node:
        def __init__(self, position, g, h, parent=None):
            self.position = position
            self.g = g
            self.h = h
            self.f = g + h
            self.parent = parent

        def __lt__(self, other):
            return self.f < other.f

    # Heuristic function - Chebyshev distance
    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)

    # Get neighbor nodes
    def get_neighbors(pos, grid_map, grid_size):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for dx, dy in directions:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < grid_size and 0 <= ny < grid_size:
                if grid_map[ny, nx] == 0:
                    result.append((nx, ny))
        return result

    # Reconstruct final path
    def reconstruct_path(meet_node_f, meet_node_b):
        path_f = []
        node = meet_node_f
        while node:
            path_f.append(node.position)
            node = node.parent
        path_f = path_f[::-1]

        path_b = []
        node = meet_node_b.parent  # avoid duplicate
        while node:
            path_b.append(node.position)
            node = node.parent

        return path_f + path_b

    # Core bidirectional A* algorithm
    def bidirectional_a_star_algorithm(start, goal, grid_map, grid_size):
        open_f = []  # forward open list
        open_b = []  # backward open list
        closed_f = {}  # forward closed list
        closed_b = {}  # backward closed list

        start_node = Node(start, 0, heuristic(start, goal))
        goal_node = Node(goal, 0, heuristic(goal, start))
        heappush(open_f, (start_node.f, start_node))
        heappush(open_b, (goal_node.f, goal_node))

        max_iterations = 20000
        iterations = 0

        while open_f and open_b and iterations < max_iterations:
            iterations += 1

            # Forward search
            if open_f:
                _, current_f = heappop(open_f)
                closed_f[current_f.position] = current_f

            # Backward search
            if open_b:
                _, current_b = heappop(open_b)
                closed_b[current_b.position] = current_b

            # Check intersection
            intersect = set(closed_f.keys()) & set(closed_b.keys())
            if intersect:
                meet_point = intersect.pop()
                return reconstruct_path(closed_f[meet_point], closed_b[meet_point])

            # Forward expansion
            for neighbor in get_neighbors(current_f.position, grid_map, grid_size):
                if neighbor in closed_f:
                    continue
                # Movement cost (√2 for diagonal, 1 for straight)
                dx = abs(neighbor[0] - current_f.position[0])
                dy = abs(neighbor[1] - current_f.position[1])
                cost = math.sqrt(2) if dx > 0 and dy > 0 else 1

                g = current_f.g + cost
                h = heuristic(neighbor, goal)
                new_node = Node(neighbor, g, h, current_f)

                # If already in open list, update if better
                found = False
                for i, (_, node) in enumerate(open_f):
                    if node.position == neighbor:
                        found = True
                        if g < node.g:
                            open_f[i] = (new_node.f, new_node)
                            open_f.sort(key=lambda x: x[0])  # re-heapify
                        break

                if not found:
                    heappush(open_f, (new_node.f, new_node))

            # Backward expansion
            for neighbor in get_neighbors(current_b.position, grid_map, grid_size):
                if neighbor in closed_b:
                    continue
                dx = abs(neighbor[0] - current_b.position[0])
                dy = abs(neighbor[1] - current_b.position[1])
                cost = math.sqrt(2) if dx > 0 and dy > 0 else 1

                g = current_b.g + cost
                h = heuristic(neighbor, start)
                new_node = Node(neighbor, g, h, current_b)

                found = False
                for i, (_, node) in enumerate(open_b):
                    if node.position == neighbor:
                        found = True
                        if g < node.g:
                            open_b[i] = (new_node.f, new_node)
                            open_b.sort(key=lambda x: x[0])
                        break

                if not found:
                    heappush(open_b, (new_node.f, new_node))

        rospy.logwarn("Bidirectional A* failed to find path within max iterations")
        return None

    # Load parameters (only once outside loop)
    try:
        resources_path = rospy.get_param('resourcesPath')
    except KeyError:
        rospy.logerr("Parameter 'resourcesPath' not found on param server.")
        return

    # Support two param names: image_name or image_base_name (only pass 'demo01')
    base_image_name = rospy.get_param('image_name', 'map')  # default: map.png
    image_name = f"{base_image_name}_{image_size}.png"  # construct dynamically
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: " + image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    if not os.path.exists(image_path):
        rospy.logerr("Map image not found: %s" % image_path)
        return

    img_orig = cv2.imread(image_path, 0)
    if img_orig is None:
        rospy.logerr("Failed to load map image (cv2 returned None): %s" % image_path)
        return

    # Morphological processing
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
            rospy.loginfo("Starting bidirectional A* path planning...")
            img_shape = img.shape
            h = img_shape[0]
            w = img_shape[1]
            grid_size = max(w, h)

            # Build obstacle list (integer coords; original code flipped y)
            obstacle_list = []
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        obstacle_list.append([ix, h - iy - 1])

            # Create grid map
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        grid_map[h - iy - 1, ix] = 1

            # Compute start and goal (use global image_size, not hardcoded 50)
            start_point = (int(round(car_position[0] + image_size / 2)), int(round(car_position[1] + image_size / 2)))
            end_point = (int(round(target_point[0] + image_size / 2)), int(round(target_point[1] + image_size / 2)))

            rospy.loginfo("Start (img coords): %s , End (img coords): %s" % (str(start_point), str(end_point)))

            # Boundary check
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

            # Check if goal is in obstacle
            if grid_map[end_point[1], end_point[0]] == 1:
                rospy.logerr("End point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Check if start is in obstacle
            if grid_map[start_point[1], start_point[0]] == 1:
                rospy.logerr("Start point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Run bidirectional A* algorithm
            path = bidirectional_a_star_algorithm(start_point, end_point, grid_map, grid_size)

            if path is None:
                rospy.logerr("Bidirectional A* failed to find a path to the target!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Convert path to list format
            route_seq = [list(p) for p in path]

            # Extract keypoints (polar method)
            key_points = polar_key_points(route_seq, obstacle_list)
            # Convert back to -50..+50 range and publish (use image_size, not hardcoded 50)
            route_list = []
            for kp in key_points:
                route_list.append([int(kp[0] - image_size / 2), int(kp[1] - image_size / 2)])

            rospy.loginfo("Bidirectional A* path planning completed with %d points" % len(route_list))
            need_planning = False

        # Always publish (even if empty)
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


# ------------------ Subscriber thread ------------------
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


# ------------------ Main entry ------------------
if __name__ == "__main__":
    rospy.init_node("improved_bidirectional_a_star_node")

    # Load parameters from ROS param server
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0

    # Launch subscriber and A* threads
    Thread(target=subData, daemon=True).start()
    Thread(target=bidirectional_a_star, daemon=True).start()
    rospy.spin()

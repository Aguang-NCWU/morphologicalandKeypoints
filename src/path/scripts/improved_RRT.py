#!/usr/bin/env python3
import rospy
import cv2
import copy
import math
import os
import json
import numpy as np
import random

from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from threading import Thread

# ------------------ Global Variables ------------------
target_point = [0.0, 0.0]
need_planning = False
car_position = [0.0, 0.0]

# Get image size parameter from ROS parameter server and calculate map ratio
image_size = rospy.get_param('image_size', 100)  # Default value = 100
# Map ratio - calculated dynamically from image_size
# image_size = 50, 100, 200 corresponds to map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0


# ------------------ Utility Functions ------------------
def to_polar_list(points, origin):
    """Convert a set of (x,y) points into (r, theta, idx) list using math (avoid numpy type issues)."""
    out = []
    for idx, pt in enumerate(points):
        dx = float(pt[0]) - float(origin[0])
        dy = float(pt[1]) - float(origin[1])
        r = math.sqrt(dx * dx + dy * dy)
        theta = math.atan2(dy, dx)
        out.append((r, theta, idx))
    return out


def find_first_obstacle(A, B):
    """
    In polar coordinate sequence A, find the first segment blocked by polar set B.
    Return corresponding index or None.
    """
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
    """Extract key points from path (list of [x,y]) using polar coordinates method."""
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
            coords = coords[obs_idx:]  # Recompute starting from this point
            polar_coords = to_polar_list(coords, coords[0])
            obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, coords[0])]
        else:
            break
    if coords:
        key_points.append(coords[-1])

    # Remove duplicates and return list of [x,y]
    kp = []
    for p in key_points:
        if kp and kp[-1] == p:
            continue
        kp.append([int(p[0]), int(p[1])])
    return kp


# ------------------ RRT Main Logic ------------------
def RRT_planner():
    global need_planning, target_point, car_position, image_size, map_ratio

    # --- Core Functions ---
    def distance(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def is_free(point, grid_map, grid_size):
        x, y = point
        if 0 <= x < grid_size and 0 <= y < grid_size:
            return grid_map[y, x] == 0
        return False

    def line_collision_check(p1, p2, grid_map, grid_size):
        """Check if line between p1 and p2 crosses obstacles (Bresenham's algorithm)."""
        x1, y1 = p1
        x2, y2 = p2
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        n = 1 + dx + dy
        x_inc = 1 if x2 > x1 else -1
        y_inc = 1 if y2 > y1 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        for _ in range(n):
            if not is_free((x, y), grid_map, grid_size):
                return False
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        return True

    def nearest_node(nodes, random_point):
        """Find the nearest node to a random point."""
        min_dist = float('inf')
        nearest = None
        for node in nodes:
            dist = distance(node['point'], random_point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def steer(from_node, to_point, step_size=5):
        """Steer from a node towards a point with step size."""
        from_x, from_y = from_node['point']
        to_x, to_y = to_point
        dist = distance(from_node['point'], to_point)
        if dist < step_size:
            new_point = to_point
        else:
            theta = math.atan2(to_y - from_y, to_x - from_x)
            new_x = int(from_x + step_size * math.cos(theta))
            new_y = int(from_y + step_size * math.sin(theta))
            new_point = (new_x, new_y)
        return new_point

    def rrt_algorithm(start, goal, grid_map, grid_size, max_iter=5000, step_size=5, goal_sample_rate=0.1):
        """Core RRT algorithm."""
        nodes = [{'point': start, 'parent': None}]

        for i in range(max_iter):
            # With some probability, sample goal directly
            if random.random() < goal_sample_rate:
                rnd_point = goal
            else:
                rnd_point = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))

            nearest = nearest_node(nodes, rnd_point)
            new_point = steer(nearest, rnd_point, step_size)

            if is_free(new_point, grid_map, grid_size) and line_collision_check(nearest['point'], new_point, grid_map, grid_size):
                new_node = {'point': new_point, 'parent': nearest}
                nodes.append(new_node)

                # Check if goal is reached
                if distance(new_point, goal) <= step_size:
                    if line_collision_check(new_point, goal, grid_map, grid_size):
                        goal_node = {'point': goal, 'parent': new_node}
                        nodes.append(goal_node)
                        # Backtrack to form path
                        path = []
                        node = goal_node
                        while node:
                            path.append(node['point'])
                            node = node['parent']
                        return path[::-1]  # From start to goal

        rospy.logwarn("RRT failed to find path within max iterations")
        return None

    # --- Load parameters ---
    try:
        resources_path = rospy.get_param('resourcesPath')
    except KeyError:
        rospy.logerr("Parameter 'resourcesPath' not found on param server.")
        return

    # Support two names: image_name or image_base_name (e.g. 'demo01')
    base_name = rospy.get_param('image_name', None)
    if base_name is None:
        base_name = rospy.get_param('image_base_name', 'demo01')
    image_name = base_name
    # If base name only, append _100.png; if already .png, keep
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
    kernel_size = 3 if image_size == 50 else 7
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    img = cv2.erode(img_orig, kernel, iterations=1)

    # Publisher
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    rate = rospy.Rate(10)
    route_list = []
    route_list_msg = String()

    while not rospy.is_shutdown():
        if need_planning:
            rospy.loginfo("Starting RRT path planning...")
            size = img.shape
            h = size[0]
            w = size[1]
            grid_size = image_size

            # Build grid map and obstacle list
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            obstacle_list = []
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        y_coord = h - iy - 1
                        grid_map[y_coord, ix] = 1
                        obstacle_list.append([ix, y_coord])

            # Compute start and goal (shift by image_size/2)
            start_point = (int(round(car_position[0] + image_size / 2)), int(round(car_position[1] + image_size / 2)))
            end_point = (int(round(target_point[0] + image_size / 2)), int(round(target_point[1] + image_size / 2)))

            rospy.loginfo("Start (img coords): %s , End (img coords): %s" % (str(start_point), str(end_point)))

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

            if grid_map[end_point[1], end_point[0]] == 1:
                rospy.logerr("End point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Run RRT
            path = rrt_algorithm(start_point, end_point, grid_map, grid_size)

            if path is None:
                rospy.logerr("RRT failed to find a path to the target!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Extract key points (polar method)
            key_points = polar_key_points(path, obstacle_list)

            # Convert key points back to coordinate system (-image_size/2 .. +image_size/2)
            route_list = []
            for kp in key_points:
                route_list.append([int(kp[0] - image_size / 2), int(kp[1] - image_size / 2)])

            rospy.loginfo("RRT path planning completed with %d points" % len(route_list))
            need_planning = False

        # Always publish (even if empty)
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
        car_position = [float(msg_p.pose.pose.position.x) * map_ratio, float(msg_p.pose.pose.position.y) * map_ratio]

    rospy.Subscriber("/target_point", Point, getTargetPoint)
    rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


# ------------------ Main Entry ------------------
if __name__ == "__main__":
    rospy.init_node("improved_rrt_node")

    # Reload parameters (in case of runtime change)
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"RRT started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Start subscription and RRT threads
    Thread(target=subData, daemon=True).start()
    Thread(target=RRT_planner, daemon=True).start()
    rospy.spin()

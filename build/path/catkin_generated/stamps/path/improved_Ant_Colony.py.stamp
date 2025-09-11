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

# Global variables
target_point = [0.0, 0.0]
need_planning = False
car_position = [0.0, 0.0]
image_size = rospy.get_param('image_size', 100)  # Default value = 100
# Map ratio - dynamically calculated based on image_size
size = int(image_size)
# image_size = 50, 100, 200 correspond to map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0


# ------------------ Utility Functions ------------------
def to_polar_list(points, origin):
    """Convert a set of (x,y) points to a list of (r, theta, idx) using math (avoid numpy float issues)."""
    out = []
    for idx, pt in enumerate(points):
        dx = float(pt[0]) - float(origin[0])
        dy = float(pt[1]) - float(origin[1])
        r = math.sqrt(dx * dx + dy * dy)
        theta = math.atan2(dy, dx)
        out.append((r, theta, idx))
    return out


def find_first_obstacle(A, B):
    """In polar coordinate sequence A, find the first segment covered by obstacle set B (return index or None)."""
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
    """Extract key points from path (list of [x,y]) using polar coordinate method."""
    if len(path) == 0:
        return []

    coords = [tuple(p) for p in path]
    origin = coords[0]
    key_points = [coords[0]]

    polar_coords = to_polar_list(coords, origin)  # list of tuples
    obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, origin)]

    while True:
        obs_idx = find_first_obstacle(polar_coords, obstacle_polar)
        if obs_idx is not None and (obs_idx + 1) < len(coords):
            key_points.append(coords[obs_idx])
            coords = coords[obs_idx:]  # Recalculate from this point
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


# ------------------ Ant Colony Algorithm ------------------
def ant_colony_optimization():
    global need_planning, target_point, car_position, image_size, map_ratio

    class Ant:
        def __init__(self, start, goal, grid_map, grid_size):
            self.position = start
            self.goal = goal
            self.grid_map = grid_map
            self.grid_size = grid_size
            self.path = [start]
            self.visited = set([start])
            self.reached_goal = False

        def move(self, pheromone_map, alpha, beta):
            if self.position == self.goal:
                self.reached_goal = True
                return

            # Get feasible neighbors
            neighbors = self.get_neighbors(self.position)
            if not neighbors:
                return

            # Calculate transition probabilities
            probabilities = []
            total = 0.0

            for neighbor in neighbors:
                if neighbor in self.visited:
                    continue

                # Heuristic (inverse of distance to goal)
                heuristic = 1.0 / (
                        1.0 + math.sqrt((neighbor[0] - self.goal[0]) ** 2 + (neighbor[1] - self.goal[1]) ** 2))

                # Pheromone
                pheromone = pheromone_map[neighbor[1], neighbor[0]]

                # Probability
                probability = (pheromone ** alpha) * (heuristic ** beta)
                probabilities.append((neighbor, probability))
                total += probability

            if total == 0:
                # No valid path → choose randomly
                next_pos = random.choice(neighbors)
            else:
                # Roulette-wheel selection
                probabilities = [(pos, prob / total) for pos, prob in probabilities]
                probabilities.sort(key=lambda x: x[1], reverse=True)

                cumulative = 0.0
                rand = random.random()
                for pos, prob in probabilities:
                    cumulative += prob
                    if rand <= cumulative:
                        next_pos = pos
                        break
                else:
                    next_pos = probabilities[-1][0]

            self.position = next_pos
            self.path.append(next_pos)
            self.visited.add(next_pos)

            if self.position == self.goal:
                self.reached_goal = True

        def get_neighbors(self, pos):
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                          (-1, -1), (-1, 1), (1, -1), (1, 1)]
            neighbors = []

            for dx, dy in directions:
                nx, ny = pos[0] + dx, pos[1] + dy
                if (0 <= nx < self.grid_size and 0 <= ny < self.grid_size and
                        self.grid_map[ny, nx] == 0 and (nx, ny) not in self.visited):
                    neighbors.append((nx, ny))
            return neighbors

        def get_path_length(self):
            length = 0
            for i in range(len(self.path) - 1):
                dx = abs(self.path[i + 1][0] - self.path[i][0])
                dy = abs(self.path[i + 1][1] - self.path[i][1])
                length += math.sqrt(2) if dx > 0 and dy > 0 else 1
            return length

    def aco_algorithm(start, goal, grid_map, grid_size,
                      num_ants=50, max_iterations=100,
                      alpha=1.0, beta=2.0, evaporation=0.5, Q=100.0):

        # Initialize pheromone matrix
        pheromone_map = np.ones((grid_size, grid_size)) * 0.1

        best_path = None
        best_length = float('inf')

        for iteration in range(max_iterations):
            if rospy.is_shutdown():
                break

            ants = [Ant(start, goal, grid_map, grid_size) for _ in range(num_ants)]

            # Let all ants search paths
            for ant in ants:
                max_steps = 2 * grid_size  # Prevent infinite loops
                for step in range(max_steps):
                    if ant.reached_goal or rospy.is_shutdown():
                        break
                    ant.move(pheromone_map, alpha, beta)

            # Pheromone evaporation
            pheromone_map *= (1.0 - evaporation)

            # Update pheromone
            for ant in ants:
                if ant.reached_goal:
                    path_length = ant.get_path_length()

                    # Update best path
                    if path_length < best_length:
                        best_path = ant.path
                        best_length = path_length

                    # Release pheromone according to path quality
                    delta_pheromone = Q / path_length
                    for pos in ant.path:
                        pheromone_map[pos[1], pos[0]] += delta_pheromone

            # Early stop if path found
            if best_path and iteration > 10:
                break

        return best_path

    # Read parameters (outside loop)
    try:
        resources_path = rospy.get_param('resourcesPath')
    except KeyError:
        rospy.logerr("Parameter 'resourcesPath' not found on param server.")
        return

    # Support two parameter names: image_name or image_base_name (only pass 'demo01')
    base_image_name = rospy.get_param('image_name', 'map')  # Default: map.png
    image_name = f"{base_image_name}_{image_size}.png"  # Dynamically construct filename
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

    # Morphological preprocessing (ensure valid img_orig first)
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
            rospy.loginfo("Starting Ant Colony Optimization path planning...")
            size = img.shape
            h = size[0]
            w = size[1]
            grid_size = max(w, h)

            # Build obstacle list (integer coordinates, original code flips y)
            obstacle_list = []
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        # map image: (ix, iy) where iy from top; original code flips y: h - iy - 1
                        obstacle_list.append([ix, h - iy - 1])

            # Build grid map
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        grid_map[h - iy - 1, ix] = 1

            # Compute start and goal (consider boundary & type)
            start_point = (int(round(car_position[0] + image_size/2)), int(round(car_position[1] + image_size/2)))
            end_point = (int(round(target_point[0] + image_size/2)), int(round(target_point[1] + image_size/2)))

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

            # Check if goal lies inside obstacle
            if grid_map[end_point[1], end_point[0]] == 1:
                rospy.logerr("End point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Check if start lies inside obstacle
            if grid_map[start_point[1], start_point[0]] == 1:
                rospy.logerr("Start point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Run ant colony optimization
            path = aco_algorithm(start_point, end_point, grid_map, grid_size)

            if path is None:
                rospy.logerr("Ant Colony Optimization failed to find a path to the target!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Convert path to list format
            route_seq = [list(p) for p in path]

            # Extract key points (polar method) - keep original post-processing
            key_points = polar_key_points(route_seq, obstacle_list)

            # Convert key points back to -50..+50 coordinate range and publish
            route_list = []
            for kp in key_points:
                route_list.append([int(kp[0] - image_size/2), int(kp[1] - image_size/2)])

            rospy.loginfo("Ant Colony Optimization completed with %d points" % len(route_list))
            need_planning = False

        # Publish (even if empty)
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


# ------------------ Subscriber Thread ------------------
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
    rospy.init_node("improved_ant_colony_node")

    # Get parameters from ROS parameter server
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0

    # Start subscriber and ant colony threads
    Thread(target=subData, daemon=True).start()
    Thread(target=ant_colony_optimization, daemon=True).start()
    rospy.spin()

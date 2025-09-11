#!/usr/bin/env python3
import rospy
import cv2
import copy
import math
import os
import json
import numpy as np
import heapq

from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from threading import Thread

# Global variables
target_point = [0.0, 0.0]
need_planning = False
car_position = [0.0, 0.0]

# Get image size parameter from ROS param server and compute map ratio
image_size = rospy.get_param('image_size', 100)  # default value 100
# Map ratio - dynamically calculated from image_size
# image_size = 50, 100, 200 correspond to map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0


# ------------------ Utility Functions ------------------
def to_polar_list(points, origin):
    """Convert a set of (x,y) points into a list of (r, theta, idx) tuples"""
    out = []
    for idx, pt in enumerate(points):
        dx = float(pt[0]) - float(origin[0])
        dy = float(pt[1]) - float(origin[1])
        r = math.sqrt(dx * dx + dy * dy)
        theta = math.atan2(dy, dx)
        out.append((r, theta, idx))
    return out


def find_first_obstacle(A, B):
    """Find the first segment in polar sequence A that is blocked by polar set B"""
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
    """Extract key points from path using polar coordinate method"""
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
            coords = coords[obs_idx:]  # restart from that point
            polar_coords = to_polar_list(coords, coords[0])
            obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, coords[0])]
        else:
            break
    if coords:
        key_points.append(coords[-1])

    # remove duplicates while preserving order
    kp = []
    for p in key_points:
        if kp and kp[-1] == p:
            continue
        kp.append([int(p[0]), int(p[1])])
    return kp


# ------------------ D* Lite Algorithm Class ------------------
class DStarLite:
    def __init__(self, grid_map, start, goal):
        self.grid = grid_map
        self.size = grid_map.shape[0]
        self.start = start
        self.goal = goal

        self.U = []  # priority queue
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.back_pointers = {}

        for x in range(self.size):
            for y in range(self.size):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.rhs[self.goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def calculate_key(self, node):
        k1 = min(self.g[node], self.rhs[node]) + self.heuristic(self.start, node) + self.km
        k2 = min(self.g[node], self.rhs[node])
        return (k1, k2)

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)

    def get_neighbors(self, pos):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for dx, dy in directions:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < self.size and 0 <= ny < self.size:
                if self.grid[ny, nx] == 0:
                    result.append((nx, ny))
        return result

    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self.calculate_key(self.start) or
                          self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                for s in self.get_neighbors(u) + [u]:
                    self.update_vertex(s)

    def update_vertex(self, u):
        if u != self.goal:
            min_rhs = float('inf')
            for s in self.get_neighbors(u):
                cost = 1 if abs(u[0] - s[0]) + abs(u[1] - s[1]) == 1 else 1.414
                if self.g[s] + cost < min_rhs:
                    min_rhs = self.g[s] + cost
                    self.back_pointers[u] = s
            self.rhs[u] = min_rhs

        self.U = [(k, s) for (k, s) in self.U if s != u]
        heapq.heapify(self.U)

        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def get_path(self):
        path = [self.start]
        current = self.start

        while current != self.goal:
            if current in self.back_pointers:
                current = self.back_pointers[current]
                path.append(current)
            else:
                return None
        return path


# ------------------ D* Lite Main Logic ------------------
def D_star_lite():
    global need_planning, target_point, car_position, image_size, map_ratio

    try:
        resources_path = rospy.get_param('resourcesPath')
    except KeyError:
        rospy.logerr("Parameter 'resourcesPath' not found on param server.")
        return

    # support both 'image_name' and 'image_base_name' param
    base_name = rospy.get_param('image_name', None)
    if base_name is None:
        base_name = rospy.get_param('image_base_name', 'demo01')
    image_name = base_name
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
        rospy.logerr("Failed to load map image: %s" % image_path)
        return

    # morphological preprocessing
    kernel_size = 3 if image_size == 50 else 5
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    img = cv2.erode(img_orig, kernel, iterations=1)

    # publisher
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    rate = rospy.Rate(10)
    route_list = []
    route_list_msg = String()

    while not rospy.is_shutdown():
        if need_planning:
            rospy.loginfo("Starting D* Lite path planning...")
            size = img.shape
            h = size[0]
            w = size[1]
            grid_size = image_size

            # build obstacle list (flip y to match coordinate system)
            obstacle_list = []
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        obstacle_list.append([ix, h - iy - 1])

            # build grid map
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            for x, y in obstacle_list:
                if 0 <= x < grid_size and 0 <= y < grid_size:
                    grid_map[y, x] = 1

            # compute start and goal (shifted by image_size/2)
            start = (int(round(car_position[0] + image_size / 2)), int(round(car_position[1] + image_size / 2)))
            goal = (int(round(target_point[0] + image_size / 2)), int(round(target_point[1] + image_size / 2)))

            rospy.loginfo("Start (img coords): %s , End (img coords): %s" % (str(start), str(goal)))

            def in_bounds(pt):
                return 0 <= pt[0] < grid_size and 0 <= pt[1] < grid_size

            if not in_bounds(start) or not in_bounds(goal):
                rospy.logerr("Start or End out of bounds. Abort planning.")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            if grid_map[goal[1], goal[0]] == 1:
                rospy.logerr("End point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # run D* Lite
            dstar = DStarLite(grid_map, start, goal)
            dstar.compute_shortest_path()
            path = dstar.get_path()

            if path is None:
                rospy.logwarn("No path found by D* Lite!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # convert to list
            route_seq = []
            for p in path:
                route_seq.append([p[0], p[1]])

            # extract key points
            key_points = polar_key_points(route_seq, obstacle_list)

            # convert back to -size/2..+size/2 coords
            route_list = []
            for kp in key_points:
                route_list.append([int(kp[0] - image_size / 2), int(kp[1] - image_size / 2)])

            rospy.loginfo("D* Lite path planning completed with %d points" % len(route_list))
            need_planning = False

        # publish (even if empty)
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


# ------------------ Entry ------------------
if __name__ == "__main__":
    rospy.init_node("improved_d_star_lite_node")

    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"D* Lite started with image_size: {image_size}, map_ratio: {map_ratio}")

    Thread(target=subData, daemon=True).start()
    Thread(target=D_star_lite, daemon=True).start()
    rospy.spin()

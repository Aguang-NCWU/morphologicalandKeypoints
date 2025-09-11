import rospy
import cv2
import copy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point
from threading import Thread
from nav_msgs.msg import Odometry
import json
import os
import numpy as np
import heapq

# Global variable for storing the target point
target_point = [0, 0]
# Flag indicating whether path planning is required
need_planning = False
# Global variable for storing the car's position
car_position = [0, 0]

# Get image size parameter from ROS parameter server and calculate map ratio
image_size = rospy.get_param('image_size', 100)  # Default value is 100
# Map ratio - dynamically calculated based on image_size
# image_size of 50, 100, 200 corresponds to map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0

# -------------------------------
# D* Lite Algorithm Class (extracted from Dlite original obstacle avoidance.py)
# -------------------------------
class DStarLite:
    def __init__(self, grid_map, start, goal):
        self.grid = grid_map
        self.size = grid_map.shape[0]
        self.start = start
        self.goal = goal

        self.U = []  # Priority queue
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

# -------------------------------
# Obstacle push post-processing function (extracted from Dlite original obstacle avoidance.py)
# -------------------------------
def apply_obstacle_push(path, obstacles, influence_radius=5, max_force=2):
    adjusted_path = [path[0]]  # Keep start point
    for point in path[1:-1]:  # Keep start and goal unchanged
        px, py = point
        force_x, force_y = 0.0, 0.0
        for ox, oy in obstacles:
            dx = px - ox
            dy = py - oy
            distance = np.hypot(dx, dy)
            if 0 < distance < influence_radius:
                angle = np.arctan2(dy, dx)
                f = (1.0 / distance) * 4
                force_x += f * np.cos(angle)
                force_y += f * np.sin(angle)

        norm = np.hypot(force_x, force_y)
        if norm > max_force:
            force_x *= max_force / norm
            force_y *= max_force / norm

        new_px = int(round(px + force_x))
        new_py = int(round(py + force_y))

        # Ensure new point is valid
        grid_size = 100  # Assume map size is 100x100
        new_px = max(0, min(grid_size - 1, new_px))
        new_py = max(0, min(grid_size - 1, new_py))

        adjusted_path.append((new_px, new_py))

    adjusted_path.append(path[-1])  # Keep goal point
    return adjusted_path

# -------------------------------
# Path planning main function (using D* Lite)
# -------------------------------
def D_star_lite_planning():
    global need_planning, target_point, car_position, image_size, map_ratio

    # Get resource path and image name from parameter server
    resources_path = rospy.get_param('resourcesPath')
    base_image_name = rospy.get_param('image_name', 'map')  # Default is map.png
    image_name = f"{base_image_name}_{image_size}.png"  # Dynamically build image filename
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: " + image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    # Publisher for the route sequence
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    # Load map as grayscale image
    img = cv2.imread(image_path, 0)
    if img is None:
        rospy.logerr("Failed to load map image: " + image_path)
        return

    # Loop frequency
    rate = rospy.Rate(10)
    # Path list
    route_list = []
    # Path list as string
    route_list_msg = String()

    # Main loop
    while not rospy.is_shutdown():
        # If planning is needed
        if need_planning:
            rospy.loginfo("Starting D* Lite path planning...")

            # Get map dimensions
            h, w = img.shape
            grid_size = image_size  # Use dynamic image_size
            # Create binary obstacle map
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            for x in range(w):
                for y in range(h):
                    if img[y, x] == 0:  # Black pixels are obstacles
                        grid_map[grid_size - y - 1, x] = 1  # Note coordinate transformation

            # Set start and goal (note coordinate transformation) - use image_size instead of hard-coded 50
            start = (round(car_position[0] + image_size / 2), round(car_position[1] + image_size / 2))
            goal = (int(target_point[0] + image_size / 2), int(target_point[1] + image_size / 2))

            # Check if goal is inside obstacle
            if grid_map[goal[1], goal[0]] == 1:
                rospy.logwarn("Goal point is inside an obstacle!")
                need_planning = False
                continue

            # Initialize D* Lite
            dstar = DStarLite(grid_map, start, goal)
            dstar.compute_shortest_path()
            path = dstar.get_path()

            if path is None:
                rospy.logwarn("No path found by D* Lite!")
                need_planning = False
                continue

            # Collect obstacle coordinates (for push processing)
            obstacles = []
            for x in range(grid_size):
                for y in range(grid_size):
                    if grid_map[y, x] == 1:
                        obstacles.append((x, y))

            # Apply obstacle push post-processing
            path = apply_obstacle_push(path, obstacles)

            # Convert coordinates (subtract image_size/2 to match original coordinate system)
            route_list_ = []
            for p in path:
                route_list_.append([p[0] - image_size / 2, p[1] - image_size / 2])

            # Remove duplicate points (optional)
            end = False
            i = 0
            q = len(route_list_) - 1
            while not end:
                for i in range(0, q):
                    if route_list_[i][0] == route_list_[q][0] and route_list_[i][1] == route_list_[q][1]:
                        route_list_ = route_list_[: i] + route_list_[q:]
                        q = i
                        break
                    if i == q - 1:
                        q = i
                if q == 0:
                    end = True

            route_list = route_list_
            rospy.loginfo("D* Lite path planning completed with " + str(len(route_list)) + " points")
            need_planning = False

        # Convert path list to JSON string and publish
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()

# -------------------------------
# Subscription callback functions (unchanged)
# -------------------------------
def subData():
    def getTargetPoint(msg_t):
        global target_point, need_planning, map_ratio
        if msg_t.x != target_point[0] or msg_t.y != target_point[1]:
            target_point[0] = msg_t.x
            target_point[1] = msg_t.y
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    def getCarPosition(msg_p):
        global car_position, map_ratio
        # Coordinate transformation using dynamically calculated map_ratio
        car_position = [msg_p.pose.pose.position.x * map_ratio, msg_p.pose.pose.position.y * map_ratio]

    target_point_sub = rospy.Subscriber("/target_point", Point, getTargetPoint)
    car_position_sub = rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()

# -------------------------------
# Main function
# -------------------------------
if __name__ == "__main__":
    rospy.init_node("D_star_lite_path_planning")

    # Reload image_size parameter and recalculate map ratio
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"D* Lite started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Run subscription and planning in separate threads
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    planning_thread = Thread(target=D_star_lite_planning)
    planning_thread.daemon = True
    planning_thread.start()

    rospy.spin()

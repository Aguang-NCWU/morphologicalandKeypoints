import rospy
import cv2
import copy
import math
import random
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from threading import Thread
from nav_msgs.msg import Odometry
import json
import os

# Global variable: store target point
target_point = [0, 0]
# Flag indicating whether path planning is required
need_planning = False
# Global variable: store car position
car_position = [0, 0]

# Get image size parameter from ROS parameter server and calculate map scale
image_size = rospy.get_param('image_size', 100)  # default is 100
# Map scale - dynamically calculated from image_size
# image_size values 50, 100, 200 correspond to map_ratio = 2.5, 5, 10 respectively
map_ratio = float(image_size) / 20.0


# Ant Colony Optimization (ACO) path planning function
def ant_colony_optimization():
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

            # Calculate transition probability
            probabilities = []
            total = 0.0

            for neighbor in neighbors:
                if neighbor in self.visited:
                    continue

                # Heuristic information (inverse of distance to goal)
                heuristic = 1.0 / (
                            1.0 + math.sqrt((neighbor[0] - self.goal[0]) ** 2 + (neighbor[1] - self.goal[1]) ** 2))

                # Pheromone value
                pheromone = pheromone_map[neighbor[1], neighbor[0]]

                # Probability calculation
                probability = (pheromone ** alpha) * (heuristic ** beta)
                probabilities.append((neighbor, probability))
                total += probability

            if total == 0:
                # If no feasible path, choose randomly
                next_pos = random.choice(neighbors)
            else:
                # Roulette wheel selection
                probabilities = [(pos, prob / total) for pos, prob in probabilities]
                probabilities.sort(key=lambda x: x[1], reverse=True)

                # Cumulative probability
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

            # Each ant searches for a path
            for ant in ants:
                max_steps = 2 * grid_size  # prevent infinite loops
                for step in range(max_steps):
                    if ant.reached_goal or rospy.is_shutdown():
                        break
                    ant.move(pheromone_map, alpha, beta)

            # Pheromone evaporation
            pheromone_map *= (1.0 - evaporation)

            # Update pheromones
            for ant in ants:
                if ant.reached_goal:
                    path_length = ant.get_path_length()

                    # Update best path
                    if path_length < best_length:
                        best_path = ant.path
                        best_length = path_length

                    # Release pheromones proportional to path quality
                    delta_pheromone = Q / path_length
                    for pos in ant.path:
                        pheromone_map[pos[1], pos[0]] += delta_pheromone

            # Stop early if a path is found
            if best_path and iteration > 10:
                break

        return best_path

    global need_planning, target_point, car_position, image_size, map_ratio

    # Get resource path and image name from parameter server
    resources_path = rospy.get_param('resourcesPath')
    base_image_name = rospy.get_param('image_name', 'map')  # default: map.png
    image_name = f"{base_image_name}_{image_size}.png"  # dynamically construct image name
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: " + image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    # Publisher for route list
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    # Load map as grayscale
    img = cv2.imread(image_path, 0)
    if img is None:
        rospy.logerr("Failed to load map image: " + image_path)
        return

    # Loop frequency
    rate = rospy.Rate(10)
    # Path list
    route_list = []
    # Path list in string format
    route_list_msg = String()

    # Main loop
    while not rospy.is_shutdown():
        # Start path planning if required
        if need_planning:
            rospy.loginfo("Starting Ant Colony Optimization path planning...")
            # Get map dimensions
            size = img.shape
            w = size[1]  # width
            h = size[0]  # height
            grid_size = max(w, h)

            # Create grid map
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            for i in range(0, w):
                for q in range(0, h):
                    # Obstacles marked as 1
                    if img[q, i] == 0:
                        grid_map[h - q - 1, i] = 1

            # Define start and goal points (with coordinate transformation)
            start_point = (round(car_position[0] + image_size / 2), round(car_position[1] + image_size / 2))
            end_point = (int(target_point[0] + image_size / 2), int(target_point[1] + image_size / 2))

            # Boundary check
            def in_bounds(point):
                return 0 <= point[0] < grid_size and 0 <= point[1] < grid_size

            if not in_bounds(start_point) or not in_bounds(end_point):
                rospy.logerr("Start or end point out of bounds!")
                need_planning = False
                continue

            # Check if goal is inside an obstacle
            if grid_map[end_point[1], end_point[0]] == 1:
                rospy.logerr("End point is in obstacle!")
                need_planning = False
                continue

            # Check if start is inside an obstacle
            if grid_map[start_point[1], start_point[0]] == 1:
                rospy.logerr("Start point is in obstacle!")
                need_planning = False
                continue

            # Run ACO algorithm
            path = aco_algorithm(start_point, end_point, grid_map, grid_size)

            if path is None:
                rospy.logerr("Ant Colony Optimization failed to find a path to the target!")
                need_planning = False
                continue

            # Path adjustment: keep away from obstacles (original feature retained)
            obstacle_list = []
            for i in range(0, w):
                for q in range(0, h):
                    if img[q, i] == 0:
                        obstacle_list.append([i, h - q - 1])

            route_list_ = [list(p) for p in path]
            route_list_length = len(route_list_)

            for i in range(1, route_list_length):
                if i != route_list_length - 1:
                    force_vector = [0, 0]
                    # Compute influence of each obstacle
                    for q in range(0, len(obstacle_list)):
                        l = math.sqrt((obstacle_list[q][0] - route_list_[i][0]) ** 2 +
                                      (obstacle_list[q][1] - route_list_[i][1]) ** 2)
                        # Only consider nearby obstacles
                        if l < 5:
                            a = math.atan2(route_list_[i][1] - obstacle_list[q][1],
                                           route_list_[i][0] - obstacle_list[q][0])
                            force_vector[0] += (1 / l) * 4 * math.cos(a)
                            force_vector[1] += (1 / l) * 4 * math.sin(a)

                    # Force tuning
                    l_ = math.sqrt(force_vector[0] ** 2 + force_vector[1] ** 2)
                    if l_ > 2:
                        force_vector[0] *= 2 / l_
                        force_vector[1] *= 2 / l_

                    # Apply repulsive force
                    route_list_[i][0] += int(force_vector[0])
                    route_list_[i][1] += int(force_vector[1])

            # Remove duplicate points
            end = False
            i = 0
            q = len(route_list_) - 1
            while not end:
                for i in range(0, q):
                    if route_list_[i][0] == route_list_[q][0] and route_list_[i][1] == route_list_[q][1]:
                        route_list_ = route_list_[:i] + route_list_[q:]
                        q = i
                        break
                    if i == q - 1:
                        q = i
                if q == 0:
                    end = True

            # Convert coordinates and store path
            route_list = []
            for point in route_list_:
                route_list.append([point[0] - image_size / 2, point[1] - image_size / 2])

            rospy.loginfo("Ant Colony Optimization completed with " + str(len(route_list)) + " points")
            # Reset path planning flag
            need_planning = False

        # Convert path list to JSON string and publish
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


def subData():
    # Callback for receiving target point
    def getTargetPoint(msg_t):
        global target_point, need_planning, map_ratio
        # Update only if new target differs from the previous one
        if msg_t.x != target_point[0] or msg_t.y != target_point[1]:
            target_point[0] = msg_t.x
            target_point[1] = msg_t.y
            # Set path planning flag
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    # Callback for receiving car position
    def getCarPosition(msg_p):
        global car_position, map_ratio
        # Coordinate transformation using dynamically calculated map_ratio
        car_position = [msg_p.pose.pose.position.x * map_ratio, msg_p.pose.pose.position.y * map_ratio]

    target_point_sub = rospy.Subscriber("/target_point", Point, getTargetPoint)
    car_position_sub = rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ant_colony_path_planning")

    # Get image size again from parameter server and recompute map ratio
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"Ant Colony Optimization started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Run subscriber and path planning in separate threads
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    aco_thread = Thread(target=ant_colony_optimization)
    aco_thread.daemon = True
    aco_thread.start()

    # No need to join, let main thread continue
    rospy.spin()

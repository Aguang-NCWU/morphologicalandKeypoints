import rospy
import cv2
import copy
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point
from threading import Thread
from nav_msgs.msg import Odometry
import json
import os
import numpy as np

# Global variables to store the target point and car position
target_point = [0, 0]  # Target position (x, y)
need_planning = False   # Flag indicating whether path planning is needed
car_position = [0, 0]   # Car position (x, y)

# Get image size parameter from ROS parameter server and calculate map scaling
image_size = rospy.get_param('image_size', 100)  # Default to 100 if not set
map_ratio = float(image_size) / 20.0  # Map scaling factor (image_size=50,100,200 → map_ratio=2.5,5,10)


# RRT (Rapidly-exploring Random Tree) path planning function
def RRT_planner():
    # Compute Euclidean distance between two points
    def distance(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Check if a point is free (not an obstacle) in the grid map
    def is_free(point, grid_map, grid_size):
        x, y = point
        if 0 <= x < grid_size and 0 <= y < grid_size:
            return grid_map[y, x] == 0
        return False

    # Check whether the straight line between two points collides with obstacles
    def line_collision_check(p1, p2, grid_map, grid_size):
        # Use Bresenham's line algorithm
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

    # Find the nearest node in the tree to a random point
    def nearest_node(nodes, random_point):
        min_dist = float('inf')
        nearest = None
        for node in nodes:
            dist = distance(node['point'], random_point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    # Steer from a node towards a target point with a step size
    def steer(from_node, to_point, step_size=5):
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

    # RRT main algorithm
    def rrt_algorithm(start, goal, grid_map, grid_size, max_iter=5000, step_size=5, goal_sample_rate=0.1):
        nodes = [{'point': start, 'parent': None}]

        for i in range(max_iter):
            # Randomly sample the goal point with some probability
            if random.random() < goal_sample_rate:
                rnd_point = goal
            else:
                rnd_point = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))

            nearest = nearest_node(nodes, rnd_point)
            new_point = steer(nearest, rnd_point, step_size)

            # Check if new point is valid and collision-free
            if is_free(new_point, grid_map, grid_size) and line_collision_check(nearest['point'], new_point, grid_map, grid_size):
                new_node = {'point': new_point, 'parent': nearest}
                nodes.append(new_node)

                # Check if goal is reached
                if distance(new_point, goal) <= step_size:
                    if line_collision_check(new_point, goal, grid_map, grid_size):
                        goal_node = {'point': goal, 'parent': new_node}
                        nodes.append(goal_node)
                        # Trace back the path
                        path = []
                        node = goal_node
                        while node:
                            path.append(node['point'])
                            node = node['parent']
                        return path[::-1]  # Return path from start to goal

        rospy.logwarn("RRT failed to find path within max iterations")
        return None

    global need_planning, target_point, car_position, image_size, map_ratio

    # Load map image
    resources_path = rospy.get_param('resourcesPath')
    base_image_name = rospy.get_param('image_name', 'map')
    image_name = f"{base_image_name}_{image_size}.png"
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: " + image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    # Load map as grayscale image
    img = cv2.imread(image_path, 0)
    if img is None:
        rospy.logerr("Failed to load map image: " + image_path)
        return

    rate = rospy.Rate(10)
    route_list = []
    route_list_msg = String()

    while not rospy.is_shutdown():
        if need_planning:
            rospy.loginfo("Starting RRT path planning...")

            w, h = img.shape[1], img.shape[0]
            grid_size = image_size  # Use dynamic image size

            # Create grid map and obstacle list
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            obstacle_list = []
            for i in range(w):
                for q in range(h):
                    if img[q, i] == 0:  # Obstacle
                        y_coord = h - q - 1
                        grid_map[y_coord, i] = 1
                        obstacle_list.append([i, y_coord])

            # Convert start and goal coordinates
            start_point = (round(car_position[0] + image_size / 2), round(car_position[1] + image_size / 2))
            end_point = (int(target_point[0] + image_size / 2), int(target_point[1] + image_size / 2))

            # Check boundaries
            def in_bounds(point):
                return 0 <= point[0] < grid_size and 0 <= point[1] < grid_size

            if not in_bounds(start_point) or not in_bounds(end_point):
                rospy.logerr("Start or end point out of bounds!")
                need_planning = False
                continue

            # Check if goal is inside obstacle
            if not is_free(end_point, grid_map, grid_size):
                rospy.logerr("End point is in obstacle!")
                need_planning = False
                continue

            # Run RRT algorithm
            path = rrt_algorithm(start_point, end_point, grid_map, grid_size)

            if path is None:
                rospy.logerr("RRT failed to find a path to the target!")
                need_planning = False
                continue

            # Apply repulsive force to keep path away from obstacles
            route_list_ = [list(p) for p in path]
            for i in range(1, len(route_list_)):
                if i != len(route_list_) - 1:
                    force_vector = [0, 0]
                    for obs in obstacle_list:
                        l = math.sqrt((obs[0] - route_list_[i][0]) ** 2 + (obs[1] - route_list_[i][1]) ** 2)
                        if l < 5:  # Only consider nearby obstacles
                            a = math.atan2(route_list_[i][1] - obs[1], route_list_[i][0] - obs[0])
                            force_vector[0] += (1 / l) * 4 * math.cos(a)
                            force_vector[1] += (1 / l) * 4 * math.sin(a)
                    # Limit force magnitude
                    l_ = math.sqrt(force_vector[0] ** 2 + force_vector[1] ** 2)
                    if l_ > 2:
                        force_vector[0] *= 2 / l_
                        force_vector[1] *= 2 / l_
                    route_list_[i][0] += int(force_vector[0])
                    route_list_[i][1] += int(force_vector[1])
                    # Keep inside map bounds
                    route_list_[i][0] = max(0, min(grid_size - 1, route_list_[i][0]))
                    route_list_[i][1] = max(0, min(grid_size - 1, route_list_[i][1]))

            # Remove duplicate points
            end = False
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

            # Convert coordinates back to original map reference
            route_list = [[p[0] - image_size / 2, p[1] - image_size / 2] for p in route_list_]

            rospy.loginfo("RRT path planning completed with " + str(len(route_list)) + " points")
            need_planning = False

        # Publish path as JSON string
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


# Subscribe to ROS topics for target point and car position
def subData():
    def getTargetPoint(msg_t):
        global target_point, need_planning
        # Update target point only if changed
        if msg_t.x != target_point[0] or msg_t.y != target_point[1]:
            target_point[0] = msg_t.x
            target_point[1] = msg_t.y
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    def getCarPosition(msg_p):
        global car_position
        # Convert odometry to map coordinates using scaling factor
        car_position = [msg_p.pose.pose.position.x * map_ratio, msg_p.pose.pose.position.y * map_ratio]

    rospy.Subscriber("/target_point", Point, getTargetPoint)
    rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("RRT_path_planning")

    # Update map_ratio based on parameter
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"RRT started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Start threads for subscribing and planning
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    RRT_thread = Thread(target=RRT_planner)
    RRT_thread.daemon = True
    RRT_thread.start()

    rospy.spin()

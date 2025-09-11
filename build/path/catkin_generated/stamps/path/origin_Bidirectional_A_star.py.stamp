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
from heapq import heappop, heappush
import numpy as np

# Global variable to store target point
target_point = [0, 0]
# Flag to indicate whether path planning is needed
need_planning = False
# Global variable to store car position
car_position = [0, 0]

# Get image size parameter from ROS parameter server and calculate map ratio
image_size = rospy.get_param('image_size', 100)  # Default value is 100
# Map ratio - dynamically calculated based on image_size
# When image_size = 50, 100, 200 → map_ratio = 2.5, 5, 10 respectively
map_ratio = float(image_size) / 20.0


# Bidirectional A* path planning function
def bidirectional_a_star():
    # Node class definition
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

    # Reconstruct path
    def reconstruct_path(meet_node_f, meet_node_b):
        path_f = []
        node = meet_node_f
        while node:
            path_f.append(node.position)
            node = node.parent
        path_f = path_f[::-1]

        path_b = []
        node = meet_node_b.parent  # avoid duplication
        while node:
            path_b.append(node.position)
            node = node.parent

        return path_f + path_b

    # Core algorithm of Bidirectional A*
    def bidirectional_a_star_algorithm(start, goal, grid_map, grid_size):
        open_f = []  # Forward open list
        open_b = []  # Backward open list
        closed_f = {}  # Forward closed list
        closed_b = {}  # Backward closed list

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

            # Check if paths meet
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

                # Check if already in open list; update if a better path is found
                found = False
                for i, (_, node) in enumerate(open_f):
                    if node.position == neighbor:
                        found = True
                        if g < node.g:
                            open_f[i] = (new_node.f, new_node)
                            # Re-heapify
                            open_f.sort(key=lambda x: x[0])
                        break

                if not found:
                    heappush(open_f, (new_node.f, new_node))

            # Backward expansion
            for neighbor in get_neighbors(current_b.position, grid_map, grid_size):
                if neighbor in closed_b:
                    continue
                # Movement cost
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

    global need_planning, target_point, car_position, image_size, map_ratio

    # Get resource path and image name from ROS parameter server
    resources_path = rospy.get_param('resourcesPath')
    base_image_name = rospy.get_param('image_name', 'map')  # Default: map.png
    image_name = f"{base_image_name}_{image_size}.png"  # Construct dynamic filename
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: " + image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    # Publisher for path sequence
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    # Load map as grayscale image
    img = cv2.imread(image_path, 0)
    if img is None:
        rospy.logerr("Failed to load map image: " + image_path)
        return

    # Loop rate
    rate = rospy.Rate(10)
    # Path list
    route_list = []
    # Path list in string format
    route_list_msg = String()

    # Main loop
    while not rospy.is_shutdown():
        # If path planning is required, start planning
        if need_planning:
            rospy.loginfo("Starting bidirectional A* path planning...")
            # Get map width and height
            size = img.shape
            w = size[1]  # Width
            h = size[0]  # Height
            grid_size = max(w, h)

            # Create grid map
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            for i in range(0, w):
                for q in range(0, h):
                    # Obstacles marked as 1
                    if img[q, i] == 0:
                        grid_map[h - q - 1, i] = 1

            # Set start and goal points (with coordinate transformation)
            start_point = (round(car_position[0] + image_size / 2), round(car_position[1] + image_size / 2))
            end_point = (int(target_point[0] + image_size / 2), int(target_point[1] + image_size / 2))

            # Boundary check
            def in_bounds(point):
                return 0 <= point[0] < grid_size and 0 <= point[1] < grid_size

            if not in_bounds(start_point) or not in_bounds(end_point):
                rospy.logerr("Start or end point out of bounds!")
                need_planning = False
                continue

            # Check if goal point is inside an obstacle
            if grid_map[end_point[1], end_point[0]] == 1:
                rospy.logerr("End point is in obstacle!")
                need_planning = False
                continue

            # Check if start point is inside an obstacle
            if grid_map[start_point[1], start_point[0]] == 1:
                rospy.logerr("Start point is in obstacle!")
                need_planning = False
                continue

            # Run bidirectional A* algorithm
            path = bidirectional_a_star_algorithm(start_point, end_point, grid_map, grid_size)

            if path is None:
                rospy.logerr("Bidirectional A* failed to find a path to the target!")
                need_planning = False
                continue

            # Move path away from obstacles
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
                    # Calculate obstacle influence on each path point
                    for q in range(0, len(obstacle_list)):
                        l = math.sqrt((obstacle_list[q][0] - route_list_[i][0]) ** 2 +
                                      (obstacle_list[q][1] - route_list_[i][1]) ** 2)
                        # Only consider obstacles within a certain range
                        if l < 5:
                            a = math.atan2(route_list_[i][1] - obstacle_list[q][1],
                                           route_list_[i][0] - obstacle_list[q][0])
                            force_vector[0] += (1 / l) * 4 * math.cos(a)
                            force_vector[1] += (1 / l) * 4 * math.sin(a)

                    # Force adjustment
                    l_ = math.sqrt(force_vector[0] ** 2 + force_vector[1] ** 2)
                    if l_ > 2:
                        force_vector[0] *= 2 / l_
                        force_vector[1] *= 2 / l_

                    # Apply force
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

            rospy.loginfo("Bidirectional A* path planning completed with " + str(len(route_list)) + " points")
            # Reset planning flag
            need_planning = False

        # Convert path list to JSON string and publish
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


def subData():
    # Callback function for receiving target point
    def getTargetPoint(msg_t):
        global target_point, need_planning, map_ratio
        # Update only if the new target point is different from the previous one
        if msg_t.x != target_point[0] or msg_t.y != target_point[1]:
            target_point[0] = msg_t.x
            target_point[1] = msg_t.y
            # Trigger path planning
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    # Callback function for receiving car position
    def getCarPosition(msg_p):
        global car_position, map_ratio
        # Coordinate transformation using dynamic map_ratio
        car_position = [msg_p.pose.pose.position.x * map_ratio, msg_p.pose.pose.position.y * map_ratio]

    target_point_sub = rospy.Subscriber("/target_point", Point, getTargetPoint)
    car_position_sub = rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("bidirectional_a_star_path_planning")

    # Get image_size parameter again and recalculate map_ratio
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"Bidirectional A* started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Multithreading for receiving messages and path planning
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    bidirectional_a_star_thread = Thread(target=bidirectional_a_star)
    bidirectional_a_star_thread.daemon = True
    bidirectional_a_star_thread.start()

    # No join required, keep main thread running
    rospy.spin()

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

# -------------------------------
# Global variables
# -------------------------------
target_point = [0, 0]  # Target coordinates
need_planning = False  # Flag indicating whether path planning is needed
car_position = [0, 0]  # Robot current position

# Get image size from ROS parameter server and compute map ratio
image_size = rospy.get_param('image_size', 100)  # Default value is 100
# Map scale ratio, e.g., image_size 50, 100, 200 correspond to map_ratio 2.5, 5, 10
map_ratio = float(image_size) / 20.0

# -------------------------------
# JPS (Jump Point Search) Path Planning Function
# -------------------------------
def JPS():
    # Heuristic function: Chebyshev distance
    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)

    # Get possible neighbor directions for a given node
    def get_neighbors(pos, grid_map, grid_size):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for dx, dy in directions:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < grid_size and 0 <= ny < grid_size:
                if grid_map[ny, nx] == 0:  # 0 means free cell
                    result.append((dx, dy))
        return result

    # Jump function: the core of JPS algorithm
    def jump(current, dx, dy, goal, grid_map, grid_size):
        x, y = current
        x += dx
        y += dy
        if not (0 <= x < grid_size and 0 <= y < grid_size):
            return None
        if grid_map[y, x] == 1:  # Obstacle
            return None
        if (x, y) == goal:
            return (x, y)

        # Check for forced neighbors during diagonal movement
        if dx != 0 and dy != 0:
            if (grid_map[y - dy, x] == 1 and grid_map[y - dy, x - dx] == 0) or \
               (grid_map[y, x - dx] == 1 and grid_map[y - dy, x - dx] == 0):
                return (x, y)
        else:
            # Horizontal movement forced neighbor check
            if dx != 0:
                if (grid_map[y + 1, x] == 1 and grid_map[y + 1, x - dx] == 0) or \
                   (grid_map[y - 1, x] == 1 and grid_map[y - 1, x - dx] == 0):
                    return (x, y)
            # Vertical movement forced neighbor check
            else:
                if (grid_map[y, x + 1] == 1 and grid_map[y - dy, x + 1] == 0) or \
                   (grid_map[y, x - 1] == 1 and grid_map[y - dy, x - 1] == 0):
                    return (x, y)

        # Diagonal jump continues horizontally or vertically
        if dx != 0 and dy != 0:
            if jump((x, y), dx, 0, goal, grid_map, grid_size) or \
               jump((x, y), 0, dy, goal, grid_map, grid_size):
                return (x, y)

        return jump((x, y), dx, dy, goal, grid_map, grid_size)

    # Apply repulsive force to avoid obstacles (post-processing)
    def apply_repulsive_force(path, obstacle_list, grid_size):
        if len(path) <= 2:
            return path

        route_list_ = copy.deepcopy(path)
        for i in range(1, len(route_list_) - 1):  # Skip start and goal
            force_vector = [0, 0]
            for obs in obstacle_list:
                dx = obs[0] - route_list_[i][0]
                dy = obs[1] - route_list_[i][1]
                dist = math.hypot(dx, dy)
                if dist < 5:  # Only consider obstacles within influence radius
                    angle = math.atan2(-dy, -dx)
                    force_vector[0] += (1 / dist) * 4 * math.cos(angle)
                    force_vector[1] += (1 / dist) * 4 * math.sin(angle)

            # Limit maximum repulsive force
            magnitude = math.hypot(force_vector[0], force_vector[1])
            if magnitude > 2:
                force_vector[0] *= 2 / magnitude
                force_vector[1] *= 2 / magnitude

            # Apply force
            route_list_[i][0] += int(force_vector[0])
            route_list_[i][1] += int(force_vector[1])

            # Keep inside grid bounds
            route_list_[i][0] = max(0, min(grid_size - 1, route_list_[i][0]))
            route_list_[i][1] = max(0, min(grid_size - 1, route_list_[i][1]))

        return route_list_

    # Remove duplicate points from path
    def remove_duplicate_points(path):
        if len(path) <= 1:
            return path
        route_list_ = copy.deepcopy(path)
        q = len(route_list_) - 1
        end = False
        while not end and q > 0:
            for i in range(0, q):
                if route_list_[i][0] == route_list_[q][0] and route_list_[i][1] == route_list_[q][1]:
                    route_list_ = route_list_[:i] + route_list_[q:]
                    q = i
                    break
                if i == q - 1:
                    q = i
            if q == 0:
                end = True
        return route_list_

    global need_planning, target_point, car_position, image_size, map_ratio

    # Load map image from resources path
    resources_path = rospy.get_param('resourcesPath')
    base_image_name = rospy.get_param('image_name', 'map')
    image_name = f"{base_image_name}_{image_size}.png"
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo(f"Loading map from: {image_path}, Image size: {image_size}, Map ratio: {map_ratio}")

    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)

    # Load grayscale map
    img = cv2.imread(image_path, 0)
    if img is None:
        rospy.logerr(f"Failed to load map image: {image_path}")
        return

    rate = rospy.Rate(10)
    route_list = []
    route_list_msg = String()

    # Main loop
    while not rospy.is_shutdown():
        if need_planning:
            rospy.loginfo("Starting JPS path planning...")
            w, h = img.shape[1], img.shape[0]
            grid_size = max(w, h)

            # Initialize grid map and obstacle list
            grid_map = np.zeros((grid_size, grid_size), dtype=int)
            obstacle_list = []
            for i in range(w):
                for q in range(h):
                    if img[q, i] == 0:
                        grid_map[h - q - 1, i] = 1
                        obstacle_list.append([i, h - q - 1])

            # Set start and goal points (coordinate conversion)
            start_point = (round(car_position[0] + image_size / 2), round(car_position[1] + image_size / 2))
            end_point = (int(target_point[0] + image_size / 2), int(target_point[1] + image_size / 2))

            # -------------------------------
            # JPS Implementation
            # -------------------------------
            open_list = []
            closed_set = set()

            class Node:
                def __init__(self, position, g, h, parent=None):
                    self.position = position
                    self.g = g
                    self.h = h
                    self.f = g + h
                    self.parent = parent

                def __lt__(self, other):
                    return self.f < other.f

            start_node = Node(start_point, 0, heuristic(start_point, end_point))
            heappush(open_list, (start_node.f, start_node))
            path_found = False

            while open_list and not rospy.is_shutdown():
                _, current = heappop(open_list)
                if current.position == end_point:
                    # Backtrack path
                    path = []
                    temp = current
                    while temp:
                        path.append(list(temp.position))
                        temp = temp.parent
                    path = path[::-1]
                    path_found = True
                    break

                closed_set.add(current.position)
                for dx, dy in get_neighbors(current.position, grid_map, grid_size):
                    jp = jump(current.position, dx, dy, end_point, grid_map, grid_size)
                    if jp and jp not in closed_set:
                        new_g = current.g + heuristic(current.position, jp)
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

            if not path_found:
                rospy.logerr("JPS failed to find a path to the target!")
                need_planning = False
                continue

            # Apply repulsive force to path
            path = apply_repulsive_force(path, obstacle_list, grid_size)
            # Remove duplicate points
            path = remove_duplicate_points(path)
            # Convert coordinates back to original system
            route_list = [[p[0] - image_size / 2, p[1] - image_size / 2] for p in path]

            rospy.loginfo(f"JPS path planning completed with {len(route_list)} points")
            need_planning = False

        # Publish path as JSON string
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()

# -------------------------------
# ROS Subscribers
# -------------------------------
def subData():
    def getTargetPoint(msg_t):
        global target_point, need_planning
        if msg_t.x != target_point[0] or msg_t.y != target_point[1]:
            target_point[0] = msg_t.x
            target_point[1] = msg_t.y
            need_planning = True
            rospy.loginfo(f"New target point received: {target_point}")

    def getCarPosition(msg_p):
        global car_position
        car_position = [msg_p.pose.pose.position.x * map_ratio,
                        msg_p.pose.pose.position.y * map_ratio]

    rospy.Subscriber("/target_point", Point, getTargetPoint)
    rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


# -------------------------------
# Main Function
# -------------------------------
if __name__ == "__main__":
    rospy.init_node("JPS_path_planning")

    # Retrieve image size and map ratio from ROS parameters
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"JPS started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Start subscriber thread
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    # Start JPS planning thread
    JPS_thread = Thread(target=JPS)
    JPS_thread.daemon = True
    JPS_thread.start()

    # Keep the node running
    rospy.spin()

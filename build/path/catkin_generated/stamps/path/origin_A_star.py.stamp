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

# Global variable to store the target point
target_point = [0, 0]
# Flag to indicate whether path planning is needed
need_planning = False
# Global variable to store the robot's position
car_position = [0, 0]

# Get image size from the ROS parameter server and compute map ratio
image_size = rospy.get_param('image_size', 100)  # Default value is 100
# Map ratio - dynamically calculated based on image_size
# image_size values of 50, 100, 200 correspond to map_ratio = 2.5, 5, 10 respectively
map_ratio = float(image_size) / 20.0


# A* path planning function, generates path based on the target point
def A_star():
    # Three heuristic functions; you can search for differences online or switch them in the code below
    # Manhattan distance
    def Manhattan_mode(i, last_val, x, y):
        if i % 2 == 0:
            D = 1
        else:
            D = pow(2, 0.5)
        h = abs(x - end_point[0]) + abs(y - end_point[1])
        this_val = last_val + D + h
        return this_val

    # Euclidean distance
    def Euclid_mode(i, last_val, x, y):
        if i % 2 == 0:
            D = 1
        else:
            D = pow(2, 0.5)
        h = pow(pow(x - end_point[0], 2) + pow(y - end_point[1], 2), 0.5)
        this_val = last_val + D + h
        return this_val

    # Chebyshev distance
    def Chebyshev_mode(i, last_val, x, y):
        if i % 2 == 0:
            D = 1
        else:
            D = pow(2, 0.5)
        h = max(abs(x - end_point[0]), abs(y - end_point[1]))
        this_val = last_val + D + h
        return this_val

    global need_planning, target_point, car_position, image_size, map_ratio

    # Get resource path and image name from parameter server
    resources_path = rospy.get_param('resourcesPath')
    base_image_name = rospy.get_param('image_name', 'map')  # Default image is map.png
    image_name = f"{base_image_name}_{image_size}.png"  # Construct filename dynamically
    image_path = os.path.join(resources_path, image_name)
    rospy.loginfo("Loading map from: " + image_path)
    rospy.loginfo(f"Image size: {image_size}, Map ratio: {map_ratio}")

    # Publisher for the planned route
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)
    # 8-direction move set, cannot be modified
    operate_list = [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]

    # Load map as single-channel grayscale
    img = cv2.imread(image_path, 0)
    if img is None:
        rospy.logerr("Failed to load map image: " + image_path)
        return

    # Loop rate
    rate = rospy.Rate(10)
    # Path list
    route_list = []
    # Path list as string
    route_list_msg = String()

    # Main loop
    while not rospy.is_shutdown():
        # Start planning if flag is set
        if need_planning:
            rospy.loginfo("Starting path planning...")
            # Get map width and height
            size = img.shape
            w = size[1]  # width
            h = size[0]  # height
            # Set map boundaries
            x_max = size[1]
            y_max = size[0]
            # Store obstacles from map image
            obstacle_list = []
            for i in range(0, w):
                for q in range(0, h):
                    # [y, x]
                    if img[q, i] == 0:
                        obstacle_list.append([i, h - q - 1])

            # Set start and end points, with coordinate transformation
            start_point = [round(car_position[0] + image_size / 2), round(car_position[1] + image_size / 2)]
            end_point = [int(target_point[0] + image_size / 2), int(target_point[1] + image_size / 2)]
            start_point.extend([0, [copy.deepcopy(start_point[0]), copy.deepcopy(start_point[1])]])
            end_point.extend([0, [copy.deepcopy(end_point[0]), copy.deepcopy(end_point[1])]])

            # Initialize open and closed lists
            open_list = [start_point]
            close_list = [start_point]

            # A* iteration
            calculate_times = 0
            end_mark = 0
            while end_mark == 0 and not rospy.is_shutdown():
                # Compute movement cost
                f_list = []
                for i in list(range(0, len(open_list))):
                    f_list.append(open_list[i][2])

                # If no available points, break
                if not f_list:
                    rospy.logwarn("No path found!")
                    break

                # Choose point with minimum cost
                min_f = min(f_list)
                save_point = open_list[f_list.index(min_f)]
                close_list.append(open_list[f_list.index(min_f)])
                open_list.pop(f_list.index(min_f))

                # Search 8 neighbors
                for i in list(range(0, 8)):
                    temp_point = copy.deepcopy(save_point)
                    temp_point[3][0] = save_point[0]
                    temp_point[3][1] = save_point[1]
                    temp_point[0] = save_point[0] + operate_list[i][0]
                    temp_point[1] = save_point[1] + operate_list[i][1]
                    mark = 0
                    # Out of bounds?
                    if (temp_point[0] < 0 or temp_point[1] < 0 or
                            temp_point[0] > x_max or temp_point[1] > y_max):
                        mark = 1
                    if mark != 1:
                        # Already in open_list?
                        for q in list(range(0, len(open_list))):
                            if open_list[q][0] == temp_point[0] and open_list[q][1] == temp_point[1]:
                                mark = 1
                                break
                        if mark != 1:
                            # Already in close_list?
                            for p in list(range(0, len(close_list))):
                                if close_list[p][0] == temp_point[0] and close_list[p][1] == temp_point[1]:
                                    mark = 1
                                    break
                            if mark != 1:
                                # Is it an obstacle?
                                for r in list(range(0, len(obstacle_list))):
                                    if obstacle_list[r][0] == temp_point[0] and obstacle_list[r][1] == temp_point[1]:
                                        mark = 1
                                        break
                    # If free, compute cost
                    if mark == 0:
                        # Use Chebyshev heuristic (can switch here)
                        # temp_point[2] = Manhattan_mode(i, save_point[2], temp_point[0], temp_point[1])
                        # temp_point[2] = Euclid_mode(i, save_point[2], temp_point[0], temp_point[1])
                        temp_point[2] = Chebyshev_mode(i, save_point[2], temp_point[0], temp_point[1])
                        open_list.append(temp_point)
                        calculate_times += 1
                    # If end point is reached
                    if temp_point[0] == end_point[0] and temp_point[1] == end_point[1]:
                        end_mark = 1
                        close_list.append(temp_point)
                        break
                # Prevent infinite loops
                if calculate_times > 10000:
                    rospy.logwarn("Path planning exceeded maximum iterations!")
                    break

            if end_mark == 0:
                rospy.logerr("Failed to find a path to the target!")
                need_planning = False
                continue

            # Backtrack from end to start to build path
            temp_point = copy.deepcopy(close_list[-1])
            end_mark = 0
            route_list_ = []
            while end_mark == 0:
                route_list_.append(temp_point)
                for i in list(range(0, len(close_list))):
                    if close_list[i][0] == temp_point[3][0] and close_list[i][1] == temp_point[3][1]:
                        if close_list[i][0] == start_point[0] and close_list[i][1] == start_point[1]:
                            end_mark = 1
                        else:
                            temp_point = copy.deepcopy(close_list[i])
            route_list_.reverse()  # Reverse to start from beginning

            # Repulsion from obstacles
            route_list_length = len(route_list_)
            for i in range(1, route_list_length):
                if i != route_list_length - 1:
                    force_vector = [0, 0]
                    # Compute effect of obstacles
                    for q in range(0, len(obstacle_list)):
                        l = pow(pow(obstacle_list[q][0] - route_list_[i][0], 2) + pow(
                            obstacle_list[q][1] - route_list_[i][1], 2), 0.5)
                        # Only consider nearby obstacles
                        if l < 5:
                            a = math.atan2(route_list_[i][1] - obstacle_list[q][1],
                                           route_list_[i][0] - obstacle_list[q][0])
                            force_vector[0] += (1 / l) * 4 * math.cos(a)
                            force_vector[1] += (1 / l) * 4 * math.sin(a)
                    # Normalize force
                    l_ = pow(pow(force_vector[0], 2) + pow(force_vector[1], 2), 0.5)
                    if l_ > 2:
                        force_vector[0] *= 2 / l_
                        force_vector[1] *= 2 / l_
                    # Apply repulsion
                    route_list_[i][0] += int(force_vector[0])
                    route_list_[i][1] += int(force_vector[1])

            # Remove duplicate points caused by adjustment
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

            # Keep only coordinates, remove cost and parent info
            route_list = []
            for i in range(0, len(route_list_)):
                route_list.append([route_list_[i][0] - image_size / 2, route_list_[i][1] - image_size / 2])

            rospy.loginfo("Path planning completed with " + str(len(route_list)) + " points")
            need_planning = False

        # Convert route list to JSON and publish
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


def subData():
    # Callback for receiving target point
    def getTargetPoint(msg_t):
        global target_point, need_planning, map_ratio
        # Update only if target point has changed
        if msg_t.x != target_point[0] or msg_t.y != target_point[1]:
            target_point[0] = msg_t.x
            target_point[1] = msg_t.y
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    # Callback for receiving car position
    def getCarPosition(msg_p):
        global car_position, map_ratio
        # Coordinate transformation with dynamic map_ratio
        car_position = [msg_p.pose.pose.position.x * map_ratio, msg_p.pose.pose.position.y * map_ratio]

    target_point_sub = rospy.Subscriber("/target_point", Point, getTargetPoint)
    car_position_sub = rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("A_star_path_planning")

    # Get image size from parameter server and recalculate map ratio
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0
    rospy.loginfo(f"A* Path Planning started with image_size: {image_size}, map_ratio: {map_ratio}")

    # Start multithreaded data subscription and path planning
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    A_star_thread = Thread(target=A_star)
    A_star_thread.daemon = True
    A_star_thread.start()

    # Keep main thread alive
    rospy.spin()

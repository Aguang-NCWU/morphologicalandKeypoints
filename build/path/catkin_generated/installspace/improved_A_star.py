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

# ------------------ Global Variables ------------------
target_point = [0.0, 0.0]   # Target point (x, y)
need_planning = False       # Flag indicating whether path planning is required
car_position = [0.0, 0.0]   # Current position of the robot (x, y)
image_size = rospy.get_param('image_size', 100)  # Default value: 100

# Map scale factor - dynamically calculated based on image_size
# image_size values 50, 100, 200 correspond to map_ratio values 2.5, 5, 10 respectively
map_ratio = float(image_size) / 20.0


# ------------------ Utility Functions ------------------
def to_polar_list(points, origin):
    """Convert a list of (x, y) points into a list of (r, theta, idx) tuples
    using math functions (avoiding numpy type pitfalls)."""
    out = []
    for idx, pt in enumerate(points):
        dx = float(pt[0]) - float(origin[0])
        dy = float(pt[1]) - float(origin[1])
        r = math.sqrt(dx*dx + dy*dy)
        theta = math.atan2(dy, dx)
        out.append((r, theta, idx))
    return out


def find_first_obstacle(A, B):
    """Find the first edge segment in polar sequence A that is intersected
    by any point in polar set B (return the index or None)."""
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
    """Extract key points from a path based on polar coordinates."""
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
            coords = coords[obs_idx:]  # Recalculate starting from this point
            polar_coords = to_polar_list(coords, coords[0])
            obstacle_polar = [(r, t) for (r, t, _) in to_polar_list(obstacle_coords, coords[0])]
        else:
            break
    if coords:
        key_points.append(coords[-1])

    # Remove duplicates while preserving order
    kp = []
    for p in key_points:
        if kp and kp[-1] == p:
            continue
        kp.append([int(p[0]), int(p[1])])
    return kp


# ------------------ A* Main Logic ------------------
def A_star():
    global need_planning, target_point, car_position

    # Heuristic function (Chebyshev distance mode, alternating cost)
    def Chebyshev_mode(i, last_val, x, y, end_point):
        if i % 2 == 0:
            D = 1
        else:
            D = math.sqrt(2)
        h = max(abs(x - end_point[0]), abs(y - end_point[1]))
        return last_val + D + h

    # Read parameters (once outside the loop)
    try:
        resources_path = rospy.get_param('resourcesPath')
    except KeyError:
        rospy.logerr("Parameter 'resourcesPath' not found on param server.")
        return

    # Support both parameter names: image_name or image_base_name (only 'demo01')
    base_image_name = rospy.get_param('image_name', 'map')  # Default: map.png
    image_name = f"{base_image_name}_{image_size}.png"  # Construct image file name dynamically
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

    # Apply morphological processing (erosion)
    kernel_size = 3 if image_size == 50 else 5
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    img = cv2.erode(img_orig, kernel, iterations=1)

    # Publisher
    route_list_pub = rospy.Publisher("/route_list", String, queue_size=10)
    operate_list = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]

    rate = rospy.Rate(10)
    route_list = []
    route_list_msg = String()

    while not rospy.is_shutdown():
        if need_planning:
            rospy.loginfo("Starting path planning...")
            size = img.shape
            h = size[0]
            w = size[1]
            x_max = w
            y_max = h

            # Build obstacle list (in integer coordinates, flipping y-axis)
            obstacle_list = []
            for ix in range(w):
                for iy in range(h):
                    if img[iy, ix] == 0:
                        obstacle_list.append([ix, h - iy - 1])

            # Compute start and goal points (ensure within bounds)
            start_point = [int(round(car_position[0] + 50)), int(round(car_position[1] + 50))]
            end_point = [int(round(target_point[0] + 50)), int(round(target_point[1] + 50))]

            rospy.loginfo("Start (img coords): %s , End (img coords): %s" % (str(start_point), str(end_point)))

            # Boundary check
            def in_bounds(pt):
                return 0 <= pt[0] < x_max and 0 <= pt[1] < y_max

            if not in_bounds(start_point) or not in_bounds(end_point):
                rospy.logerr("Start or End out of map bounds. Aborting planning.")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Check if goal lies inside an obstacle
            if end_point in obstacle_list:
                rospy.logerr("End point is inside an obstacle!")
                need_planning = False
                route_list = []
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Initialize open/close lists (format: [x, y, f, [px, py]])
            start_node = [start_point[0], start_point[1], 0, [start_point[0], start_point[1]]]
            end_node = [end_point[0], end_point[1], 0, [end_point[0], end_point[1]]]
            open_list = [start_node]
            close_list = []
            calculate_times = 0
            end_mark = 0

            max_iterations = 20000

            # Main A* loop
            while end_mark == 0 and not rospy.is_shutdown():
                f_list = [node[2] for node in open_list]
                if not f_list:
                    rospy.logwarn("No path found (open list empty).")
                    break

                min_f = min(f_list)
                idx_min = f_list.index(min_f)
                save_point = open_list.pop(idx_min)
                close_list.append(save_point)

                for i_dir in range(8):
                    temp_point = copy.deepcopy(save_point)
                    temp_point[3][0] = save_point[0]
                    temp_point[3][1] = save_point[1]
                    temp_point[0] = save_point[0] + operate_list[i_dir][0]
                    temp_point[1] = save_point[1] + operate_list[i_dir][1]

                    mark = 0
                    # Boundary check
                    if temp_point[0] < 0 or temp_point[1] < 0 or temp_point[0] >= x_max or temp_point[1] >= y_max:
                        mark = 1
                    if mark != 1:
                        # Check open_list
                        if any((n[0] == temp_point[0] and n[1] == temp_point[1]) for n in open_list):
                            mark = 1
                        # Check close_list
                        if mark != 1 and any((n[0] == temp_point[0] and n[1] == temp_point[1]) for n in close_list):
                            mark = 1
                        # Check obstacle
                        if mark != 1 and [temp_point[0], temp_point[1]] in obstacle_list:
                            mark = 1

                    if mark == 0:
                        temp_point[2] = Chebyshev_mode(i_dir, save_point[2], temp_point[0], temp_point[1], end_point)
                        open_list.append(temp_point)
                        calculate_times += 1

                    if temp_point[0] == end_point[0] and temp_point[1] == end_point[1]:
                        end_mark = 1
                        close_list.append(temp_point)
                        break

                if calculate_times > max_iterations:
                    rospy.logwarn("Path planning exceeded maximum iterations (%d)!" % max_iterations)
                    break

            if end_mark == 0:
                rospy.logerr("Failed to find a path to the target!")
                need_planning = False
                route_list = []
                # Publish empty route
                route_list_msg.data = json.dumps(route_list)
                route_list_pub.publish(route_list_msg)
                rate.sleep()
                continue

            # Backtrack to reconstruct the path
            temp_point = copy.deepcopy(close_list[-1])
            route_seq = []
            while True:
                route_seq.append([temp_point[0], temp_point[1]])
                parent_x, parent_y = temp_point[3][0], temp_point[3][1]
                # Locate parent node
                found_parent = False
                for node in close_list:
                    if node[0] == parent_x and node[1] == parent_y:
                        temp_point = node
                        found_parent = True
                        break
                if not found_parent:
                    break
                # If back to start, stop
                if temp_point[0] == start_point[0] and temp_point[1] == start_point[1]:
                    break
            route_seq.reverse()

            # Extract key points (polar coordinate method)
            key_points = polar_key_points(route_seq, obstacle_list)
            # Convert key points back to -50..+50 coordinate range before publishing
            route_list = []
            for kp in key_points:
                route_list.append([int(kp[0] - 50), int(kp[1] - 50)])

            rospy.loginfo("Path planning completed with %d points" % len(route_list))
            need_planning = False

        # Publish (always publish, even if empty)
        route_list_msg.data = json.dumps(route_list)
        route_list_pub.publish(route_list_msg)
        rate.sleep()


# ------------------ Subscription Thread ------------------
def subData():
    global target_point, need_planning, car_position
    def getTargetPoint(msg_t):
        """Subscriber callback: update target point."""
        global target_point, need_planning
        if float(msg_t.x) != target_point[0] or float(msg_t.y) != target_point[1]:
            target_point[0] = float(msg_t.x)
            target_point[1] = float(msg_t.y)
            need_planning = True
            rospy.loginfo("New target point received: " + str(target_point))

    def getCarPosition(msg_p):
        """Subscriber callback: update car position (scaled)."""
        global car_position
        car_position = [float(msg_p.pose.pose.position.x) * map_ratio, float(msg_p.pose.pose.position.y) * map_ratio]

    rospy.Subscriber("/target_point", Point, getTargetPoint)
    rospy.Subscriber("/laser_robot/odom", Odometry, getCarPosition)
    rospy.spin()


# ------------------ Main Entry ------------------
if __name__ == "__main__":
    rospy.init_node("improved_a_star_node")
    # Start subscriber thread and A* planning thread
    Thread(target=subData, daemon=True).start()
    Thread(target=A_star, daemon=True).start()
    rospy.spin()

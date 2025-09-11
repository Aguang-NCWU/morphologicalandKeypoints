import rospy
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMdiArea, QMdiSubWindow, QWidget, QVBoxLayout, QLabel, \
    QPushButton
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QFont, QColor, QPainter, QPen
import cv2
from geometry_msgs.msg import Point
from threading import Thread
from std_msgs.msg import String
import json
from nav_msgs.msg import Odometry
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('--demo', type=str, default='demo01', help='Which demo to use')
args, unknown = parser.parse_known_args()  # Avoid errors caused by ROS launch passing extra arguments

demo_name = args.demo

# Get image size parameter from ROS parameter server
image_size = rospy.get_param('image_size', 100)  # Default value is 100
size = int(image_size)
# Global variable storing the path planning result list, containing several 2D coordinate points
route_list = []
# Global variable storing the target point, using Point type from geometry_msgs, initialized here
target_point = Point()
target_point.x = 0
target_point.y = 1
target_point.z = 0

# Global variable storing the robot's position
robotPosition = Odometry()

# Control flag list: first indicates whether to send target point, second indicates whether a new path is received and needs drawing
control_mark = [False, False]

# Global variable storing the path to the resources directory passed by launch file
resources_directory = String()

# Map ratio - calculated dynamically based on image_size
# For example: image_size = 50, 100, 200 correspond to map_ratio = 2.5, 5, 10
map_ratio = float(image_size) / 20.0


# Custom QLabel for drawing dashed paths (feature removed)
class PathLineLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.path_points = []
        self.setAttribute(Qt.WA_TransparentForMouseEvents)  # Allow mouse events to pass through
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowTransparentForInput)  # Remove border and make transparent
        # Set background to transparent
        self.setStyleSheet("background: transparent;")

    def set_path_points(self, points):
        self.path_points = points
        self.update()  # Trigger repaint

    def paintEvent(self, event):
        # No longer draw dashed lines
        pass


# Open GUI window
class MainWindow(QMainWindow):
    drawPathUpdate = pyqtSignal()  # Define a signal

    def __init__(self):
        super().__init__()
        # Global variable declaration
        global resources_directory, map_ratio, image_size
        self.resources_directory = resources_directory
        self.map_ratio = map_ratio
        self.image_size = image_size
        # Load images
        self.backgroundImgPath = os.path.join(self.resources_directory, f"{demo_name}_1000.png")
        self.image_path = os.path.join(self.resources_directory, f"{demo_name}_{self.image_size}.png")
        self.image = cv2.imread(self.image_path, 0)
        if self.image is None:
            raise FileNotFoundError(f"Map image does not exist: {self.image_path}")

        # Initialize lists storing start and end point positions
        self.start_label_pos = [0, 0]
        self.end_label_pos = [0, 1]

        # Path point drawing list
        self.path_label_list = []
        # Load path point patch
        self.pathPatch = QPixmap(self.resources_directory + "red_patch.png")
        # Set size of path point patch
        self.pathPatch = self.pathPatch.scaled(8, 8)

        # Create label for drawing dashed line (unused)
        self.path_line_label = PathLineLabel()

        # Flag for whether currently in path planning process, used to block mouse clicks
        self.inPathPlanningProcess = False

        # Add boundary check flag
        self.coordinates_out_of_bounds = False

        # Create main window
        self.setWindowTitle('Path Planning')
        # Main window initial position at (0, 0) with initial size 800x600
        self.setGeometry(0, 0, 800, 600)

        self.mdi_area = QMdiArea()
        self.setCentralWidget(self.mdi_area)

        # Create first subwindow and set its content
        self.subwindow1 = QMdiSubWindow()
        self.subwindow1.setWindowTitle('SubWindow1')  # Title
        self.subwindow1.setWindowFlags(Qt.FramelessWindowHint)  # Auto resize
        widget1 = QWidget()
        layout1 = QVBoxLayout(widget1)
        self.subwindow1.setWidget(widget1)
        self.mdi_area.addSubWindow(self.subwindow1)
        self.subwindow1.resize(600, 600)  # Initial size
        self.subwindow1.setObjectName("SubWindow1")  # Object name
        # Set background image
        self.subwindow1.setStyleSheet("#SubWindow1{border-image:url(%s)}" % (self.backgroundImgPath))
        # Bind mouse click event handler
        self.subwindow1.mousePressEvent = self.handle_mouse_press

        # Create second subwindow
        self.subwindow2 = QMdiSubWindow()
        self.subwindow2.setWindowFlags(Qt.FramelessWindowHint)
        widget2 = QWidget()
        layout2 = QVBoxLayout(widget2)

        # Add two buttons
        self.startButton = QPushButton('Start/Show')
        layout2.addWidget(self.startButton)
        self.startButton.clicked.connect(self.startButtonClicked)

        self.exitButton = QPushButton('Exit')
        layout2.addWidget(self.exitButton)
        self.exitButton.clicked.connect(self.exitButtonClicked)

        # Add three labels
        label2 = QLabel('Start:')
        label2.setWordWrap(True)
        label2_color = QColor("orange")
        label2.setStyleSheet(f"QLabel {{ color: {label2_color.name()}; }}")
        layout2.addWidget(label2)

        label3 = QLabel('End:')
        label3.setWordWrap(True)
        label3_color = QColor("blue")
        label3.setStyleSheet(f"QLabel {{ color: {label3_color.name()}; }}")
        layout2.addWidget(label3)

        label4 = QLabel('Output:')
        label4.setWordWrap(True)
        font = QFont("Arial", 14)
        label4.setFont(font)
        label4_color = QColor("red")
        label4.setStyleSheet(f"QLabel {{ color: {label4_color.name()}; }}")
        layout2.addWidget(label4)

        widget2.setLayout(layout2)
        self.subwindow2.setWidget(widget2)
        self.mdi_area.addSubWindow(self.subwindow2)
        self.subwindow2.resize(200, 600)

        # Create axis image for subwindow1
        self.axis_label = QLabel()
        axis = QPixmap(self.resources_directory + "axis.png")
        axis = axis.scaled(self.subwindow1.width() / 4, self.subwindow1.height() / 4)
        self.axis_label.setPixmap(axis)
        self.axis_label.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowTransparentForInput)
        self.axis_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.subwindow1.layout().addWidget(self.axis_label)

        # Create start point image
        self.start_label = QLabel()
        start = QPixmap(self.resources_directory + "orange_patch.png")
        start = start.scaled(20, 20)
        self.start_label.setPixmap(start)
        self.start_label.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowTransparentForInput)
        self.start_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.subwindow1.layout().addWidget(self.start_label)

        # Create end point image
        self.end_label = QLabel()
        end = QPixmap(self.resources_directory + "blue_patch.png")
        end = end.scaled(20, 20)
        self.end_label.setPixmap(end)
        self.end_label.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowTransparentForInput)
        self.end_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.subwindow1.layout().addWidget(self.end_label)

        # Bind subwindow show event
        self.subwindow1.showEvent = self.on_subwindow_show
        # Bind main window resize event
        self.resizeEvent = self.on_resize

    # Window refresh
    def on_subwindow_show(self, event):
        # Draw axis
        axis = self.axis_label.pixmap()
        if axis:
            # Calculate and set axis position (14 is an empirical offset)
            self.axis_label.setGeometry(
                self.subwindow1.width() / 2 - 14,
                self.subwindow1.height() / 2 - axis.height() + 14,
                axis.width(),
                axis.height()
            )

        # Draw start point
        start = self.start_label.pixmap()
        if start:
            self.start_label.setGeometry(
                int(((self.start_label_pos[0] + (size / 2)) / size) * self.subwindow1.width() - start.width() / 2),
                int(((-self.start_label_pos[1] + (size / 2)) / size) * self.subwindow1.height() - start.height() / 2),
                start.width(),
                start.height()
            )

        # Draw end point
        end = self.end_label.pixmap()
        if end:
            self.end_label.setGeometry(
                int(((self.end_label_pos[0] + (size / 2)) / size) * self.subwindow1.width() - end.width() / 2),
                int(((-self.end_label_pos[1] + (size / 2)) / size) * self.subwindow1.height() - end.height() / 2),
                end.width(),
                end.height()
            )

        # Update path points
        for i in range(0, len(self.path_label_list)):
            path_label = self.path_label_list[i].pixmap()
            if path_label:
                self.path_label_list[i].setGeometry(
                    int(((route_list[i][0] + (size / 2)) / size) * self.subwindow1.width() - path_label.width() / 2),
                    int(((-route_list[i][1] + (size / 2)) / size) * self.subwindow1.height() - path_label.height() / 2),
                    path_label.width(),
                    path_label.height()
                )

    def on_resize(self, event):
        # Adjust subwindow size according to main window resize
        self.subwindow1.resize(self.width() * 0.75, self.height())
        self.subwindow1.move(0, 0)
        self.subwindow2.resize(self.width() * 0.25, self.height())
        self.subwindow2.move(self.width() * 0.75, 0)
        self.on_subwindow_show(self)

    # Start button
    def startButtonClicked(self):
        self.updateStartAndEndLabel()
        self.on_subwindow_show(self)

    # Exit button
    def exitButtonClicked(self):
        exit()

    # Mouse click handler
    def handle_mouse_press(self, event):
        global control_mark, target_point, robotPosition
        # Handle left-click, only if not in path planning process
        if event.button() == Qt.LeftButton and not self.inPathPlanningProcess:
            # Get global mouse position
            global_pos = event.globalPos()
            # Convert to subwindow1 local position
            local_pos = self.subwindow1.mapFromGlobal(global_pos)
            # Extract click coordinates
            x = local_pos.x()
            y = local_pos.y()
            # Convert to coordinates centered at subwindow1
            x = int((x / self.subwindow1.width()) * size - (size / 2))
            y = int(((self.subwindow1.height() - y) / self.subwindow1.height()) * size - (size / 2))

            # Update end point position
            self.end_label_pos = [x, y]

            # Boundary checks
            if self.coordinates_out_of_bounds:
                self.updateOutputText("Selected point is out of map range, please choose again...")
            elif self.end_loc_in_obstacle():
                self.updateOutputText("End point is in an obstacle, please choose again...")
            elif self.endIsStart():
                self.updateOutputText("End point overlaps start point, please choose again...")
                self.updateStartAndEndLabel()
                self.on_subwindow_show(self)
            else:
                # Enable target point sending
                control_mark[0] = True
                # Write target point info
                target_point.x = self.end_label_pos[0]
                target_point.y = self.end_label_pos[1]
                # Update current start point based on robot position from Gazebo
                self.start_label_pos[0] = round(robotPosition.pose.pose.position.x * self.map_ratio)
                self.start_label_pos[1] = round(robotPosition.pose.pose.position.y * self.map_ratio)
                self.updateOutputText("Planning path, please wait...")
                self.updateStartAndEndLabel()
                self.on_subwindow_show(self)
                # Enable in-path-planning flag
                self.inPathPlanningProcess = True

    # Update start/end label text
    def updateStartAndEndLabel(self):
        for child in self.subwindow2.findChildren(QLabel):
            if child.text().startswith('Start:'):
                child.setText('Start:' + str(self.start_label_pos[0]) + ', ' + str(self.start_label_pos[1]))
            if child.text().startswith('End:'):
                child.setText('End:' + str(self.end_label_pos[0]) + ', ' + str(self.end_label_pos[1]))

    # Update output text
    def updateOutputText(self, text):
        for child in self.subwindow2.findChildren(QLabel):
            if child.text().startswith('Output:'):
                child.setText('Output:' + text)

    # Check if selected end point is inside obstacle
    def end_loc_in_obstacle(self):
        in_obstacle = False
        self.coordinates_out_of_bounds = False

        # Convert float coordinates to integers
        x_coord = int(self.end_label_pos[0] + (size / 2))
        y_coord = int(self.image.shape[0] - (self.end_label_pos[1] + (size / 2)))

        # Ensure coordinates are within image
        if 0 <= y_coord < self.image.shape[0] and 0 <= x_coord < self.image.shape[1]:
            # Obstacles are black (grayscale = 0)
            if self.image[y_coord, x_coord] == 0:
                in_obstacle = True
        else:
            self.coordinates_out_of_bounds = True
            in_obstacle = True

        return in_obstacle

    # Check if end point equals start point
    def endIsStart(self):
        isStart = False
        if self.end_label_pos == self.start_label_pos:
            isStart = True
        return isStart

    # Draw path after receiving data
    def drawPath(self):
        # Delete old path
        for path_label in self.path_label_list:
            if path_label and path_label.parent():
                self.subwindow1.layout().takeAt(self.subwindow1.layout().indexOf(path_label))
                path_label.deleteLater()

        # Reset path label list
        self.path_label_list = []
        self.path_label_list = [None] * len(route_list)
        for i in range(0, len(route_list)):
            self.path_label_list[i] = QLabel()
            self.path_label_list[i].setPixmap(self.pathPatch)
            self.path_label_list[i].setWindowFlags(Qt.FramelessWindowHint | Qt.WindowTransparentForInput)
            self.path_label_list[i].setAttribute(Qt.WA_TransparentForMouseEvents)
            self.subwindow1.layout().addWidget(self.path_label_list[i])
            path_label = self.path_label_list[i].pixmap()
            self.path_label_list[i].setGeometry(
                int(((route_list[i][0] + (size / 2)) / size) * self.subwindow1.width() - path_label.width() / 2),
                int(((-route_list[i][1] + (size / 2)) / size) * self.subwindow1.height() - path_label.height() / 2),
                path_label.width(),
                path_label.height()
            )

        self.on_subwindow_show(self)
        self.updateOutputText('Path planning completed. Click "Start/Update" to display, or click the map to reselect.')
        self.inPathPlanningProcess = False


# ROS Publisher function
def pubData():
    global control_mark, target_point
    target_point_pub = rospy.Publisher('/target_point', Point, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if control_mark[0]:
            target_point_pub.publish(target_point)
        rate.sleep()


# ROS Subscriber function
def subData():
    def getRouteList(msg_r):
        global route_list, control_mark
        list_msg_r = json.loads(str(msg_r.data))
        if list_msg_r != route_list and len(list_msg_r) > 0:
            route_list = list_msg_r
            control_mark[0] = False
            control_mark[1] = True
            window.drawPathUpdate.emit()

    def getRobotPos(msg_p):
        global robotPosition
        robotPosition = msg_p

    route_list_sub = rospy.Subscriber("/route_list", String, getRouteList)
    robot_pos_sub = rospy.Subscriber("/laser_robot/odom", Odometry, getRobotPos)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("runPathPlanningGuiNode")
    # Get resource directory path from launch parameters
    resources_directory = rospy.get_param('resourcesPath')
    # Get image size
    image_size = rospy.get_param('image_size', 100)
    map_ratio = float(image_size) / 20.0

    # Create GUI window
    app = QApplication(sys.argv)
    window = MainWindow()
    window.drawPathUpdate.connect(window.drawPath)
    window.show()

    # Start publisher thread
    pubData_thread = Thread(target=pubData)
    pubData_thread.daemon = True
    pubData_thread.start()

    # Start subscriber thread
    subData_thread = Thread(target=subData)
    subData_thread.daemon = True
    subData_thread.start()

    # Close process when window closes
    sys.exit(app.exec_())
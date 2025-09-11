# Image cropping and resource image generation program with different sizes
import rospy
import cv2 as cv
import numpy as np
import os


if __name__ == "__main__":

    rospy.init_node("cutPgm_testWorld")
    # Get the current workspace, pointing to the SLAMandGuide folder
    workSpacePath = os.getcwd() 
    # Load the path of the pgm file and define the save locations and names for two images of different sizes
    pgmPath = workSpacePath + '/src/integrated_launch/resources/demo.pgm'
    savePath1 = workSpacePath + '/src/integrated_launch/resources/testWorld1_100_100.png'
    savePath2 = workSpacePath + '/src/integrated_launch/resources/testWorld1_1000_1000.png'

    # Load the pgm map
    image = cv.imread(pgmPath)
    # Crop the image. The cropping parameters need to be adjusted manually and are image-specific, not universal
    image = image[1800: 2200, 1800: 2200]
    # Resize the cropped image to 100×100
    image = cv.resize(image, (100, 100), interpolation=cv.INTER_AREA)
    # Convert the image to a single-channel grayscale image
    image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # Create an empty image with the same size as the cropped image (100×100). 
    # Initialize all pixel values to 0 (black), representing obstacles
    image_new = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

    # Traverse each pixel in the grayscale image
    for i in range(image.shape[0]):  # Iterate through all rows
        for j in range(image.shape[1]):  # Iterate through all columns
            # Query the grayscale value of the pixel. 
            # 240 is an empirical threshold (range 0–255). 
            # If the pixel value > 240, set it to 255 (white), representing free space
            if image[i, j] > 240:
                image_new[i, j] = 255

    # Resize the processed image to 1000×1000 using linear interpolation
    image_new_ = cv.resize(image_new, (1000, 1000), interpolation=cv.INTER_AREA)

    # Display the cropped image
    cv.namedWindow("image", cv.WINDOW_NORMAL)
    cv.imshow('image', image_new)
    cv.resizeWindow("image", 800, 600)
    # Save the two images with different sizes
    cv.imwrite(savePath1, image_new)
    cv.imwrite(savePath2, image_new_)
    keyValue = cv.waitKey(0)             # Capture keyboard input
    if keyValue == 27:     # ESC key value
        cv.destroyAllWindows() 
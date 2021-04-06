import numpy as np
import cv2 as cv
import time
import lines_tools as ltools
import os

'''   Video processing    '''
# Video capture do apk 'DroidCam'
droidcam = 'http://192.168.0.25:4747/video'
gazebo_video = "./tentativa4-2021-03-16_14.29.webm"
cap = cv.VideoCapture(gazebo_video)

# Assuming a line that has 2x20 cm
A_known = 0.02*0.08

# Intrinsics
intrinsics = np.array([[277, 0, 160],
                       [0, 277, 120],
                       [0, 0, 1]])

# Show image function
def show_img(image, name):
    
    cv.namedWindow(name, cv.WINDOW_GUI_EXPANDED)
    cv.imshow(name, image)
    
while True:

    # Set cronometer
    t = time.time()

    # Capture frame by frame    
    ret, frame = cap.read()

    ## Descomentar quando utilizar o DroidCam
    # frame = cv.rotate(frame,cv.ROTATE_90_CLOCKWISE)
    # frame = cv.flip(frame, 1)[:,10:]

    # Src images to both methods
    blur = cv.GaussianBlur(frame, (5,5), 1)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    
    '''direction by lane center (moments) method:''' 
    lower_moments = np.array([0, 0, 100])
    upper_moments = np.array([255, 255, 255])
    mask_moments = cv.inRange(hsv, lower_moments, upper_moments)
        
    flag_moments, y_pts_moments, x_pts_moments, areas = ltools.direction_by_moments(mask_moments, remove_outliers=True)
    
    
    if flag_moments == 1:
        
        # Calculate yaw
        yaw_moments = ltools.yaw_angle_rads_alternative(x_pts_moments, y_pts_moments)
        
        # Calculate x, y and z
        x_near, y_near, x, y, z = ltools.get_xyz(mask_moments ,intrinsics, A_known, x_pts_moments, y_pts_moments, areas)
        
        # Draw ROI and orientation line
        hud_moments_img = ltools.display_HUD(blur)
        direction_by_moments_img = ltools.display_line_of_orientation(hud_moments_img, yaw_moments, line_length=150 ,put_text=False )
        
        # Draw points on image
        ltools.draw_circles(direction_by_moments_img, x_pts_moments, y_pts_moments)
        cv.circle(direction_by_moments_img, (int(x_near), int(y_near)),  radius = 7, color = (0,255,0), thickness=-1)

        # Display infos on image
        fps_txt = "FPS: " + str(round(1/(time.time()-t),0))
        coord_txt = "x:" + str(round(x,2)) + " y:" + str(round(y,2)) + " z:" + str(round(z,2)) + "(m)"
        orient_txt = "yaw:" + str(round(yaw_moments,2)) + "(rads)"
        cv.putText(direction_by_moments_img, coord_txt, (20,30), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,50), 2)
        cv.putText(direction_by_moments_img, orient_txt, (20,70), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,50), 2)
        cv.putText(direction_by_moments_img, fps_txt, (20,110), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,50), 2)

        # IO interface
        ltools.show_img(direction_by_moments_img, "Direction by moments")
    
    # '''direction by 10pts method:'''
    # lower_10pts = np.array([0, 0, 0])
    # upper_10pts = np.array([0, 0, 5])
    # mask_10pts = cv.inRange(hsv, lower_10pts, upper_10pts)
    
    # flag_10pts, y_10pts, x_10pts = ltools.direction_by_10pts(mask_10pts, remove_outliers=True)
    
    # if flag_10pts == 1:
    #     yaw_10pts = ltools.yaw_angle_rads_alternative(x_10pts, y_10pts)
    #     hud_10pts_img = ltools.display_HUD(blur)
    #     direction_by_10pts_img = ltools.display_line_of_orientation(hud_10pts_img, yaw_10pts, put_text=True)
    #     ltools.draw_circles(direction_by_10pts_img, x_10pts, y_10pts)
    #     ltools.show_img(direction_by_10pts_img, "Direction by 10 pts")
    
    ##uncomment to show masks
    # show_img(mask_moments, "mask_moments")
    # show_img(mask_10pts, "mask_10pts")

    k = cv.waitKey(1) & 0xff
    if k == 27:
        break

cap.release()
cv.destroyAllWindows()

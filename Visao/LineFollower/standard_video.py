import numpy as np
import cv2 as cv
import lines_tools as ltools

'''   Video processing    '''
# Video capture do apk 'DroidCam'
droidcam = 'http://192.168.0.25:4747/video'
gazebo_video = "gazebo01.webm"
cap = cv.VideoCapture(gazebo_video)

# Show image function
def show_img(image, name):
    
    cv.namedWindow(name, cv.WINDOW_GUI_EXPANDED)
    cv.imshow(name, image)
    

def nothing(x):
    pass

cv.namedWindow("Trackbars")
cv.createTrackbar("L-H", "Trackbars", 0, 255, nothing)
cv.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
cv.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
cv.createTrackbar("U-H", "Trackbars", 0, 255, nothing)
cv.createTrackbar("U-S", "Trackbars", 0, 255, nothing)
cv.createTrackbar("U-V", "Trackbars", 0, 255, nothing)



# Show video
while True:
    # Capture frame by frame    
    ret, frame = cap.read()
    ## Descomentar quando utilizar o DroidCam
    # frame = cv.rotate(frame,cv.ROTATE_90_CLOCKWISE)
    # frame = cv.flip(frame, 1)[:,10:]

    
    l_h = cv.getTrackbarPos("L-H", "Trackbars")
    l_s = cv.getTrackbarPos("L-S", "Trackbars")
    l_v = cv.getTrackbarPos("L-V", "Trackbars")
    u_h = cv.getTrackbarPos("U-H", "Trackbars")
    u_s = cv.getTrackbarPos("U-S", "Trackbars")
    u_v = cv.getTrackbarPos("U-V", "Trackbars")

    # src images to both methods
    imagens = ltools.binarization(frame, noise_level=1, threshold_type="global")
    blur = imagens["blur"]
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    
    
    '''direction by lane center (moments) method:''' 
    lower_moments = np.array([0, 0, 100])
    upper_moments = np.array([255, 255, 255])
    mask_moments = cv.inRange(hsv, lower_moments, upper_moments)
    binary_of_interest_moments = ltools.region_of_interest(mask_moments)
    
    flag_moments, y_pts_drone, x_pts_drone, y_pts_cv, x_pts_cv = ltools.direction_by_moments(binary_of_interest_moments)
    
    if flag_moments == 1:
        yaw_moments = ltools.yaw_angle_rads(x_pts_drone, y_pts_drone)
        hud_moments_img = ltools.display_HUD(blur)
        direction_by_moments_img = ltools.display_line_of_orientation(hud_moments_img, yaw_moments, put_text=True)
        ltools.draw_circles(direction_by_moments_img, x_pts_cv, y_pts_cv)
        ltools.show_img(direction_by_moments_img, "Direction by moments")


    '''direction by 10pts method:'''
    lower_10pts = np.array([0, 0, 0])
    upper_10pts = np.array([0, 0, 5])
    mask_10pts = cv.inRange(hsv, lower_10pts, upper_10pts)
    binary_of_interest_10pts = ltools.region_of_interest(mask_10pts)

    flag_10pts, y_10pts, x_10pts = ltools.direction_by_10pts(binary_of_interest_10pts)
    
    if flag_10pts == 1:
        yaw_10pts = ltools.yaw_angle_rads(x_10pts, y_10pts)
        hud_10pts_img = ltools.display_HUD(blur)
        direction_by_10pts_img = ltools.display_line_of_orientation(hud_10pts_img, yaw_10pts, put_text=True)
        ltools.draw_circles(direction_by_10pts_img, x_10pts, y_10pts)
        ltools.show_img(direction_by_10pts_img, "Direction by 10 pts")
    
    ##uncomment to show masks
    # show_img(mask_moments, "mask_moments")
    # show_img(mask_10pts, "mask_10pts")
    k = cv.waitKey(1) & 0xff
    if k == 27:
        break

cap.release()
cv.destroyAllWindows()

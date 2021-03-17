import numpy as np
import cv2 as cv
import lines_tools as ltools

'''   Video processing    '''
# Video capture do apk 'DroidCam'
droidcam = 'http://192.168.0.25:4747/video'
gazebo_video = "/home/lucas/Documents/Codes/PyProjects/02_lane_detection/01_videosrc/gazebo01.webm"
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
    # Descomentar quando utilizar o DroidCam
    # frame = cv.rotate(frame,cv.ROTATE_90_CLOCKWISE)
    # frame = cv.flip(frame, 1)[:,10:]

    # noisy = ltools.add_gaussian_noise(frame)
    imagens = ltools.binarization(frame, noise_level=1, threshold_type="global")
    
    blur = imagens["blur"]
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    

    l_h = cv.getTrackbarPos("L-H", "Trackbars")
    l_s = cv.getTrackbarPos("L-S", "Trackbars")
    l_v = cv.getTrackbarPos("L-V", "Trackbars")
    u_h = cv.getTrackbarPos("U-H", "Trackbars")
    u_s = cv.getTrackbarPos("U-S", "Trackbars")
    u_v = cv.getTrackbarPos("U-V", "Trackbars")

    lower = np.array([0, 0, 0])
    upper = np.array([255, 255, 5])

    mask = cv.inRange(hsv, lower, upper)
    # ret, thresh = cv.co
    try:
        y_pts, x_pts = ltools.direction_by_lane_center()
        yaw = ltools.yaw_angle_rads(coordinates)
        line_on_img = ltools.display_line_of_orientation(blur, yaw)
        final_img = ltools.display_HUD(line_on_img)
        print(np.cos(yaw))
        for i in range(len(y_pts)):
            cv.circle(final, (x_pts[i], y_pts[i]),color = (0,0,255), radius = 5, thickness = 2)
        print("ok")

    except:
        final_img = ltools.display_HUD(blur)
        print("not ok")
    # cv.circle(final_img, (coordinates[1], coordinates[0]), radius = 5, color = (0,0,255), thickness = 1)
    show_img(final_img, "final")
    show_img(hsv, "HSV")
    show_img(mask, "mask")
    k = cv.waitKey(1) & 0xff
    if k == 27:
        break

cap.release()
cv.destroyAllWindows()

import numpy as np
import cv2 as cv
import lines_tools as ltools

'''   Video processing    '''
# Video capture do apk 'DroidCam'
droidcam = 'http://192.168.0.25:4747/video'
gazebo_video = "tentativa4-2021-03-16_14.29.webm"
cap = cv.VideoCapture(gazebo_video)

# Show image function
def show_img(image, name):
    
    cv.namedWindow(name, cv.WINDOW_GUI_EXPANDED)
    cv.imshow(name, image)
    
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

    lower = np.array([0, 0, 100])
    upper = np.array([255, 255, 255])

    bin = cv.inRange(hsv, lower, upper)
    
    #try:
    flag, y_pts_drone, x_pts_drone, y_pts_cv, x_pts_cv = ltools.direction_by_lane_center(bin)
    #printValues(x_pts, y_pts, flag, )

    if flag == 1:
        coords_vert = ltools.filter_points(x_pts_drone, y_pts_drone)
        yaw = ltools.yaw_angle_rads(coords_vert[:,0][:], coords_vert[:,1][:])
        line_on_img = ltools.display_line_of_orientation(blur, yaw, line_length=200)
        blur = ltools.display_HUD(line_on_img)

        for i in range(len(y_pts_cv)):
            cv.circle(blur, (np.int(x_pts_cv[i]), np.int(y_pts_cv[i])),color = (0,0,255), radius = 5, thickness = 9)
    
        for i in range(len(coords_vert)):
            cv.circle(blur, (np.int(coords_vert[i][0]), np.int(blur.shape[0] - coords_vert[i][1])),color = (0,255,0), radius = 5, thickness = 9)

        print(coords_vert)
    #final_img = ltools.display_HUD(blur)
    
    
    show_img(blur, "final")
    show_img(hsv, "HSV")
    show_img(bin, "bin")
    
    k = cv.waitKey(1) & 0xff
    if k == 27:
        break

cap.release()
cv.destroyAllWindows()

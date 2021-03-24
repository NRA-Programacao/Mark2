import numpy as np
import cv2 as cv


def show_img(image, name):
    cv.namedWindow(name, cv.WINDOW_GUI_EXPANDED)
    cv.imshow(name, image)

def get_image_parameters(image):
    
    img_shape = image.shape
    img_height, img_width = image.shape[0], image.shape[1]

    parameters = {"image_shape": img_shape, "image_height": img_height, "image_width": img_width} 
    return parameters

def binarization(imagem, noise_level = 1, threshold_type = "otsu", inv = "True"):
    ''' 
        Filtro para remover ruídos da imagem:
        
            --img0: cópia da imagem original
            -- noise_level(int): nivel de ruido na imagem original. Default = 1
            --blur_img: homogeneizar  a imagem img0. O gaussian blur é aplicado proporcionalmente ao nivel de ruido. kernel 3x3
            --bw_img: torna a blur_img em preto e branco
            --opening5: homogeneização separada das regiões claras e escuras (kernel np.ones((5,5))) 
            --binary: binariza a opening5, caso necessário
            --canny: destaca os contornos
            --dilat: aplica morfologia "dilate" na imagem canny para destacar as extremidades/contornos
            --lines: output da HoughLinesP, que retorna as linhas da imagem
            --
    '''
    # imagem para edição
    img0 = np.copy(imagem)
        
    def apply_blur():
        # Aplica o Gaussian Blur numa quantidade de vezes que depende do nivel de ruido
        blur = np.copy(img0)
        for i in range(noise_level):
            blur = cv.GaussianBlur(blur, (3,3), 1)
        return blur
    
    blur_img = apply_blur()

    # imagem preto e branco
    bw_img = cv.cvtColor(blur_img, cv.COLOR_BGR2GRAY)    

    # aplicando morfologia "opening"
    kernel5 = np.ones((5,5), np.uint8)
    opening5 = cv.morphologyEx(bw_img, cv.MORPH_OPEN, kernel5)

    def apply_threshold():
        # THRESHOLD da imagem (binarizacao):
        if threshold_type == "otsu":
            if inv == False:
                    ret, thresh = cv.threshold(opening5,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
            else:
                    ret, thresh = cv.threshold(opening5,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
            
        elif threshold_type =="adaptative":
            if inv == False:
                thresh = cv.adaptiveThreshold(opening5, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11,2 )
                
            else:
                thresh = cv.adaptiveThreshold(opening5, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
        elif threshold_type == "global":
            if inv == False:
                ret, thresh = cv.threshold(opening5,40,255,cv.THRESH_BINARY)
            else:
                ret, thresh = cv.threshold(opening5,40,255,cv.THRESH_BINARY_INV)
                
        return thresh
    
    thresh_img = apply_threshold()
    
    contours, hierarchy = cv.findContours(thresh_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Canny
    canny = cv.Canny(thresh_img, 100, 255)
    
    # Dilatation aplicado ao canny
    dilat = cv.dilate(canny, kernel5, iterations=1)

    output_imgs = {"blur": blur_img, "contours": contours, "bw": bw_img, "thresh": thresh_img}
    return output_imgs

def region_of_interest(image, three_channels = False):

    """
        In this case, our region of interest corresponds to the central of the image(to focus on the central lane). We will apply the 'direction_by_10pts' or 'direction_by_moments'
        in this region to obtain our points.

        -- (x_max - x_min) = width of the region, that's, 20% of total image width (image.shape[1]) 
        -- y_max = height of the selected region is equal to the image height
        -- mask = binary array of the same shape of 'image'. It's central elements are equal to 1. The others elements are equal to 0 
    """

    width = image.shape[1]
    height = image.shape[0]

    mask = np.zeros_like(image)

    x_min = int(0.4*width)
    x_max = int(0.6*width)
    y_min = 0
    y_max = height

    mask[y_min:y_max, x_min:x_max] = 1
    region_of_interest = mask*image

    return region_of_interest

def direction_by_10pts(binary_img, remove_outliers = False):

    '''   
        --- direction_by_10pts: 75% of binary_inv image is horizontally sliced (cropped) in 10 parts. We will then compute the non zeros
        indexes of each part. After that, we calculate and return the centers, (y_center, x_center)_i (i = 0, 1,.., 9),  of all 
        those parts. These point's coordinates will be the input of np.polyfit(X_pts, Y_pts, deg = 1) function. 

        -- L: height of the region will be horizontally sliced.      
        -- dL: height of each sliced part
        -- y_min: slices begin at the point (y = y_min, x = 0)
        -- IMPORTANT: x_min, x_max, y_min_y_max need to be the same of 'region_of_interest' function
        -- pti (i = 0, 1, ..,9): coordinates of (y_center, x_center)_i (WITH RESPECT TO i-th SLICE'S COORDINATE SYSTEM). 
                To correct the coordinates, we need to sum      y_center_i + (y_min + (i-1*dL))
        -- yx_correction: array with the (i-1)*dL values
        -- yx_cg_coordinates: all 10 (y_center, x_center) we will use as input of np.polyfit
    '''
    
    

    """
        Region of interest
        In this case, our region of interest corresponds to the central of the binary_img(to focus on the central lane). We will apply the 'direction_by_10pts' or 'direction_by_moments'
        in this region to obtain our points.

        -- (x_max - x_min) = width of the region, that's, 20% of total binary_img width (binary_img.shape[1]) 
        -- y_max = height of the selected region is equal to the binary_img height
        -- mask = binary array of the same shape of 'binary_img'. It's central elements are equal to 1. The others elements are equal to 0 
    """
    
    height = binary_img.shape[0]
    width = binary_img.shape[1]
    x_min = int(0.4*width)
    x_max = int(0.6*width)
    y_min = 0
    y_max = height

    num_of_slices = 10
    L = int(1*height)
    dL = int(L/num_of_slices)
    
    bin_of_interest = region_of_interest(binary_img)
    
    pt0 = np.mean(np.nonzero(bin_of_interest[y_min+0*dL:y_min+dL,:]), axis=1)
    pt1 = np.mean(np.nonzero(bin_of_interest[y_min+dL:y_min+2*dL,:]), axis=1)
    pt2 = np.mean(np.nonzero(bin_of_interest[y_min+2*dL:y_min+3*dL,:]), axis=1)
    pt3 = np.mean(np.nonzero(bin_of_interest[y_min+3*dL:y_min+4*dL,:]), axis=1)
    pt4 = np.mean(np.nonzero(bin_of_interest[y_min+4*dL:y_min+5*dL,:]), axis=1)
    pt5 = np.mean(np.nonzero(bin_of_interest[y_min+5*dL:y_min+6*dL,:]), axis=1)
    pt6 = np.mean(np.nonzero(bin_of_interest[y_min+6*dL:y_min+7*dL,:]), axis=1)
    pt7 = np.mean(np.nonzero(bin_of_interest[y_min+7*dL:y_min+8*dL,:]), axis=1)
    pt8 = np.mean(np.nonzero(bin_of_interest[y_min+8*dL:y_min+9*dL,:]), axis=1)
    pt9 = np.mean(np.nonzero(bin_of_interest[y_min+9*dL:y_min+10*dL,:]), axis=1)
    
    yx_correction = np.array([[y_min , 0],[y_min + dL,0],[y_min + 2*dL,0],[y_min + 3*dL,0],[y_min + 4*dL,0],
    [y_min + 5*dL,0],[y_min + 6*dL,0],[y_min + 7*dL,0],[y_min + 8*dL,0],[y_min + 9*dL,0]])

    yx_cg_coordinates = np.array([pt0,pt1, pt2, pt3,pt4, pt5, pt6,pt7, pt8, pt9]) + yx_correction
    y_pts = yx_cg_coordinates[:,0][:]
    x_pts = yx_cg_coordinates[:,1][:] 
    # Removing possible "NaN" elements in our array
    y_pts_no_nan_cv = y_pts[np.logical_not(np.isnan(y_pts))]
    x_pts_no_nan_cv = x_pts[np.logical_not(np.isnan(x_pts))]

    if x_pts_no_nan_cv.shape[0] == 0 or y_pts_no_nan_cv.shape[0] == 0:
            flag = 0
    else:
        flag = 1
        if remove_outliers == True:
            x_pts_no_nan_cv, y_pts_no_nan_cv = filter_points(x_pts_no_nan_cv, y_pts_no_nan_cv)
    
    return flag, y_pts_no_nan_cv, x_pts_no_nan_cv
    
    
    
def direction_by_moments(binary_img, LOWER_AREA = 10, HIGHER_AREA = 1000, remove_outliers = False):

    
    height = binary_img.shape[0]

    # MEGA se multiplicar por 0 os pixels que vc quer eliminar (bin_of_interest = region_of_interest(binary_img), a imagem permanece com o mesmo shape
    #aí não precisa corrigir usando  x_pts_cv = centroids[:,0][:] + lower_xval
    bin_of_interest = region_of_interest(binary_img)
  
    def moments_pass(moments):
        flag = moments['m00'] > LOWER_AREA and moments['m00'] < HIGHER_AREA
        flag = flag and moments['m10'] != 0
        flag = flag and moments['m01'] != 0 
        return flag

    # Get contours and hierarchy
    contours, hierarchy = cv.findContours(bin_of_interest, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
       
    if(len(contours) != 0):

        areas = []
        centroids = np.zeros((len(contours),2))
        num_contour = 0

        for i in range(len(contours)):
            moments = cv.moments(contours[int(i)])
            

            if(moments_pass(moments)):
                # print(moments_pass(moments))
                # Cx = m10/m00, Cy = m01/m00
                centroids[num_contour][0] = moments['m10']/moments['m00']
                centroids[num_contour][1] = moments['m01']/moments['m00'] 
                num_contour += 1

        centroids = np.resize(centroids, (num_contour, 2))
        
        x_pts_cv = centroids[:,0][:] 
        y_pts_cv = centroids[:,1][:]
        
               
        if x_pts_cv.shape[0] == 0 or y_pts_cv.shape[0] == 0:
            flag = 0
        else:
            flag = 1
            if remove_outliers == True:
                x_pts_cv, y_pts_cv = filter_points(x_pts_cv, y_pts_cv)

        return flag, y_pts_cv, x_pts_cv 

    flag = 0
    return flag, -1, -1, -1, -1

def filter_points(x_pts, y_pts, m = 0.75):

    if x_pts.shape[0] == 0:
        return 0

    x_median = np.median(x_pts)
    x_std = np.std(x_pts)
    delta = abs(x_pts - x_median)
    
    x_pts_without_outliers = x_pts[np.less(delta, m*x_std)]
    y_pts_without_outliers = y_pts[np.less(delta, m*x_std)]

    return x_pts_without_outliers, y_pts_without_outliers

def yaw_angle_rads(X_pts, Y_pts):
    
    #Y_pts, X_pts = coordinates[:,0][:], coordinates[:,1][:]
    if X_pts.shape[0] <= 1 and Y_pts.shape[0] <= 1:
        return 0

    m,b = np.polyfit(X_pts, Y_pts, deg = 1)
  
    yaw_rad_in_arctan_domain = - np.arctan(m)

    if yaw_rad_in_arctan_domain >= 0:
        yaw_rad_in_arccos_domain = yaw_rad_in_arctan_domain
    else:
        yaw_rad_in_arccos_domain = np.pi + yaw_rad_in_arctan_domain

    return yaw_rad_in_arccos_domain

def yaw_angle_rads_alternative(X_pts, Y_pts):
    if X_pts.shape[0] <= 1 and Y_pts.shape[0] <= 1:
        return 0

    n = len(X_pts)
    X1 = X_pts[:n-1]
    Y1 = Y_pts[:n-1]
    X2 = X_pts[1:]
    Y2 = Y_pts[1:]
    
    dY = Y2 - Y1 
    dX = X2 - X1 + 1e-5

    M = dY/dX
    m = np.mean(M)

    yaw_rad_in_arctan_domain = - np.arctan(m)

    if yaw_rad_in_arctan_domain >= 0:
        yaw_rad_in_arccos_domain = yaw_rad_in_arctan_domain
    else:
        yaw_rad_in_arccos_domain = np.pi + yaw_rad_in_arctan_domain

    return yaw_rad_in_arccos_domain    



def draw_circles(image ,x_pts, y_pts, circ_color = (0,0,255)):
    
    # y_pts, x_pts = coordinates[:,0][:], coordinates[:,1][:]
    for i in range(len(y_pts)):
        cv.circle(image, (int(x_pts[i]),int(y_pts[i])), radius = 3, color = circ_color, thickness=-1)

def display_HUD(input_img, lines = None, x_med_estrada = None, beta = 1, region = True ):
    
    # Dimensoes
    shape_mask = input_img.shape
    height, width = shape_mask[0], shape_mask[1]
    x_med_img = int(0.5*width)

    # Central region of the image
    y_max = height
    y_min = 0
    x_max = int(0.60*width)
    x_min = int(0.40*width)

    # mascara
    mask = np.zeros(shape_mask, dtype=np.uint8)

    def HUD():
        
        '''   Mostra na imagem as linhas de referencia e o retangulo da regiao central de interesse    '''
        # linha CENTRAL da IMAGEM (x_linha_media) - linha verde
        cv.line(mask, pt1 = (x_med_img , int((y_max+y_min)*0.5)), pt2 = (x_med_img,y_min), color = (10,200,10),thickness= 1)
        cv.line(mask, pt1 = (x_min , int((y_max+y_min)*0.5)),pt2 = (x_max ,int((y_max+y_min)*0.5)), color =(10,200,10), thickness=1 )
        '''   Mostra a região retangular de interesse   '''
        if region == True:
            retangulo = cv.rectangle(mask, (x_min, y_min), (x_max, y_max), (10,150,10), 1)
    HUD()

    def   display_central_pista():
        ''' mostra na imagem a linha central da pista '''
        if lines is not None and x_med_estrada is not None:
            # Linha CENTRAL da PISTA (aproximadamente)
            cv.line(mask, (x_med_estrada, y_min - 50), (x_med_estrada, y_min + 50), (0,0,255), 2)
            #linhas das extremidades e do centro da pista (azuis)
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv.line(mask, (x1,y1), (x2,y2), (150,0,255), 3)
    display_central_pista()
    
    # Adiciona os elementos anteriores na imagem
    hud_on_img = cv.addWeighted(input_img, 1, mask, beta, 1)
    
    return hud_on_img

def display_line_of_orientation(image, yaw_angle_rads, x_to_compute_distance = None, line_length = 100, beta = 1, put_text = False):
    
    '''
        The "line of orientation" help us to visualize the direction computed by 'yaw_angle_rads'.
        -- In putText, yaw corresponds to the angle (in deg) between the line of orientation and the vertical. That's, 90 - angle_deg 
            
    '''
    height = image.shape[0]
    width = image.shape[1]
    
    mask = np.zeros(image.shape, dtype=np.uint8)

    sin = np.sin(yaw_angle_rads)
    cos = np.cos(yaw_angle_rads)
    
    y1 = int(0.5*height)
    if x_to_compute_distance is not None:
        x1 = int(x_to_compute_distance - 0.5*width)
    else:
        x1 = int(0.5*width)
    
    y2 = y1 - np.int(line_length*sin)
    x2 = x1 + np.int(line_length*cos)

    cv.line(mask, pt1 = (x1,y1), pt2 = (x2, y2), color = (0,0,255),thickness= 2)
    
    line_on_img = cv.addWeighted(image, 1, mask, beta, 1)
    
    if put_text == True:
        font = cv.FONT_HERSHEY_SIMPLEX
        angle_deg = yaw_angle_rads*180/3.14
        text = "yaw= " + str(90 - angle_deg)[:5] + " deg"
        cv.putText(line_on_img, text, (10, height - 20), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
    
    return line_on_img









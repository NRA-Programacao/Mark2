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

def linha_media(dilate_img, minLineLength = 100, maxLineGap = 20):

    # Hough transform para obter as informações das faixas das extremidades
    lines = cv.HoughLinesP(dilate_img, 1, np.pi/180, 100, minLineLength, maxLineGap)

    
    # numero total de pontos (x,y) de 'lines'
    n_points = lines.shape[0]

    # redimensionando a matriz 'lines'
    lines_reshape = np.reshape(lines, (n_points,4))

    # x medio de interesse (aprox a linha do centro da pista): soma dos elementos das colunas 0.5*(sum x1 + sum x2)/n_points
    x_med_road = int(0.5*(np.sum(lines_reshape, axis=0)[0] + np.sum(lines_reshape, axis=0)[2])/n_points)
    
    
    #dimensoes da imagem original
    altura, largura = dilate_img.shape[0], dilate_img.shape[1]
    x_linha_media = int(0.5*largura)

    '''    Distância aproximada entre a linha media da imagem e a linha media da pista: 
            -- distancia > 0 : tracejado da pista à direita da linha media da imagem
            -- distancia < 0 : ''   ''  ''   ''   à esquerda da linha media da imagem
    '''
    distance =  x_med_road - x_linha_media 
    
    linhas_outputs = {"lines": lines, "x_med_road": x_med_road, "distance": distance}

    return linhas_outputs

def masked_img(input_img, lines, x_med_estrada):
    
    # Dimensoes
    shape_mask = input_img.shape
    altura, largura = shape_mask[0], shape_mask[1]
    x_med_img = int(0.5*largura)

    # mascara
    mask = np.zeros(shape_mask, dtype=np.uint8)

    # linha media da imagem (x_linha_media) - linha verde
    cv.line(mask, (x_med_img,0), (x_med_img,altura), (0,255,0), 3)
           
    if lines is not None:
        # aproximadamente a linha media da pista (x_med) - linha vermelha
        
        cv.line(mask, (x_med_estrada, 0), (x_med_estrada, altura), (0,0,255), 2)
        #linhas das extremidades e do centro da pista (azuis)
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv.line(mask, (x1,y1), (x2,y2), (255,0,0), 2)
    
    masked = cv.addWeighted(input_img, 1, mask, 0.9, 1 )
    
    return masked

def display_HUD(input_img, lines = None, x_med_estrada = None, beta = 1, regiao = True ):
    
    # Dimensoes
    shape_mask = input_img.shape
    height, width = shape_mask[0], shape_mask[1]
    x_med_img = int(0.5*width)

    y_max = height
    y_min = int(0.25*height)
    x_max = int(0.75*width)
    x_min = int(0.25*width)

    # mascara
    mask = np.zeros(shape_mask, dtype=np.uint8)

    def HUD():
        
        '''   Mostra na imagem as linhas de referencia e o retangulo da regiao central de interesse    '''
        # linha CENTRAL da IMAGEM (x_linha_media) - linha verde
        cv.line(mask, pt1 = (x_med_img , int((y_max+y_min)*0.5)), pt2 = (x_med_img,y_min), color = (10,200,10),thickness= 1)
        cv.line(mask, pt1 = (x_min , int((y_max+y_min)*0.5)),pt2 = (x_max ,int((y_max+y_min)*0.5)), color =(10,200,10), thickness=1 )
        '''   Mostra a região retangular de interesse   '''
        if regiao == True:
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

def display_line_of_orientation(image, yaw_angle_rads, x_to_compute_distance = None, line_length = 50, beta = 1):
     
    height = image.shape[0]
    width = image.shape[1]
    
    mask = np.zeros(image.shape, dtype=np.uint8)
    
    sin = np.sin(yaw_angle_rads)
    cos = np.cos(yaw_angle_rads)
    
    y1 = int(0.625*height)
    if x_to_compute_distance is not None:
        x1 = int(x_to_compute_distance - 0.5*width)
    else:
        x1 = int(0.5*width)
    
    y2 = y1 - np.int(line_length*sin)
    x2 = x1 + np.int(line_length*cos)

    cv.line(mask, pt1 = (x1,y1), pt2 = (x2, y2), color = (0,0,255),thickness= 2)
    
    line_on_img = cv.addWeighted(image, 1, mask, beta, 1)
    
    return line_on_img

def draw_circles(image ,coordinates):
    
    y_pts, x_pts = coordinates[:,0][:], coordinates[:,1][:]
         
    for i in range(len(y_pts)):
        cv.circle(image, (int(x_pts[i]),int(y_pts[i])),1,(0,0,255),5)
    
def direction_by_10pts(binary_img):

    '''   
        --- direction_by_10pts: 75% of binary_inv image is horizontally sliced (cropped) in 10 parts. We will then compute the non zeros
        indexes of each part. After that, we calculate and return the centers, (y_center, x_center)_i (i = 0, 1,.., 9),  of all 
        those parts. These point's coordinates will be the input of np.polyfit(X_pts, Y_pts, deg = 1) function. 

        -- L: height of the region will be horizontally sliced. That is 75% of image's total height      
        -- dL: height of each sliced part
        -- y0: slices begin at the point (y = y0, x = 0). That is 0.25*height = height - L
        -- pti (i = 0, 1, ..,9): coordinates of (y_center, x_center)_i (WITH RESPECT TO i-th SLICE'S COORDINATE SYSTEM). 
                To correct the coordinates, we need to sum      y_center_i + (y0 + (i-1*dL))
        -- yx_correction: array with the (i-1)*dL values
        -- yx_cg_coordinates: all 10 (y_center, x_center) we will use as input of np.polyfit
    '''

    height = binary_img.shape[0]
    width = binary_img.shape[1]
    
    num_of_slices = 10
    L = int(0.75*height)
    dL = int(L/num_of_slices)
    x_min = int(0.25*width)
    x_max = int(0.75*width)
    
    y0 = height - L
        
    if np.nonzero(binary_img[y0:,:]) is not None:
        pt0 = np.mean(np.nonzero(binary_img[y0+0*dL:y0+dL, x_min:x_max]), axis=1)
        pt1 = np.mean(np.nonzero(binary_img[y0+dL:y0+2*dL, x_min:x_max]), axis=1)
        pt2 = np.mean(np.nonzero(binary_img[y0+2*dL:y0+3*dL, x_min:x_max]), axis=1)
        pt3 = np.mean(np.nonzero(binary_img[y0+3*dL:y0+4*dL, x_min:x_max]), axis=1)
        pt4 = np.mean(np.nonzero(binary_img[y0+4*dL:y0+5*dL, x_min:x_max]), axis=1)
        pt5 = np.mean(np.nonzero(binary_img[y0+5*dL:y0+6*dL, x_min:x_max]), axis=1)
        pt6 = np.mean(np.nonzero(binary_img[y0+6*dL:y0+7*dL, x_min:x_max]), axis=1)
        pt7 = np.mean(np.nonzero(binary_img[y0+7*dL:y0+8*dL, x_min:x_max]), axis=1)
        pt8 = np.mean(np.nonzero(binary_img[y0+8*dL:y0+9*dL, x_min:x_max]), axis=1)
        pt9 = np.mean(np.nonzero(binary_img[y0+9*dL:y0+10*dL, x_min:x_max]), axis=1)

        yx_correction = np.array([[y0 , x_min],[y0 + dL,x_min],[y0 + 2*dL,x_min],[y0 + 3*dL,x_min],[y0 + 4*dL,x_min],
        [y0 + 5*dL,x_min],[y0 + 6*dL,x_min],[y0 + 7*dL,x_min],[y0 + 8*dL,x_min],[y0 + 9*dL,x_min]])
            
        yx_cg_coordinates = np.array([pt0,pt1, pt2, pt3,pt4, pt5, pt6,pt7, pt8, pt9]) + yx_correction
        
        return yx_cg_coordinates
    else:
        print("direction_by_10pts() function: there is no nonzero point detected ")
        return None

def direction_by_lane_center(binary_img, LOWER_AREA = 10, HIGHER_AREA = 1000):

    # Crop binary image for vertical analisys
    bin_center_x = int(binary_img.shape[1]/2)
    lower_xval = int(bin_center_x - binary_img.shape[1]/6)
    upper_xval = int(bin_center_x + binary_img.shape[1]/6)
    y_val = binary_img.shape[0]
    cropped_bin = binary_img[0:y_val,lower_xval:upper_xval]

    cv.imshow("cropped_bin",cropped_bin)

    def moments_pass(moments):
        flag = moments['m00'] > LOWER_AREA and moments['m00'] < HIGHER_AREA
        flag = flag and moments['m10'] != 0
        flag = flag and moments['m01'] != 0 
        return flag

    # Get contours and hierarchy
    contours, hierarchy = cv.findContours(cropped_bin, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    print("Number of contours = " + str(len(contours)))

    if(len(contours) != 0):

        areas = []
        centroids = np.zeros((len(contours),2))
        num_contour = 0

        for i in range(len(contours)):
            moments = cv.moments(contours[int(i)])

            if(moments_pass(moments)):
                print(moments_pass(moments))
                # Cx = m10/m00, Cy = m01/m00
                centroids[num_contour][0] = moments['m10']/moments['m00']
                centroids[num_contour][1] = moments['m01']/moments['m00'] 
                num_contour += 1

        centroids = np.resize(centroids, (num_contour, 2))
        
        x_pts_cv = centroids[:,0][:] + lower_xval 
        y_pts_cv = centroids[:,1][:]

        x_pts_drone = np.zeros((len(centroids)))
        y_pts_drone = np.zeros((len(centroids)))

        # TODO: vectorize
        for i in range(len(centroids)):
            x_pts_drone[i] = x_pts_cv[i]
            y_pts_drone[i] = binary_img.shape[0] - y_pts_cv[i]
 
        #for i in range(len(centroids)):
        #    cv.circle(img0, (int(centroids[i][0]),int(centroids[i][1])),1,(0,0,255),10)    
        
        if x_pts_drone.shape[0] == 0 or y_pts_drone.shape[0] == 0:
            flag = 0
        else:
            flag = 1
        return flag, y_pts_drone, x_pts_drone, y_pts_cv, x_pts_cv 

    flag = 0
    return flag, -1, -1, -1, -1

def filter_points(x_pts, y_pts):

    if x_pts.shape[0] == 0:
        return 0

    coords = np.stack((x_pts, y_pts), axis=1)
    # Get vertical
    iter = 0
    m = 0.75

    x_median = np.median(coords[:,0][:])
    x_std = np.std(coords[:,0][:])
    coords_vert = np.zeros((coords.shape[0],coords.shape[1]))
    points_number = 0
    
    for i in range(len(coords)):
        if(abs(coords[i][0] - x_mean) < m * x_std):
            coords_vert[points_number][0] = coords[i][0]
            coords_vert[points_number][1] = coords[i][1]
            points_number += 1

    coords_vert = np.resize(coords_vert, (points_number, 2))
    # coords_vert = coords[abs(coords[:,0][:]-np.mean(coords[:,0][:])) < m * np.std(coords[:,0][:])]
    return coords_vert #, coords_hor

def yaw_angle_rads(X_pts, Y_pts):
    
    #Y_pts, X_pts = coordinates[:,0][:], coordinates[:,1][:]
    if X_pts.shape[0] <= 1 and Y_pts.shape[0] <= 1:
        return 0

    m,b = np.polyfit(X_pts, Y_pts, deg = 1)
    
    if m >= 0:
        yaw_rad = np.arctan(m)
    
    if m < 0:
        yaw_rad = np.arctan(m) + 3.1415

    return yaw_rad

def add_gaussian_noise(image):
    mean = 0
    var = 0.1
    sigma = var**0.5
    row, col, ch = image.shape
    gauss = np.random.normal(mean, sigma, image.shape)
    gauss = gauss.reshape(row, col, ch)
    noisy = image + gauss
    
    return noisy

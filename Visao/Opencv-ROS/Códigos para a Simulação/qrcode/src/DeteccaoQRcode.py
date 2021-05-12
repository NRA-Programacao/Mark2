#!/usr/bin/env python3
# mudar o python3 para a sua versão do python
from __future__ import print_function

from numpy.lib.utils import info
import rospy  # biblioteca padão para trabalhar com ros em python
import cv2  # biblioteca para trabalhar Opencv
# biblioteca para trabalhar com as std_msgs (strings)

from sensor_msgs.msg import Image
# biblioteca que faz a conversão da imagem publicada em ROS para Opencv
from cv_bridge import CvBridge, CvBridgeError
import sys
import pyzbar.pyzbar as pyzbar
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


bridge = CvBridge()  # objeto que faz a transformação entre as imagens do ROS e Opencv


def decode(im):
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)
    return decodedObjects

# Display barcode and QR code location


def display(im, decodedObjects):
    i = 1
    # Loop over all decoded objects
    for decodedObject in decodedObjects:
        points = decodedObject.polygon
        # If the points do not form a quad, find convex hull
        if len(points) > 4:
            hull = cv2.convexHull(
                np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else:
            hull = points
        # Number of points in the convex hull
        n = len(hull)
        if n != 0:
            i = 0
            X = np.zeros(n)
            Y = np.zeros(n)
            for (x, y) in hull:
                X[i] = x
                Y[i] = y
                i += 1
            m1 = (Y[2]-Y[0])/(X[2]-X[0])
            m2 = (Y[3]-Y[1])/(X[3]-X[1])
            X_encontro = (-(m2*X[1])+Y[1]+(m1*X[0])-Y[0])/(m1-m2)
            Y_encontro = Y[0]+(m1*X_encontro)-(X[0]*m1)
            rospy.init_node('QRcode', anonymous=True)  # inicia o nó
            pub_coord_QRcode = rospy.Publisher("/qrcode/posicao", Pose2D)
            pub_info_QRcode = rospy.Publisher("qrcode/informacao", String)

            coordenada = Pose2D()
            info = String()
            for decodedObject in decodedObjects:
                teste = (decodedObject.data).decode('utf-8')
                print(teste)
                info.data = teste
                pub_info_QRcode.publish(info)
                # print(info.data)
            coordenada.x = X_encontro
            coordenada.y = Y_encontro
            pub_coord_QRcode.publish(coordenada)

        # Draw the convext hull
        for j in range(0, n):
            cv2.line(im, hull[j], hull[(j+1) % n], (255, 0, 0), 3)
        cv2.circle(im, (int(X_encontro), int(Y_encontro)), 5, (255, 0, 0), -1)
        cv2.circle(im, (300, 200), 5, (0, 255, 0), -1)
    # Display results

    cv2.namedWindow("janela", cv2.WINDOW_GUI_EXPANDED)
    cv2.imshow("janela", im)
    cv2.waitKey(1)


# função de callback do SUbscriber
def image_callback(ros_image):
    global bridge
    # c
    try:
        # faz a converção entre a imagem em ROS e a de Opencv bgr de 8 bits
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # a partir de agora a variavel cv_image é uma imagem no padrão Opencv
    # como exemplo é possivel mostrar ela em uma janela com comandos do Opencv
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    threshold = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 12)
    decodedObjects = decode(cv_image)
    display(cv_image, decodedObjects)
    cv2.namedWindow("thresh", cv2.WINDOW_FREERATIO)
    cv2.imshow("thresh", threshold)
    cv2.waitKey(1)


def main(args):
    rospy.init_node('QRcode', anonymous=True)  # inicia o nó
    # para mudar qual tópico ele deve se subscrever basta alterar "/topico_video" pelo nome do tópico desejado
    image_sub = rospy.Subscriber(
        "/downward_cam/camera/image", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

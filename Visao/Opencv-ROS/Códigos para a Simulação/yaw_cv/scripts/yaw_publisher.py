#!/usr/bin/python3

import rospy                                        #biblioteca padão para trabalhar com ros em python
import cv2       
import numpy as np                                  #biblioteca para trabalhar Opencv
from std_msgs.msg import String, Float32            #biblioteca para trabalhar com as std_msgs (strings, float, etc)
from sensor_msgs.msg import Image                   #biblioteca para trabalhar com  as sensor_msgs (imagens)
from cv_bridge import CvBridge, CvBridgeError       #biblioteca que faz a conversão da imagem publicada em ROS para Opencv
import sys
import lines_tools as ltools

bridge = CvBridge()    #objeto que faz a transformação entre as imagens do ROS e Opencv
yaw_cv = Float32()      #o angulo que queremos retornar (yaw) é do tipo Float32

#função de callback do SUbscriber
def yaw_callback(ros_image):
  global bridge
  global yaw_cv
  # print(type(bridge))
  #c
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")    #faz a converção entre a imagem em ROS e a de Opencv bgr de 8 bits
    
    
    '''direction by lane center (moments) method: to compute yaw_cv (in rads)''' 
    blur = cv2.GaussianBlur(cv_image, (5,5), 1)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    # Threshold
    lower_moments = np.array([0, 0, 100])
    upper_moments = np.array([255, 255, 255])
    # imagem binaria:
    mask_moments = cv2.inRange(hsv, lower_moments, upper_moments)
    binary_of_interest = ltools.region_of_interest(mask_moments)
    
    flag_moments, y_pts_moments, x_pts_moments = ltools.direction_by_moments(mask_moments, remove_outliers=True)
    
    if flag_moments == 1:
      yaw_moments = ltools.yaw_angle_rads_alternative(x_pts_moments, y_pts_moments)
      yaw_cv = np.pi/2 - yaw_moments
      ltools.show_img(binary_of_interest, "Threshold image")
      cv2.waitKey(1)
      ## Descomentar as linhas abaixo para visualisar o HUD e o vetor de orientação
      # hud_moments_img = ltools.display_HUD(blur)
      # direction_by_moments_img = ltools.display_line_of_orientation(hud_moments_img, yaw_moments, line_length=150 ,put_text=True )
      # ltools.draw_circles(direction_by_moments_img, x_pts_moments, y_pts_moments)
      # ltools.show_img(direction_by_moments_img, "Direction by moments")
      # cv2.waitKey(1) 
      
      return yaw_cv
  
  except CvBridgeError as e:
    return 0
    print(e)
  #a partir de agora a variavel cv_image é uma imagem no padrão Opencv
  #como exemplo é possivel mostrar ela em uma janela com comandos do Opencv
  
    
def main():
  rospy.init_node('line_follower', anonymous=True)                        #inicia o nó
  #cria o Subscriber que vai receber a imagem
  #para mudar qual tópico ele deve se subscrever basta alterar "/topico_video" pelo nome do tópico desejado  
  rospy.Subscriber("/videofile/image_raw",Image, yaw_callback)
  
  #cria um publisher
  pub = rospy.Publisher("yaw_rads", Float32, queue_size=10)
  
  r = rospy.Rate(10)
  
  while not rospy.is_shutdown():
    
    pub.publish(yaw_cv)
    rospy.loginfo("yaw_cv: %s rads", yaw_cv)
    r.sleep()

    


if __name__ == '__main__':
    main()

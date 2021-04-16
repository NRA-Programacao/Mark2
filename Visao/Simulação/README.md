# Simulação com Hector Quadrotor para ROS-Noetic ou ROS-Kinetic

## Instalação

Clonar o git do Hector Quadrotor e mudar o branch para Noetic (ou Kinetic com a sua versão do gazebo)

		git clone https://bitbucket.org/theconstructcore/hector_quadrotor_sim.git
		cd hector_quadrotor_sim
		git switch noetic
		
Seguir o tutorial abaixo para instalar o pacote para controlar o drone pelo teclado
	
		http://wiki.ros.org/teleop_twist_keyboard
	

## Adaptação do Hector Quadrotor para ter a câmera em baixo

Substiuir os arquivos abaixo pelos originais:     
Primeiro o arquivo

			put_robot_in_world.launch
			
deve ser subsitituído em     ```hector_quadrotor/hector_quadrotor_demo/launch``` e o arquivo		
			
			quadrotor_hokuyo_utm30lx.urdf.xacro
			
em ```hector_quadrotor/hector_quadrotor_description/urdf```. Além disso, copiar o arquivo:

			markII.rviz

para     ```hector_quadrotor/hector_quadrotor_demo/rviz_cfg```
			

## Simulação

No **Terminal 1** :

		roslaunch hector_gazebo_worlds start.launch
		
No **Terminal 2** :
		
		roslaunch hector_quadrotor_demo put_robot_in_world.launch
		
No **Terminal 3** :
		
		rosrun teleop_twist_keyboard teleop_twist_keyboard.py 		

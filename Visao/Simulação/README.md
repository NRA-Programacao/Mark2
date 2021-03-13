Clonar o git do Hector Quadrotor --> https://bitbucket.org/theconstructcore/hector_quadrotor_sim/src/master/

Substiuir os arquivos abaixo pelos originais:
			put_robot_in_world.launch
			em hector_quadrotor/hector_quadrotor_demo/launch
			
			quadrotor_hokuyo_utm30lx.urdf.xacro
			em hector_quadrotor/hector_quadrotor_description/urdf
			
Copiar o arquivo:
			markII.rviz
para
			hector_quadrotor/hector_quadrotor_demo/rviz_cfg
			

No terminal
		Terminal 1 --> roslaunch hector_gazebo_worlds start.launch
		Terminal 2 --> roslaunch hector_quadrotor_demo put_robot_in_world.launch
			

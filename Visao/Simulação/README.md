# Tutorial de Configuração do Workspace de ROS 

Primeiramente, tendo ROS instalado pelo tutorial da ROS Wiki ([Link](http://wiki.ros.org/ROS/Installation)) deve-se criar uma área de trabalho para armazenar e copilar os projetos de ROS de maneira mais prática. Para isso num terminal qualquer temos:

📌 OBS.: Tutorial abaixo baseado na ROS Wiki ([Link](wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)), em um vídeo do Youtube ([Link](https://www.youtube.com/watch?v=iLiI_IRedhI&list=PLuteWQUGtU9BD_vxTEZy8tP8FF4zE2VJH&index=7)) e no pdf da Universidade ETH Zürich ([Link](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2021/lec1/ROS%20Course%20Slides%20Course%201.pdf))


```console
user@pc: ~$ mkdir -p ~/catkin_ws/src
user@pc: ~$ cd ~/catkin_ws/
user@pc: ~/catkin_ws/$ catkin_make
```


O comando _catkin_make_ será sempre usado para compilar os pacotes de ROS criados

⚠️ O comando deve ser usado **SEMPRE** no diretório raíz do ser workspace, no caso, no ``/catkin_ws``, para compilação dos pacotes

Nesse instante dentro do diretório ``/catkin_ws`` temos 2 novos diretórios criados ``/catkin_ws/build/``,``/catkin_ws/devel/`` e o anterior ``/catkin_ws/src/``, sempre que criar ou modificar um pacote ele **DEVE** estar localizado no diretório ``/catkin_ws/src/``, os outros dois armazenam executáveis e aquivos de configuração de ROS

![grafik](https://user-images.githubusercontent.com/70553958/116134484-51ace780-a6a6-11eb-8485-f69ebed28a7f.png)

Em tese, toda a instalação e configuração do ambiente está completa, mas deve-se realçar que no momento **SEMPRE** que abrir um terminal novo, para que o ROS entenda esse workspace criado como de fato um workspace de ROS devemos usar o seguinte comando:

```console
user@pc: ~/catkin_ws/$ source devel/setup.bash
```

Porém há uma adaptação que facilita e sempre reconhece esse workspace criado como o workspace principal de ROS

📌 Os comandos abaixo são opcionais, caso você queira ter mais de um workspace de ROS o melhor é utilizar o comando _source devel/setup.bash_ para cada situação

```console
user@pc: ~/catkin_ws/$ cd ..
user@pc: ~$ gedit .bashrc
```

Após o comando deve-se descer até o **final** do arquivo _bashrc_ e escrever os seguintes comandos para o caso de ter instalado ROS Noetic:

	source /opt/ros/noetic/setup.bash
	
	source /home/YOUR_USER_NAME/catkin_ws/devel/setup.bash

⚠️ Para outras distros de ROS que não a Noetic é só substituir o noetic no primeiro comando por sua distro (O comando genérico seria: ``source /opt/ros/<DISTRO>/setup.bash``)

⚠️ O YOUR_USER_NAME que é o nome que aparece antes do @ em qualquer comando em um terminal, no caso do tutorial seria _user_

Assim, só resta salvar o arquivo _bashrc_ editado e abrir um novo terminal. Para testar se está tudo certo utilize o comando abaixo:

```console
user@pc: ~$ echo $ROS_PACKAGE_PATH
```
E a resposta deve ser do tipo:

	/home/YOUR_USER_NAME/catkin_ws/src:/opt/ros/noetic/share
	

Agora pode-se criar e editar pacotes de ROS. 

Sendo que assim, siga os passos a seguir para instalar o simulador Hector Quadrotor e depois vá para [esse link](https://github.com/NRA-Programacao/Mark2_ros) clonar o repositório com os pacotes de ROS utilizados no Mark II 

# Simulação com Hector Quadrotor para ROS-Noetic ou ROS-Kinetic

## Instalação

Clonar o git do Hector Quadrotor no seu _catkin_ws/src_ e mudar o branch para Noetic (ou Kinetic com a sua versão do gazebo) com:


```console
user@pc: ~$ cd ~/catkin_ws/src/
user@pc: ~/catkin_ws/src/$ git clone https://bitbucket.org/theconstructcore/hector_quadrotor_sim.git
user@pc: ~/catkin_ws/src/$ cd hector_quadrotor_sim
user@pc: ~/catkin_ws/src/hector_quadrotor_sim/$ git switch noetic
```

Recomenda-se seguir o tutorial para instalar o pacote para controlar o drone pelo teclado, Teleop_twist Keyboard ([Link](http://wiki.ros.org/teleop_twist_keyboard))


## Adaptação do Hector Quadrotor para ter a câmera em baixo

Copiar os arquivos presentes nesse diretório do Git do Mark II pelos originais do Hector para isso:     

⚠️ Considerando que você clonou este diretórtio do Mark2 na área de trabalho então é só seguir os comandos abaixo, caso esteja em algum diretório é só acresentar o Path até o diretório em cada comando

### Arquivo put_robot in world

```console
user@pc: ~$ cp /Mark2/Visao/Simulação/put_robot_in_world.launch /catkin_ws/src/hector_quadrotor_sim/hector_quadrotor/hector_quadrotor_demo/launch
```


### Arquivo quadrotor_hokuyo_utm30lx.urdf.xacro

```console
user@pc: ~$ cp /Mark2/Visao/Simulação/quadrotor_hokuyo_utm30lx.urdf.xacro /catkin_ws/src/hector_quadrotor_sim/hector_quadrotor/hector_quadrotor_description/urdf
```


### Arquivo markII.rviz

```console
user@pc: ~$ cp /Mark2/Visao/Simulação/markII.rviz /catkin_ws/src/hector_quadrotor_sim/hector_quadrotor/hector_quadrotor_demo/rviz_cfg
```

Agora é só compilar o simulador Hector Quadrotor no diretório raiz do seu workspace

```console
user@pc: ~$ cd ~/catkin_ws/
user@pc: ~/catkin_ws/$ catkin_make
```
			

## Simulação

Para testar uma simulação própria do Hector Quadrotor pode-se utilizar os seguinte comandos em 3 terminais diferentes:

No **Terminal 1** :

```console
user@pc: ~$ roslaunch hector_gazebo_worlds start.launch
```

		
No **Terminal 2** :

```console
user@pc: ~$ roslaunch hector_quadrotor_demo put_robot_in_world.launch
```

		
No **Terminal 3** :

```console
user@pc: ~$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Se tiver dado tudo certo o drone do Hector estará em um ambiente vazio do Gazebo e poderá ser controlado pelo teclado seguindo as instruções presentes no **Terminal 3** 

Agora, como indicado, pode-se clonar o repositório com os pacotes de ROS utilizados no Mark II [nesse link](https://github.com/NRA-Programacao/Mark2_ros) 


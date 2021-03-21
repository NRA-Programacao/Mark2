# Ambiente da FIRA Air no Gazebo

Após ter alguma distro de ROS instalada e esse repositório adicionado no seu workspace/src local temos as seguintes versões de ambientes da Fira no gazebo

## Versão Final

A versão oficial da Fira com os dois obstáculos, torre elétrica, prédio, campos de pouso (cruz vermelha e heliponto) e todos os QR Codes pode ser obtida com

      roslaunch fira fira.launch

Essa versão (como as outras) segue as especifições apresentadas no edital da FIRA Air - Emergency Service Indoor - 2019 ([link](https://docs.google.com/document/d/1gacUn0bwbEUpGLTWW0aSvrTOMHL1vZxMERJuWrwBnOw/edit)) e indicadas abaixo:
	
![image](https://github.com/NRA-Programacao/Mark2/blob/master/Visao/Fira/FIRA_Cenario.jpg)

## Versões de Testes

Para poder testar missões separadamente, primeiro devemos abrir um mundo vazio no gazebo:

      roslaunch fira empty_world.launch

E então, em outro terminal, usar um ou mais dos comandos abaixos para cada uma das missões:



### 1. _Visual navigation_
Ruas para Navegação do Drone, ainda sem os QR Code

      roslaunch fira Spawn_ruas_sem_qr.launch

❗ Recomendo utilar essa versão SOZINHA, se quiser testar a navegação com outras missões usar a versão abaixo com os pousos

### 2. _Delivering the first aid kit_ e _Return and land on starting point_
Ruas para Navegação do Drone com todos os QR Codes de orientação, Cruz Vermelha e Heliponto Azul

      roslaunch fira Spawn_ruas_e_pousos.launch
   
      
### 3. _Collision avoidance_
Traves Vermelhas e Amarelas em meio ao percurso
     
      roslaunch fira Spawn_obstacles.launch 
 
### 4. _Insulator scan in electrical tower_ e _Searching for victims in tall building_
Torre elétrica e Prédio alto com QR Codes para inspeção
   
      roslaunch fira Spawn_torres.launch 
    

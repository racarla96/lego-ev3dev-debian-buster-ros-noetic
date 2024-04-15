# Imagen base de EV3Dev - Debian 10 Buster para LEGO EV3
FROM ev3dev/ev3dev-buster-ev3-generic:2020-04-10

# Cambia al directorio home del usuario robot
WORKDIR /home/robot

# Inclusión de los repositorios de ROS y Debian 10 Buster
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo sh -c 'echo "deb http://deb.debian.org/debian buster main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb-src http://deb.debian.org/debian buster main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb http://deb.debian.org/debian buster-updates main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb-src http://deb.debian.org/debian buster-updates main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb http://deb.debian.org/debian buster-backports main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb-src http://deb.debian.org/debian buster-backports main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb http://security.debian.org/debian-security/ buster/updates main contrib non-free" >> /etc/apt/sources.list'
RUN sudo sh -c 'echo "deb-src http://security.debian.org/debian-security/ buster/updates main contrib non-free" >> /etc/apt/sources.list'

# Paquete para la gestión de las claves públicas de los repositorios
RUN sudo apt-get install -y gnupg1

# Introducción de las claves públicas de los repositorios de ROS y Debian buster-backports 
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 0E98404D386FA1D9 6ED0E7B82643E131

# Actualización de la información de los repositorios y paquetes
RUN sudo apt-get --allow-releaseinfo-change update
RUN sudo apt-get upgrade -y

# Instalación de los certificados para comunicarnos por SSL
RUN sudo apt-get install ca-certificates -y

# Instalación del paquete pip para Python3
RUN sudo apt-get install python3-pip -y

# Instalación de los paquetes de ROS necesarios para la compilación
RUN sudo pip3 install -U rosdep rosinstall_generator rosinstall catkin_pkg rospkg vcstool

# Cambia al directorio home del usuario robot
WORKDIR /home/robot

# Descarga el paquete SBCL (necesario posteriormente) usando wget para posteriormente compilarlo e instalarlo
RUN wget http://netcologne.dl.sourceforge.net/project/sbcl/sbcl/1.2.7/sbcl-1.2.7-armel-linux-binary.tar.bz2

# Descomprime el archivo descargado
RUN tar -xjf sbcl-1.2.7-armel-linux-binary.tar.bz2

# Cambia al directorio recién creado
WORKDIR /home/robot/sbcl-1.2.7-armel-linux

# Ejecuta el script de instalación con el directorio de instalación especificado
RUN sudo INSTALL_ROOT=/usr/local bash install.sh

# Vuelve al directorio anterior
WORKDIR /home/robot

# Inicio del repositorio de ROS
RUN rosdep init

# Añadiendo repositorios a rosdep
RUN sudo sh -c 'echo "yaml https://raw.githubusercontent.com/moriarty/ros-ev3/master/ev3dev.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list'

# Actualiza rosdep
RUN sudo rosdep update

# Crea un directorio ros_catkin_ws en el directorio home del usuario
RUN sudo mkdir -p /home/robot/ros_catkin_ws
WORKDIR /home/robot/ros_catkin_ws

# Genera un archivo .rosinstall para instalar ros_core
RUN sudo rosinstall_generator ros_comm common_msgs --rosdistro noetic --deps --tar --wet-only > noetic-ros_comm.rosinstall

# Crea un directorio src en el directorio ros_catkin_ws
RUN sudo mkdir -p /home/robot/ros_catkin_ws/src

# Importa los paquetes de ros_core al directorio src
RUN sudo vcs import --input noetic-ros_comm.rosinstall /home/robot/ros_catkin_ws/src

# Instala las dependencias de los paquetes en el directorio src
RUN sudo rosdep install --from-paths /home/robot/ros_catkin_ws/src --ignore-packages-from-source --rosdistro noetic -y --os=debian:buster --skip-keys sbcl --skip-keys python3-catkin-pkg-modules --skip-keys python3-rosdep-modules

# Construye los paquetes en el directorio src
RUN sudo /home/robot/ros_catkin_ws/src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 

# Añade la configuración de ros_catkin_ws al archivo .bashrc
RUN sudo sh -c 'echo "source /home/robot/ros_catkin_ws/install_isolated/setup.bash" >> /home/robot/.bashrc'

# Cambia al directorio home del usuario robot
WORKDIR /home/robot

# Clonar el repositorio con la libería de lenguaje C++ para LEGO EV3
RUN git clone https://github.com/ddemidov/ev3dev-lang-cpp

# Cambia al directorio del repositorio
WORKDIR /home/robot/ev3dev-lang-cpp

# Creamos el directorio para la compilación
RUN mkdir build

# Cambia al directorio de build
WORKDIR /home/robot/ev3dev-lang-cpp/build

# Generamos la dependencias
RUN cmake ..

# Compilamos
RUN make

# Instalamos en el sistema
RUN sudo make install
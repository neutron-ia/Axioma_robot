
# Axioma.io   :hugs: :muscle: :seedling: :nerd_face:

Este proyecto de grado aborda el desarrollo de software con el sistema operativo de robots ROS2 para convertir la plataforma robótica móvil Axioma.io desarrollado por estudiantes del semillero de robótica SIRO en una plataforma autónoma con la capacidad de percibir y entender el entorno de trabajo en el que se encuentre y pueda calcular una ruta para desplazarse de un punto de origen a un punto final llevando a bordo algún producto, todo esto sin intervención de un operario y sirviendo así como una solución a la automatización de la logística en cadenas y/o procesos de producción cumpliendo los requerimientos de la industria en el suministro, planificación , gestión y control del almacenamiento de mercancía para conseguir los niveles más altos de servicio, calidad y eficiencia al menor tiempo y costo posible. 

#### palabras clave

Robot móvil, autónomo, logística, planeación, trayectorias. 

#### Objetivo general

*Diseñar, simular e implementar software de planificación de trayectorias robóticas para la plataforma de robótica móvil Axioma.io con el fin de cumplir con la función de transportar productos de forma autónoma desde un punto inicial a un punto final dentro de un espacio de trabajo determinado, garantizando la automatización de la logística en la gestión y coordinación de estas actividades dentro de una cadena o proceso de producción.*

##### Objetivos específicos

* Diseñar un entorno de trabajo tridimensional que simula el área de trabajo con obstáculos.  
* Instrumentar el robot virtual con los sensores encargados de orientación, posición, navegación y mapeo.
* Programar el ecosistema de ROS con todos paquetes de los nodos necesarios para la localización, control y navegación, así como también el mapeo del entorno de trabajo del robot. 
* Desarrollar las técnicas de planeación de trayectorias robóticas que indique el camino para que el robot pueda ir de un punto a otro. 
* Integrar el software desarrollado para la localización, control, navegación, mapeo y planificación de trayectorias  en el robot físico Axioma.io. 
Agregar sensores para realizar odometría y cálculo de velocidad de giros del robot.  

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/axioma.jpeg" alt="Axioma Robot" width="400">
</div>

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/open_software.jpeg" alt="Open Source" width="400">
</div>

## 🎥 Demostración del Proyecto

### Videos del Sistema Funcionando

<div align="center">

| **Navegación Autónoma** | **SLAM y Mapeo** |
|:------------------------:|:-----------------:|
| [![Axioma Navigation Part 1](https://img.youtube.com/vi/U28n4vSAwDk/0.jpg)](https://youtu.be/U28n4vSAwDk) | [![Axioma SLAM Part 2](https://img.youtube.com/vi/A-7UMoYXUBQ/0.jpg)](https://youtu.be/A-7UMoYXUBQ) |
| *Navegación en entorno con mapa previamente cargado* | *Robot navegando con LIDAR creando mapa en tiempo real* |

| **Sensores y Frames** | **Ensamblaje 3D** |
|:---------------------:|:-----------------:|
| [![Axioma Sensors Part 3](https://img.youtube.com/vi/dHnnpMOO5yg/0.jpg)](https://youtu.be/dHnnpMOO5yg) | [![Axioma Assembly](https://img.youtube.com/vi/buS84GiqQug/0.jpg)](https://youtu.be/buS84GiqQug) |
| *Robot en movimiento, sensores en RViz y frames* | *Ensamblaje del robot en Autodesk Inventor* |

| **Concurso Mercury Robotics** | **Plataforma Teleoperada** |
|:-----------------------------:|:--------------------------:|
| [![Mercury Challenge 2019](https://img.youtube.com/vi/8E0mYynNUog/0.jpg)](https://youtu.be/8E0mYynNUog) | [![Axioma Teleop](https://img.youtube.com/vi/sHgdL3dffgw/0.jpg)](https://youtu.be/sHgdL3dffgw) |
| *Axioma One en Mercury Robotics Challenge 2019* | *Robot teleoperado con Raspberry Pi y Flask* |

</div>

### 🏆 Características Destacadas en los Videos

- **Navegación Autónoma**: Planificación de rutas y evitación de obstáculos
- **SLAM en Tiempo Real**: Mapeo simultáneo y localización con LIDAR
- **Visualización en RViz**: Monitoreo de sensores y transformadas
- **Diseño Mecánico**: Estructura robusta para aplicaciones industriales
- **Control Remoto**: Interfaz web para teleoperación

---

## 📋 Tabla de Contenidos

- [Hardware](#1-hardware)
- [Software](#2-software)
- [Arquitectura ROS2](#arquitectura-ros2)
- [Conceptos Fundamentales de ROS2](#conceptos-fundamentales-de-ros2)
- [Modelo Matemático](#4-modelo-matemático)
- [Instalación y Configuración](#instalación-y-configuración)
- [Comandos para Ejecutar el Proyecto](#comandos-para-ejecutar-el-proyecto)
- [Desarrollo con ROS2](#desarrollo-con-ros2)

## 1 Hardware

El hardware se encuentra compuesto por dos herramientas open hardware muy usadas en el desarrollo y prototipado rapido de dispositivos electrónicos y mecátronicos y una raspberry pi. Estos dispositivos estan clasificados ***"One Chip"*** por todo en uno solo como lo es arduino que posee un microcontrolador, chip para la comunicación serial, reguladores de voltajes y demas componentes electronicos que permitan conectar actuadores y sensores de forma facil y rapida. En la clasificación One Chip tambien tenemos lo que es la raspberry pi que lleva a bordo un chip microprocesador, ram, video, ethernet/wifi, regulador, comunicación serial que le permiten conectar otros dispositivos como camaras, monitores y toda clase de perifericos usb que le brindan a esta pequeña tarjeta la posibilidad de crear muchas aplicaciones web, IoT, Entretenimiento y Robotica.

### Sensores y actuadores

#### LIDAR
El sensor LIDAR (Light Detection and Ranging) es fundamental para la navegación autónoma del robot Axioma.io. Este sensor utiliza pulsos de luz láser para medir distancias y crear un mapa 2D del entorno.

**Características técnicas:**
- Rango de detección: 0.1m - 10m
- Resolución angular: 1°
- Frecuencia de escaneo: 10 Hz
- Interfaz: USB/Serial
- Campo de visión: 360°

**Aplicaciones en Axioma:**
- Detección de obstáculos
- Mapeo SLAM
- Localización
- Navegación autónoma

#### Cámara
Sistema de visión por computadora para reconocimiento de objetos y navegación visual.

**Especificaciones:**
- Resolución: 640x480 px
- Frame rate: 30 fps
- Interfaz: USB 2.0
- Formato: RGB/BGR
- Ángulo de visión: 60°

**Funcionalidades:**
- Reconocimiento de objetos
- Seguimiento de líneas
- Detección de marcadores ArUco
- Navegación visual

#### Encoders
Sensores de posición rotativa instalados en cada rueda para odometría precisa.

**Características:**
- Tipo: Encoder incremental óptico
- Resolución: 1000 PPR (Pulsos Por Revolución)
- Salida: Cuadratura (A/B)
- Voltaje de operación: 5V
- Frecuencia máxima: 200 kHz

**Datos proporcionados:**
- Posición angular de las ruedas
- Velocidad de rotación
- Dirección de giro
- Distancia recorrida

#### Motores DC
Sistema de tracción diferencial con dos motores DC con reductores.

**Especificaciones técnicas:**
- Voltaje nominal: 12V DC
- Corriente nominal: 2A
- Velocidad sin carga: 180 RPM
- Torque nominal: 5 kg⋅cm
- Relación de reducción: 1:30
- Eficiencia: 85%

### Fuente de alimentación

El sistema de alimentación está diseñado para proporcionar energía estable y confiable a todos los componentes del robot.

**Configuración del sistema:**
- **Batería principal**: Li-Po 3S (11.1V nominal, 12.6V máximo)
- **Capacidad**: 5000 mAh
- **Reguladores de voltaje**:
  - 12V → 5V (5A) para Raspberry Pi y sensores
  - 12V → 3.3V (2A) para Arduino y periféricos
- **Protecciones**: Fusibles, circuitos de protección contra sobrecorriente
- **Autonomía estimada**: 4-6 horas de operación continua

**Distribución de corriente:**
| Componente | Voltaje | Corriente | Potencia |
|------------|---------|-----------|----------|
| Raspberry Pi 4 | 5V | 1.5A | 7.5W |
| Arduino Mega | 5V | 0.5A | 2.5W |
| Motores DC (x2) | 12V | 4A | 48W |
| LIDAR | 5V | 0.8A | 4W |
| Cámara | 5V | 0.3A | 1.5W |
| **Total** | - | **7.1A** | **63.5W** |

### Micros

#### [Arduino](https://www.arduino.cc/)

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/arduino.jpeg" alt="Arduino" width="350">
</div>

**Arduino Mega 2560** actúa como unidad de control de bajo nivel, encargándose de:

**Funciones principales:**
- Control PWM de motores DC
- Lectura de encoders con interrupciones
- Comunicación serial con Raspberry Pi
- Control de actuadores auxiliares
- Monitoreo de sensores analógicos

**Especificaciones:**
- Microcontrolador: ATmega2560
- Voltaje de operación: 5V
- Pines digitales: 54 (15 PWM)
- Pines analógicos: 16
- Memoria Flash: 256 KB
- SRAM: 8 KB
- EEPROM: 4 KB
- Frecuencia de reloj: 16 MHz

**Código ejemplo para control de motores:**
```cpp
// Control de motores con PWM
#define MOTOR_L_PWM 5
#define MOTOR_R_PWM 6
#define MOTOR_L_DIR 7
#define MOTOR_R_DIR 8

void setMotorSpeeds(int left_speed, int right_speed) {
    digitalWrite(MOTOR_L_DIR, left_speed > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_DIR, right_speed > 0 ? HIGH : LOW);
    analogWrite(MOTOR_L_PWM, abs(left_speed));
    analogWrite(MOTOR_R_PWM, abs(right_speed));
}
```

#### [Raspberry Pi](https://www.raspberrypi.com/)

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/raspberry.jpeg" alt="Raspberry Pi" width="350">
</div>

**Raspberry Pi 4 Model B** funciona como cerebro principal del robot, ejecutando ROS2 y algoritmos de alto nivel.

**Especificaciones:**
- SoC: Broadcom BCM2711 (Cortex-A72 64-bit)
- CPU: 4 núcleos a 1.5GHz
- RAM: 4GB LPDDR4
- Conectividad: WiFi 802.11ac, Bluetooth 5.0, Ethernet Gigabit
- USB: 4 puertos USB 3.0/2.0
- GPIO: 40 pines
- Almacenamiento: MicroSD 64GB

**Responsabilidades principales:**
- Ejecución del stack completo de ROS2
- Procesamiento de imágenes y LIDAR
- Algoritmos de navegación y SLAM
- Comunicación inalámbrica
- Interfaz web de control
- Logging y telemetría

### Diseño y Modelado 3D

El robot Axioma.io cuenta con un diseño mecánico completo desarrollado en software CAD profesional, permitiendo la fabricación y prototipado de todos sus componentes.

#### Modelo 3D Interactivo

<div align="center">
  <a href="https://www.autodesk.com/community/gallery/project/147581/robot-axioma-io-with-raspberry-pi-and-python">
    <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/axioma.jpeg" alt="Modelo 3D Axioma.io" width="400">
  </a>
  <br>
  <strong><a href="https://www.autodesk.com/community/gallery/project/147581/robot-axioma-io-with-raspberry-pi-and-python">🔗 Ver Modelo 3D Interactivo en Autodesk Gallery</a></strong>
</div>

#### Características del Diseño

- **Plataforma modular** para fácil ensamblaje y mantenimiento
- **Estructura robusta** optimizada para aplicaciones industriales
- **Compartimentos específicos** para Raspberry Pi, Arduino y sensores
- **Sistema de montaje** para LIDAR y cámara
- **Chasis diferencial** para tracción de dos ruedas
- **Materiales**: Aluminio, PLA/ABS para impresión 3D
- **Herramientas CAD**: Autodesk Inventor, Fusion 360

#### Archivos de Diseño

| Componente | Descripción | Formato |
|------------|-------------|---------|
| **Chasis Principal** | Estructura base del robot | `.ipt`, `.stl` |
| **Soportes de Sensores** | Montajes para LIDAR y cámara | `.ipt`, `.stl` |
| **Carcasa Electrónica** | Protección para PCBs | `.ipt`, `.stl` |
| **Sistema de Tracción** | Acoples para motores y ruedas | `.ipt`, `.stl` |
| **Ensamble Completo** | Modelo integrado | `.iam`, `.step` |

💡 **Nota**: Los archivos CAD están disponibles para modificación y mejora por parte de la comunidad maker.

## 2 Software

### 2.1 [ROS/ROS2](https://www.ros.org/)  

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/ros.jpeg" alt="ROS2" width="400">
</div>

[source](https://github.com/ros)

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/service.gif" alt="ROS2 Service" width="400">
</div>

### [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html)

**ROS2** (*Robot Operating System 2*) es una plataforma de código abierto diseñada para facilitar el desarrollo, operación y mantenimiento de sistemas robóticos y de automatización industrial. Ofrece una arquitectura modular y flexible que permite la comunicación entre componentes distribuidos, soportando una variedad de sistemas operativos y arquitecturas de hardware. ROS 2 se destaca por su capacidad de escalabilidad, seguridad y robustez, lo que lo convierte en una herramienta crucial para la creación de sistemas robóticos avanzados en diversos entornos industriales y de investigación.

#### Historia de ROS2

**ROS** en su primera versión, **ROS1**, se desarrolló en los Laboratorios de Inteligencia Artificial de Stanford (SAIL) por estudiantes de doctorado **Eric Berger** y **Keenan Wyrobek**. Se publicó bajo una **licencia BSD** de software libre en 2007, que permite libertad para uso comercial e investigador. Desde 2008, el instituto **Willow Garage** se ha encargado principalmente del desarrollo y soporte.

La idea de crear un sistema operativo era estandarizar tareas como la *abstracción de hardware*, *control de dispositivos* de bajo nivel (drivers), implementación de *procesos comunes*, manejo de *comunicación*, *soporte* de paquetes y otras ventajas.

**ROS2** es la evolución natural del exitoso marco de trabajo **ROS1**. Desarrollado para abordar las limitaciones de su predecesor, ROS2 ofrece una *arquitectura modular* y *distribuida*, mejor *rendimiento* y *escalabilidad*, así como soporte *multiplataforma*. Lanzado oficialmente en 2015, ROS2 mantiene la *flexibilidad* y *robustez* de ROS1, al tiempo que introduce mejoras significativas en herramientas de desarrollo y comunicación. Su diseño modular permite una fácil integración con otros sistemas y una adaptación más rápida a diferentes entornos de desarrollo. Con características como compatibilidad con múltiples lenguajes de programación y una creciente comunidad de desarrolladores, ROS2 es la elección preferida para proyectos de robótica modernos y ambiciosos.

#### Filosofía
***"ROS, nacido del corazón del código abierto, ofrece libertad y flexibilidad para que los usuarios moldeen su propia realidad robótica, trazando un camino lleno de posibilidades infinitas en el vasto horizonte de la tecnología"***.

#### Diferencias entre ROS1 y ROS2

| Característica        | ROS1          | ROS2        |
|-----------------------|---------------|-------------|
| **Arquitectura**  | Basada en un sistema de nodos con comunicación XML-RPC y TCP/IP | Arquitectura modular y distribuida, comunicación basada en DDS    |
| **Lenguajes de Programación** | Soporte para C++, Python, Lisp, entre otros                   | Soporte para varios lenguajes, incluyendo C++, Python, y más      |
| **Rendimiento** | Limitaciones en rendimiento, seguridad y escalabilidad         | Mejoras significativas en rendimiento, seguridad y escalabilidad  |
| **Multiplataforma** | Principalmente enfocado en Linux                               | Soporte multiplataforma incluyendo Linux, Windows, y macOS        |
| **Herramientas**  | Herramientas de desarrollo y depuración limitadas              | Mejoras en herramientas de depuración, simulación, y gestión de paquetes |
| **Compatibilidad**  | No es directamente compatible con ROS 2                        | Introduce puentes y herramientas de migración para la compatibilidad con ROS 1 |
| **Ecosistema**  | Ecosistema consolidado con una amplia comunidad                 | Ecosistema en constante crecimiento con una creciente comunidad de desarrolladores |

## Arquitectura ROS2

La arquitectura de ROS2 se ha diseñado para abordar las limitaciones de ROS1 y proporcionar una plataforma más flexible, escalable y robusta para el desarrollo de aplicaciones robóticas. A continuación, se proporciona una explicación paso a paso de la arquitectura de ROS2:

| Paso  | Descripción  |
|-------|----------------|
| 1. Arquitectura Modular y Distribuida | ROS 2 se basa en una arquitectura modular y distribuida, donde los nodos son componentes independientes que pueden ejecutarse de manera separada.          |
| 2. Comunicación Basada en DDS | Utiliza DDS para la comunicación entre nodos, ofreciendo un rendimiento superior, mayor seguridad y mejor escalabilidad que el sistema de ROS 1.            |
| 3. Nodos                     | Cada nodo en ROS 2 es un proceso independiente que realiza una tarea específica y se comunica con otros nodos intercambiando mensajes a través de DDS.     |
| 4. Middleware (DDS)          | DDS actúa como el middleware que facilita la comunicación entre nodos, proporcionando mecanismos eficientes para la publicación y suscripción de mensajes. |
| 5. Interfaces de Mensajería (IDL) | Utiliza interfaces de definición de lenguaje (IDL) para describir la estructura de los mensajes que se intercambian entre nodos.                        |
| 6. Gestión de Recursos       | Incluye una capa de gestión de recursos para asignar y administrar eficientemente los recursos del sistema, como memoria y procesamiento.                   |
| 7. Soporte Multiplataforma   | Diseñado para ser ejecutado en una variedad de sistemas operativos, incluyendo Linux, Windows y macOS, lo que proporciona mayor flexibilidad y portabilidad.  |

En resumen, la arquitectura de ROS2 se caracteriza por su modularidad, su sistema de comunicación basado en DDS, su soporte multiplataforma y su capacidad para gestionar eficientemente los recursos del sistema. Estas características hacen de ROS2 una plataforma poderosa y versátil para el desarrollo de aplicaciones robóticas modernas.

## Conceptos Fundamentales de ROS2

### NODOS

Los nodos son bloques de código (clases) que se encargan de partes específicas de las actividades del robot. Estos se van a enlazar mediante tópicos, servicios o acciones. Básicamente nos ayudan a crear un sistema modular que se pueda modificar fácilmente y comunicar.

#### Comandos básicos para nodos:
```bash
# Ejecutar un nodo
ros2 run <paquete> <nodo>

# Visualizar nodos en ejecución
ros2 node list

# Información de un nodo
ros2 node info <nombre_nodo>

# Cambiar nombre del nodo
ros2 run <paquete> <nodo> --ros-args --remap __node:=<nuevo_nombre>
```

#### Ejemplo práctico con turtlesim:

1. **Ejecutar un nodo:**
```bash
ros2 run turtlesim turtlesim_node
```
Este comando lanza el nodo que mediante rqt lanza una interfaz gráfica con una tortuga en unas coordenadas específicas.

2. **Ejecutar un segundo nodo:**
```bash
ros2 run turtlesim turtle_teleop_key
```

3. **Visualizar nodos en ejecución:**
```bash
ros2 node list
```
Resultado:
```
/teleop_turtle
/turtlesim
```

4. **Información detallada de un nodo:**
```bash
ros2 node info /turtlesim
```
Resultado:
```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

### TÓPICOS

Son canales en los cuales unos nodos publican información y otros se suscriben para recibirla. La relación para la comunicación puede ser de *muchos a uno*(one to many), *muchos a uno*(many to one) y *muchos a muchos*(many to many).

#### Características de los tópicos

- **Definición de Tópicos**: Canales de comunicación identificados por un nombre único.
- **Tipos de Mensajes**: Los mensajes transmitidos a través de los tópicos pueden ser de tipos estándar (std_msgs) o personalizados.
- **Publicación y Suscripción**: Los nodos pueden publicar o suscribirse a un tópico para enviar o recibir mensajes.
- **Comunicación Desacoplada**: La comunicación se realiza de forma asíncrona y desacoplada entre nodos.
- **Calidad de Servicio (QoS)**: Configuraciones de QoS permiten ajustar la durabilidad, fiabilidad, latencia, entre otros aspectos de la comunicación.
- **Jerarquía de Nombres de los Tópicos**: Los nombres de los tópicos pueden ser jerárquicos para organizar la información.
- **Tópicos Privados**: Los nodos pueden usar tópicos privados para encapsular la comunicación dentro de un nodo o grupo de nodos.

#### Comandos básicos para tópicos:
```bash
# Listar tópicos
ros2 topic list

# Listar tópicos con tipos
ros2 topic list -t

# Ver información de un tópico
ros2 topic info <nombre_topico>

# Escuchar mensajes de un tópico
ros2 topic echo <nombre_topico>

# Publicar en un tópico
ros2 topic pub <nombre_topico> <tipo_mensaje> '<datos>'

# Ver frecuencia de publicación
ros2 topic hz <nombre_topico>

# Ver estructura del mensaje
ros2 interface show <tipo_mensaje>
```

#### Tipos de Mensajes Estándar

##### **std_msgs**: Mensajes estándar básicos
- **std_msgs/String**: Un mensaje de texto simple
- **std_msgs/Int32**: Un entero de 32 bits
- **std_msgs/Float32**: Un número de punto flotante de 32 bits

##### **geometry_msgs**: Mensajes de geometría y movimiento

**geometry_msgs/Twist** - *Crucial para Axioma*
```yaml
# Estructura del mensaje de velocidad
Vector3 linear
    float64 x    # Velocidad lineal hacia adelante/atrás (m/s)
    float64 y    # Velocidad lineal lateral (m/s) 
    float64 z    # Velocidad lineal vertical (m/s)
Vector3 angular
    float64 x    # Velocidad angular en X (rad/s)
    float64 y    # Velocidad angular en Y (rad/s)
    float64 z    # Velocidad angular en Z (rad/s)
```

**Ejemplo de uso en Axioma:**
```bash
# Mover el robot hacia adelante a 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar el robot a 0.3 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

**geometry_msgs/Pose** - *Posición y orientación*
```yaml
Point position
    float64 x    # Posición X (metros)
    float64 y    # Posición Y (metros)
    float64 z    # Posición Z (metros)
Quaternion orientation
    float64 x    # Componente X del cuaternión
    float64 y    # Componente Y del cuaternión
    float64 z    # Componente Z del cuaternión
    float64 w    # Componente W del cuaternión
```

##### **sensor_msgs**: Mensajes relacionados con sensores

**sensor_msgs/LaserScan** - *Datos del LIDAR*
```yaml
Header header
    builtin_interfaces/Time stamp
    string frame_id          # Frame de referencia del sensor
float32 angle_min            # Ángulo mínimo de escaneo (rad)
float32 angle_max            # Ángulo máximo de escaneo (rad)
float32 angle_increment      # Incremento angular entre mediciones (rad)
float32 time_increment       # Tiempo entre mediciones (segundos)
float32 scan_time           # Tiempo para completar un escaneo (segundos)
float32 range_min           # Distancia mínima válida (metros)
float32 range_max           # Distancia máxima válida (metros)
float32[] ranges            # Array de distancias medidas (metros)
float32[] intensities       # Array de intensidades de retorno
```

**Ejemplo de datos LIDAR del Axioma:**
```bash
# Escuchar datos del LIDAR
ros2 topic echo /scan

# Resultado típico:
header:
  stamp:
    sec: 1634567890
    nanosec: 123456789
  frame_id: "laser_frame"
angle_min: -3.14159265359
angle_max: 3.14159265359
angle_increment: 0.0174532925199
range_min: 0.1
range_max: 10.0
ranges: [2.3, 2.4, 2.5, 2.6, inf, 1.8, ...]
intensities: []
```

##### **nav_msgs**: Mensajes de navegación

**nav_msgs/Odometry** - *Odometría del robot*
```yaml
Header header
    builtin_interfaces/Time stamp
    string frame_id                    # Frame de odometría (típicamente "odom")
string child_frame_id                  # Frame del robot (típicamente "base_link")
geometry_msgs/PoseWithCovariance pose
    Pose pose
        Point position                 # Posición estimada
        Quaternion orientation         # Orientación estimada
    float64[36] covariance            # Matriz de covarianza 6x6
geometry_msgs/TwistWithCovariance twist
    Twist twist
        Vector3 linear                # Velocidad lineal
        Vector3 angular               # Velocidad angular
    float64[36] covariance           # Matriz de covarianza 6x6
```

**Ejemplo de datos de odometría del Axioma:**
```bash
# Escuchar odometría
ros2 topic echo /odom

# Resultado típico:
header:
  stamp:
    sec: 1634567890
    nanosec: 123456789
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  pose:
    position:
      x: 1.23
      y: 0.45
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.1736
      w: 0.9848
  covariance: [0.1, 0.0, 0.0, ...]
twist:
  twist:
    linear:
      x: 0.2
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1
  covariance: [0.05, 0.0, 0.0, ...]
```

**nav_msgs/OccupancyGrid** - *Mapa de ocupación*
```yaml
Header header
MapMetaData info
    builtin_interfaces/Time map_load_time
    float32 resolution              # Resolución del mapa (m/pixel)
    uint32 width                   # Ancho del mapa (pixels)
    uint32 height                  # Alto del mapa (pixels)
    Pose origin                    # Origen del mapa en el mundo
int8[] data                        # Datos del mapa (-1: desconocido, 0: libre, 100: ocupado)
```

### SERVICIOS

Los servicios son un mecanismo de comunicación que permite a los nodos intercambiar datos de forma *síncrona*. Un nodo (el servidor) puede ofrecer una funcionalidad específica que otros nodos (los clientes) pueden solicitar.

#### Comandos básicos para servicios:
```bash
# Listar servicios
ros2 service list

# Listar servicios con tipos
ros2 service list -t

# Ver tipo de un servicio
ros2 service type <nombre_servicio>

# Llamar a un servicio
ros2 service call <nombre_servicio> <tipo_servicio> '<datos>'

# Ver estructura del servicio
ros2 interface show <tipo_servicio>
```

#### Clasificación de Servicios

**Servicios estándar (std_srvs):**
- **Empty**: Sin datos de solicitud ni respuesta
- **SetBool**: Toma un booleano y devuelve éxito/fallo
- **Trigger**: Sin solicitud, devuelve éxito/fallo con mensaje

**Servicios del sistema (rcl_interfaces):**
- **SetParameters**: Configurar parámetros de un nodo
- **GetParameters**: Obtener parámetros de un nodo
- **ListParameters**: Listar parámetros disponibles

### ACCIONES

Las acciones en ROS 2 permiten a los nodos ejecutar tareas complejas de forma asíncrona, con retroalimentación y capacidad de cancelación. Son útiles para operaciones que requieren tiempo y seguimiento.

#### Componentes de una acción:
1. **Goal**: El objetivo que el cliente envía al servidor
2. **Result**: El resultado final que el servidor devuelve al cliente
3. **Feedback**: Información intermedia durante la ejecución

#### Comandos básicos para acciones:
```bash
# Listar acciones
ros2 action list

# Listar acciones con tipos
ros2 action list -t

# Ver información de una acción
ros2 action info <nombre_accion>

# Enviar un objetivo
ros2 action send_goal <nombre_accion> <tipo_accion> '<datos>'

# Enviar objetivo con feedback
ros2 action send_goal <nombre_accion> <tipo_accion> '<datos>' --feedback
```

### INTERFACES

En ROS 2, las interfaces definen cómo se comunican los nodos entre sí mediante mensajes, servicios o acciones.

#### Comandos para interfaces:
```bash
# Listar todas las interfaces
ros2 interface list

# Mostrar estructura de una interfaz
ros2 interface show <nombre_interfaz>

# Listar solo interfaces de un tipo
ros2 interface list | grep msg
ros2 interface list | grep srv
ros2 interface list | grep action
```

### LAUNCH FILES

Los launch files son scripts en Python que permiten iniciar y configurar múltiples nodos simultáneamente.

#### Ejemplo de launch file para Axioma:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Argumentos del launch
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Nodo del robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        
        # Nodo del LIDAR
        Node(
            package='axioma_drivers',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'frame_id': 'laser_frame',
                'scan_topic': '/scan',
            }],
        ),
        
        # Nodo de control
        Node(
            package='axioma_control',
            executable='control_node',
            name='axioma_control',
            output='screen',
            parameters=[{
                'wheel_separation': 0.3,
                'wheel_radius': 0.05,
            }],
        ),
        
        # Nodo de navegación
        Node(
            package='axioma_navigation',
            executable='navigation_node',
            name='axioma_navigation',
            output='screen',
        ),
    ])
```

  * #### [Librerias](http://wiki.ros.org/Client%20Libraries)
  Son las herramientas que permiten la interación entre ROS y el codigo fuente del proyecto construido en determinado lenguaje. Las librerias principales son RCLCPP para C++ y RCLPY para Python, pero hay librerias clientes para todos los gustos y necesidades. 
  
   * ###### [rclcpp](https://docs.ros2.org/foxy/api/rclcpp/index.html) 
   Es la biblioteca cliente de ROS proporciona la API canónica de ***C++*** para interactuar con ROS.
    
   * ###### [rclpy](https://docs.ros2.org/latest/api/rclpy/index.html) 
   Es la biblioteca cliente de ROS proporciona la API canónica de ***Python*** para interactuar con ROS.
    
   Acontinuación veremos otros ejemplos de las librerias clientes para ROS.
   
|    Libreria   | Lenguaje    |   Libreria       | Lenguaje     |
| ------------- | ----------- | ---------------- | ------------ |
| ***rclcpp***  | *C++*       | ***rclpy***      | *Python*    |
| ***roscs***   | *Mono/.NET* | ***rosnode***    | *Node.js*    |
| ***rosseus*** | *Lisp*      | ***RobotOS.jl*** | *Julia*      |
| ***rosgo***   | *Go*        | ***PhaRos***     | *Pharo*      |
| ***roshask*** | *Haskell*   | ***rosR***       | *R*          | 
| ***rosjava*** | *Java*      | ***rosRuby***    | *Ruby*       |

---

* #### Nodos importantes del proyecto Axioma

|  Nodo                |Descripción |
|----------------------|------------|
| ***axioma_urdf***    |Descripción del robot virtual en URDF|
| ***axioma_gazebo***  |Robot simulado en Gazebo|
| ***axioma_node***    |Estado del robot y telemetría |
| ***axioma_nav2***    |Planificación de trayectorias con Nav2|
| ***axioma_explorer*** |Técnica SLAM para mapeo|
| ***axioma_control*** |Control de bajo nivel de motores|
| ***axioma_lidar***   |Driver del sensor LIDAR|
| ***axioma_camera***  |Procesamiento de imágenes|

---

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/frames4.png" alt="TF Frames" width="500">
</div>

---

### Virtualización del Robot (URDF)

En ROS (Robot Operating System), el **URDF** (Unified Robot Description Format) es un formato de archivo XML utilizado para describir la geometría del robot, es decir, su estructura física, en términos de enlaces (**links**) y juntas (**joints**). Los enlaces representan las partes sólidas del robot (como **eslabones o piezas**), mientras que las juntas describen cómo estos enlaces están conectados y pueden moverse entre sí (como **articulaciones rotativas o prismáticas**).

El **Xacro** es una extensión de XML utilizada para escribir URDF de manera más eficiente y modular. Permite la reutilización de código y la parametrización de modelos, lo que simplifica la descripción y mantenimiento de robots complejos.

| Elemento              | Descripción                                                      | Ejemplo de Uso                                               |
|-----------------------|------------------------------------------------------------------|-------------------------------------------------------------|
| **`<robot>`**         | Elemento raíz que define el robot.                              | `<robot name="axioma_robot">`                                   |
| **`<link>`**          | Define un link del robot, que representa una parte rígida.      | `<link name="base_link">`                                   |
| **`<joint>`**         | Define la conexión entre dos links y su tipo de movimiento.     | `<joint name="wheel_joint" type="continuous">`                   |
| **`<origin>`**        | Especifica la posición y orientación del link o joint.          | `<origin xyz="0 0 0.1" rpy="0 0 0"/>`                      |
| **`<parent>`**        | Define el link padre en un joint.                               | `<parent link="base_link"/>`                                |
| **`<child>`**         | Define el link hijo en un joint.                                | `<child link="wheel_left"/>`                                     |
| **`<visual>`**        | Define la representación visual del link.                       | `<visual><geometry><cylinder radius="0.05" length="0.03"/></geometry></visual>` |
| **`<collision>`**     | Define la geometría para las colisiones.                        | `<collision><geometry><cylinder radius="0.05" length="0.03"/></geometry></collision>` |
| **`<sensor>`**        | Define un sensor asociado al link.                              | `<sensor name="lidar" type="ray">...</sensor>`      |
| **`<material>`**      | Define las propiedades de material, como color.                 | `<material name="blue"><color rgba="0 0 1 1"/></material>` |

#### Ejemplo de URDF simplificado para Axioma:

```xml
<?xml version="1.0"?>
<robot name="axioma_robot">
  <!-- Base del robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Rueda izquierda -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint rueda izquierda -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.15 -0.05" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <!-- LIDAR -->
  <link name="laser_frame">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint LIDAR -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
```

## 4 Modelo matemático 

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/diff.jpeg" alt="Differential Drive Model" width="400">
</div>

### Modelo Cinemático del Robot Diferencial

El robot Axioma.io utiliza un sistema de locomoción diferencial que se basa en dos ruedas motrices independientes. Este modelo matemático describe el comportamiento cinemático del robot.

#### Parámetros del Sistema

- **L**: Distancia entre las ruedas (wheelbase) = 0.3 m
- **R**: Radio de las ruedas = 0.05 m
- **vL**: Velocidad lineal de la rueda izquierda
- **vR**: Velocidad lineal de la rueda derecha
- **v**: Velocidad lineal del robot
- **ω**: Velocidad angular del robot
- **(x, y)**: Posición del robot en el plano
- **θ**: Orientación del robot

#### Odometría

* ##### Modelo matemático robot Axioma

**Posición del Robot en coordenadas cartesianas:**

    Vx = V·cos(θ)
    Vy = V·sin(θ)

**Ecuaciones de velocidades lineales y angulares:**

    vL = v - (ω·L)/2    # Velocidad rueda izquierda
    vR = v + (ω·L)/2    # Velocidad rueda derecha
    
    ω = (vR - vL)/L     # Velocidad Angular
    v = (vR + vL)/2     # Velocidad lineal promedio

**Relaciones angulares:**

    ω = 90°/Δt   →   ω = (2π/ticks)/Δt  (rad/s)

**Período y Frecuencia:**

    T = Δt              # Periodo
    f = 1/T             # Frecuencia
    ω = 2π/T            # Velocidad angular
    V = ω·R             # Velocidad lineal

* ##### Odometría con encoders

**Distancias calculadas por encoders:**
- Dc: distancia central (Posición promedio)  
- Dr: distancia rueda derecha  
- Dl: distancia rueda izquierda

        Dc = (Dr + Dl)/2  

**Actualización de la posición:**

    x' = x + Dc·cos(θ)
    y' = y + Dc·sin(θ)
    θ' = θ + (Dr - Dl)/L

**Cálculo de distancia por encoder:**

    ΔTicks = TickActual - TickAnterior
    D = 2πR·(ΔTicks/N)   
    
donde **N** es el número de ticks por revolución de la rueda (1000 PPR para Axioma).

* ##### Sistema de Control con Retroalimentación

**Flujo de datos del sistema:**

    Ticks → Distancia → Posición (x,y)
    Ticks → Velocidad → Control de motores

**Control de tiempo de muestreo:**

| Muestra anterior | **Δmuestreo = 10ms** | Muestra actual |
|------------------|----------------------|----------------|
|                  | ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑  |                |

    Δmuestreo = muestreoActual - muestreoAnterior
    
    if(Δmuestreo > 10ms) ===> Ejecutar acción de Control

#### Matriz de Transformación Homogénea

Para el robot diferencial, la matriz de transformación que relaciona el sistema de coordenadas local del robot con el sistema global es:

```
T = [cos(θ)  -sin(θ)   x]
    [sin(θ)   cos(θ)   y]
    [   0        0     1]
```

#### Jacobiano del Robot

El jacobiano que relaciona las velocidades de las ruedas con las velocidades del robot es:

```
[v]   [R/2   R/2 ] [ωR]
[ω] = [R/L  -R/L ] [ωL]
```

donde ωR y ωL son las velocidades angulares de las ruedas derecha e izquierda respectivamente.

#### Modelo de Control PID

Para el control de velocidad de los motores, se implementa un controlador PID:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Donde:
- **Kp**: Ganancia proporcional = 2.0
- **Ki**: Ganancia integral = 0.5
- **Kd**: Ganancia derivativa = 0.1
- **e(t)**: Error de velocidad = velocidad_deseada - velocidad_actual

## Instalación y Configuración

### Prerrequisitos

Asegúrate de tener instalado:
- Ubuntu 20.04 LTS
- ROS2 Foxy
- Python 3.8+
- Git

### Instalación de ROS2 Foxy

```bash
# Configurar fuentes
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2 Foxy
sudo apt update
sudo apt install ros-foxy-desktop python3-argcomplete
```

### Instalación de dependencias específicas

```bash
# Dependencias del proyecto Axioma
sudo apt install ros-foxy-rqt-robot-steering ros-foxy-robot-state-publisher ros-foxy-joy ros-foxy-joy-linux ros-foxy-joy-teleop

# Dependencias de Python
pip3 install pyserial

# Herramientas de desarrollo
sudo apt install python3-colcon-common-extensions

# Navegación y SLAM
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-slam-toolbox

# Gazebo para simulación
sudo apt install ros-foxy-gazebo-ros-pkgs
```

### Configuración del entorno

```bash
# Agregar a ~/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Comandos para Ejecutar el Proyecto

### 1. Clonar y configurar el workspace

```bash
# Crear workspace
mkdir -p ~/axioma_ws/src
cd ~/axioma_ws/src

# Clonar el repositorio
git clone https://github.com/MrDavidAlv/Axioma_robot.git

# Compilar el workspace
cd ~/axioma_ws
colcon build

# Configurar el entorno
source install/setup.bash
```

### 2. Lanzar la simulación en Gazebo

```bash
# Terminal 1: Lanzar mundo de Gazebo con Axioma
ros2 launch axioma_gazebo axioma_world.launch.py

# Terminal 2: Lanzar nodos de navegación
ros2 launch axioma_nav2 navigation.launch.py

# Terminal 3: Lanzar SLAM (opcional)
ros2 launch axioma_explorer slam.launch.py
```

### 3. Control manual del robot

```bash
# Control por teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Control por joystick
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node
```

### 4. Visualización

```bash
# Lanzar RViz2 para visualización
ros2 run rviz2 rviz2

# Ver gráfico de nodos
rqt_graph

# Monitorear tópicos
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

### 5. Mapeo SLAM

```bash
# Iniciar mapeo
ros2 launch slam_toolbox online_async_launch.py

# Guardar mapa
ros2 run nav2_map_server map_saver_cli -f ~/axioma_ws/maps/mi_mapa
```

### 6. Navegación autónoma

```bash
# Cargar mapa y navegación
ros2 launch nav2_bringup navigation_launch.py map:=~/axioma_ws/maps/mi_mapa.yaml

# Establecer punto objetivo desde línea de comandos
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'
```

### 7. Monitoreo del sistema

```bash
# Ver estado de nodos
ros2 node list
ros2 node info /axioma_node

# Monitorear odometría
ros2 topic echo /odom

# Ver datos del LIDAR
ros2 topic echo /scan

# Verificar transformadas
ros2 run tf2_tools view_frames
```

### 8. Comandos de depuración

```bash
# Ver todos los tópicos activos
ros2 topic list -t

# Ver todos los servicios
ros2 service list

# Ver parámetros del robot
ros2 param list
ros2 param get /axioma_node robot_description

# Verificar frecuencia de tópicos
ros2 topic hz /odom
ros2 topic hz /scan
```

## Desarrollo con ROS2

### Preparación del espacio de trabajo

#### Creación del proyecto ROS2

1. **Creación del workspace:**
```bash
mkdir -p axioma_ws/src
```

2. **Ingresar al workspace:**
```bash
cd axioma_ws
```

3. **Compilación del proyecto:**
```bash
colcon build
```

Al compilar se crean 3 directorios nuevos:
```
\build    # Archivos de compilación
\install  # Archivos instalados
\log      # Logs de compilación
```

4. **Actualizar las fuentes compiladas:**
```bash
source install/setup.bash
```

💡 **Nota**: Realizar los pasos 3 y 4 cada vez que se realice un cambio.

### Creando paquetes

#### Paquete Python
```bash
ros2 pkg create --build-type ament_python --node-name axioma_node axioma_python

# Compilar paquete específico
cd ~/axioma_ws/
colcon build --packages-select axioma_python
source install/setup.bash

# Ejecutar el nodo
ros2 run axioma_python axioma_node
```

#### Paquete C++
```bash
ros2 pkg create --build-type ament_cmake --node-name axioma_node_cpp axioma_cpp

# Compilar paquete específico
cd ~/axioma_ws/
colcon build --packages-select axioma_cpp
source install/setup.bash

# Ejecutar el nodo
ros2 run axioma_cpp axioma_node_cpp
```

### APIs principales

#### Python (rclpy)

| Método | Descripción | Ejemplo |
|--------|-------------|---------|
| `rclpy.init()` | Inicializa rclpy | `rclpy.init()` |
| `Node()` | Crea un nodo | `node = Node('axioma_node')` |
| `create_subscription()` | Crea suscriptor | `sub = node.create_subscription(LaserScan, '/scan', callback, 10)` |
| `create_publisher()` | Crea publicador | `pub = node.create_publisher(Twist, '/cmd_vel', 10)` |
| `spin()` | Mantiene nodo activo | `rclpy.spin(node)` |
| `get_logger()` | Logger del nodo | `self.get_logger().info('Axioma iniciado')` |

#### Ejemplo de nodo en Python para Axioma:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AxiomaController(Node):
    def __init__(self):
        super().__init__('axioma_controller')
        
        # Publisher para comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber para datos del LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer para control periódico
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Axioma Controller iniciado')
    
    def scan_callback(self, msg):
        # Procesar datos del LIDAR
        min_distance = min([r for r in msg.ranges if r > msg.range_min])
        
        if min_distance < 0.5:  # Obstáculo detectado
            self.get_logger().warn(f'Obstáculo a {min_distance:.2f}m')
    
    def control_loop(self):
        # Lógica de control del robot
        cmd = Twist()
        cmd.linear.x = 0.2  # Velocidad hacia adelante
        cmd.angular.z = 0.0  # Sin rotación
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = AxiomaController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### C++ (rclcpp)

| Método | Descripción | Ejemplo |
|--------|-------------|---------|
| `rclcpp::init()` | Inicializa rclcpp | `rclcpp::init(argc, argv)` |
| `std::make_shared<Node>()` | Crea nodo | `auto node = std::make_shared<rclcpp::Node>("axioma_node")` |
| `create_subscription()` | Crea suscriptor | `auto sub = create_subscription<LaserScan>("/scan", 10, callback)` |
| `create_publisher()` | Crea publicador | `auto pub = create_publisher<Twist>("/cmd_vel", 10)` |
| `spin()` | Mantiene nodo activo | `rclcpp::spin(node)` |

#### Ejemplo de nodo en C++ para Axioma:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class AxiomaController : public rclcpp::Node
{
public:
    AxiomaController() : Node("axioma_controller")
    {
        // Publisher para comandos de velocidad
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Subscriber para datos del LIDAR
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&AxiomaController::scan_callback, this, std::placeholders::_1));
        
        // Timer para control periódico
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AxiomaController::control_loop, this));
        
        RCLCPP_INFO(get_logger(), "Axioma Controller iniciado");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Procesar datos del LIDAR
        float min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        
        if (min_distance < 0.5) {
            RCLCPP_WARN(get_logger(), "Obstáculo detectado a %.2fm", min_distance);
        }
    }
    
    void control_loop()
    {
        // Lógica de control del robot
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.2;  // Velocidad hacia adelante
        cmd.angular.z = 0.0; // Sin rotación
        cmd_vel_pub_->publish(cmd);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AxiomaController>());
    rclcpp::shutdown();
    return 0;
}
```

### Launch Files Avanzados

#### Launch file completo para Axioma:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Argumentos del launch
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true')
    
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=PathJoinSubstitution([
            FindPackageShare('axioma_description'),
            'urdf',
            'axioma.urdf.xacro'
        ]),
        description='Path to robot URDF file')
    
    # Nodos del sistema
    nodes = [
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description')
            }],
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        
        # Driver del LIDAR
        Node(
            package='axioma_drivers',
            executable='lidar_driver',
            name='lidar_driver',
            output='screen',
            parameters=[{
                'frame_id': 'laser_frame',
                'scan_topic': '/scan',
                'port': '/dev/ttyUSB0',
                'baud_rate': 115200,
            }],
        ),
        
        # Driver de motores
        Node(
            package='axioma_drivers',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[{
                'wheel_separation': 0.3,
                'wheel_radius': 0.05,
                'max_speed': 1.0,
                'port': '/dev/ttyACM0',
                'baud_rate': 57600,
            }],
        ),
        
        # Controlador de base
        Node(
            package='axioma_control',
            executable='base_controller',
            name='base_controller',
            output='screen',
            parameters=[{
                'update_rate': 50.0,
                'publish_odom_tf': True,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
            }],
        ),
        
        # Nodo de navegación
        Node(
            package='axioma_navigation',
            executable='navigation_node',
            name='axioma_navigation',
            output='screen',
            parameters=[{
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.0,
                'obstacle_distance': 0.3,
            }],
        ),
    ]
    
    # Incluir launch de navegación si está disponible
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                FindPackageShare('axioma_navigation'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time,
        robot_description,
        *nodes,
        nav2_launch,
    ])
```

### Programación Orientada a Objetos

#### POO con Python para ROS2

| Concepto | Descripción | Ejemplo en ROS2 |
|----------|-------------|-----------------|
| **Herencia** | Heredar de la clase Node | `class AxiomaNode(Node):` |
| **Constructor** | Inicializar el nodo | `def __init__(self): super().__init__('axioma')` |
| **Métodos** | Callbacks y funciones | `def scan_callback(self, msg):` |
| **Atributos** | Publishers, subscribers | `self.cmd_vel_pub = ...` |

#### POO con C++ para ROS2

| Concepto | Descripción | Ejemplo en ROS2 |
|----------|-------------|-----------------|
| **Herencia** | Heredar de rclcpp::Node | `class AxiomaNode : public rclcpp::Node` |
| **Constructor** | Inicializar el nodo | `AxiomaNode() : Node("axioma")` |
| **Métodos** | Callbacks y funciones | `void scan_callback(const LaserScan::SharedPtr msg)` |
| **Atributos** | Publishers, subscribers | `rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_` |

## 🚀 Características del Sistema

- **Navegación autónoma** con planificación de rutas usando Nav2
- **SLAM** (Simultaneous Localization and Mapping) con slam_toolbox
- **Evitación de obstáculos** en tiempo real con LIDAR
- **Control diferencial** con odometría de encoders de alta precisión
- **Interfaz de visualización** en RViz2 con datos en tiempo real
- **Simulación completa** en Gazebo con física realista
- **Comunicación inalámbrica** para monitoreo y control remoto
- **Arquitectura modular** con nodos especializados
- **Control PID** para velocidades de motores
- **Telemetría completa** del estado del robot

## 🤝 Contribuir

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📝 Licencia

Este proyecto está bajo la Licencia BSD - ver el archivo [LICENSE](LICENSE) para más detalles.

## 👥 Autores

- **Mario David Alvarez** - *Desarrollo principal* - [MrDavidAlv](https://github.com/MrDavidAlv)
- **Semillero de Robótica SIRO** - *Colaboradores*

## 🙏 Agradecimientos

- Semillero de Robótica SIRO
- Comunidad ROS2
- Open Source Robotics Foundation
- Universidad de Bogotá Jorge Tadeo Lozano

---

*Para más información, visita el repositorio: https://github.com/MrDavidAlv/Axioma_robot*

## 🎯 Enlaces Adicionales

- **Universidad de Bogotá Jorge Tadeo Lozano** - [UTADEO](https://www.utadeo.edu.co/)
- **Semillero de Robótica SIRO** - Desarrollo de proyectos robóticos
- **Mercury Robotics Challenge** - Competencia latinoamericana de robótica
- **ROS2 Documentation** - [docs.ros.org](https://docs.ros.org/)
- **Nav2 Documentation** - [navigation.ros.org](https://navigation.ros.org/)

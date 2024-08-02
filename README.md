# LoRaMesh-CHAT
Sistema de mensajería para redes comunitarias en situaciones de emergencia 

# Configuración de LoRaMesh-CHAT con PlatformIO y Visual Studio Code
1. Descargue Visual Studio Code.
2. Descargue PlatformIO dentro de Visual Studio Code.
3. Clona el repositorio o descarguelo.
4. Vaya a la página de inicio de PlatformIO, haga clic en el botón Proyectos, luego en "Agregar existente" y agregue LoRaMesh-CHAT.
5. Compile el proyecto con PlatformIO.
6. Cargue el proyecto en el microcontrolador LoRa especificado. En nuestro caso, utilizamos el módulo TTGOLoRa32 v2.

# data
En esta carpeta se encuentra el código implementado para la aplicación de la interfaz de chat web. 
Dentro de esta carpeta se colocan los archivos HTML, CSS, JS, etc. que se desea cargar en la tarjeta de desarrollo.

Al momento de cargar el código a un microcontrolador es importante ejecuta el siguiente comando para subir los archivos al sistema de archivos SPIFFS del ESP32:

pio run --target uploadfs

Este comando empaquetará todos los archivos en la carpeta data y los cargará en la memoria flash del ESP32 usando SPIFFS.

Otra opción es utilizando la interfaz de Platformio. Dentro de las tareas del proyecto en platformio (PROYECT TASKS) en el apartado de Platform hacer clic en (Build Filesystem Image) para crear la imagen del sistema de archivos y luego hacer clic en (Upload Filesystem Image) para cargar la imagen del sistema de archivos. Esto cargará los códigos que se encuentran dentro de la carpeta data al SPIFFS del microcontrolador.
[![Imageneee1.png](https://i.postimg.cc/28NKkTWP/Imageneee1.png)](https://postimg.cc/BtNgNTMg)

# main

En esta carpeta se encuentra el firmware utilizado para establecer el cifrado AES y las diferentes comunicaciones en el sistema.

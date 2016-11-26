# Bioloid-Locomotion-API
API for locomotion of Bioloid robot using a BeagleBone Black system

 *  Librería para locomoción de Robot humanoide Bioloid usando BeagleBone Black
 *  Funciones de cinemática inversa, generaciób de trayectoria y movimientos básicos
 *
 *  Creada por Juan Carlos Brenes Torres
 *  Máster en Automática e Informática Industrial
 *  Universidad Politécnica de Valencia

Archivo principal, contiene la máquina de estados del robot:
---BioloidIK.cpp

Librería de funciones de movimientos, cinemática inversa y trayectorias
---BioloidLocomotion.h
---BioloidLocomotion.cpp

Archivo con matrices de ángulos importadas desde Roboplus. Archivo personalizado para la 
prueba de subir y bajar escalones en el CEABOT.
---BioloidPosDef.h


*Librerías de comunicación Serial
---BlackDynamixel.cpp
---BlackDynamixel.h
---seriallib.cpp
---seriallib.h

*Se utiliza la API de comunicación con los servos AX-12, creada por David Cisternes
---DynamixelAXDef.h
---InstrucDynamixel.cpp
---InstrucDynamixel.h
---UtilDynamixel.cpp
---UtilDynamixel.h

*También se utilizan las librerías generales:
---GPIO.cpp
---GPIO.h
---util.cpp
---util.h

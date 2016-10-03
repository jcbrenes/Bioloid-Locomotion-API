 *  Librer�a para locomoci�n de Robot humanoide Bioloid usando BeagleBone Black
 *  Funciones de cinem�tica inversa, generaci�b de trayectoria y movimientos b�sicos
 *
 *  Creada por Juan Carlos Brenes Torres
 *  M�ster en Autom�tica e Inform�tica Industrial
 *  Universidad Polit�cnica de Valencia

Archivo principal, contiene la m�quina de estados del robot:
---BioloidIK.cpp

Librer�a de funciones de movimientos, cinem�tica inversa y trayectorias
---BioloidLocomotion.h
---BioloidLocomotion.cpp

Archivo con matrices de �ngulos importadas desde Roboplus. Archivo personalizado para la 
prueba de subir y bajar escalones en el CEABOT.
---BioloidPosDef.h


*Librer�as de comunicaci�n Serial
---BlackDynamixel.cpp
---BlackDynamixel.h
---seriallib.cpp
---seriallib.h

*Se utiliza la API de comunicaci�n con los servos AX-12, creada por David Cisternes
---DynamixelAXDef.h
---InstrucDynamixel.cpp
---InstrucDynamixel.h
---UtilDynamixel.cpp
---UtilDynamixel.h

*Tambi�n se utilizan las librer�as generales:
---GPIO.cpp
---GPIO.h
---util.cpp
---util.h

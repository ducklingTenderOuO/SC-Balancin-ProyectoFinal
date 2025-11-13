1. Usamos pyserial para comunicarnos con el Arduino.

buscar_puertos()
* Detecta los puertos disponibles con serial.tools.list_ports.comports().
* Los muestra en un Combobox para que el usuario elija uno.

conectar_arduino()
* Lanza un hilo separado (threading.Thread) para conectar sin congelar la interfaz.
* Abre el puerto serie a 9600 baudios.
* Envía STATUS\n para comprobar si el Arduino responde correctamente.
* Si la respuesta contiene palabras clave (como “KP”, “Balancin”, etc.), asume conexión exitosa.
* Inicia otro hilo leer_serial() para recibir datos continuamente.

leer_serial()
* Escucha el puerto en un bucle, leyendo líneas que envía el Arduino.
* Las pasa a procesar_mensaje_arduino().



2. Procesamiento de los datos recibidos

El Arduino envía líneas de texto, por ejemplo:
DATA:0.35,125.0,1,8.0,0.2,3100.0,0.0

La función procesar_mensaje_arduino() interpreta esa línea:

Campo	Significado
0	posición actual de la pelota (cm, entre -20 y +20)
1	salida PID
2	estado del PID (1 = activo, 0 = inactivo)
3–5	KP, KI, KD
6	Setpoint (posición objetivo)



3. Gráfica en tiempo real

* Usa una cola circular (deque(maxlen=100)) para guardar los últimos 100 puntos de tiempo y posición.

Cada vez que llega un dato nuevo:
* Se agrega el valor.
* Se actualizan las líneas self.line_pos (posición) y self.line_setpoint (objetivo).
* Se redibuja el gráfico.
* Esto permite ver en tiempo real cómo la pelota sigue el setpoint.



4. Envío de comandos al Arduino

Las funciones actualizar_kp, actualizar_ki, etc., envían comandos tipo:

KP:8.0
KI:0.2
KD:3100.0
SET:0.0

o comandos simples como:

STATUS
RESET
TOGGLE

Cada uno es enviado por serie usando:
self.arduino.write(f"{comando}\n".encode())
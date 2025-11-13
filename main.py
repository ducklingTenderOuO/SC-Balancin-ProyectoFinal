import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque


class InterfazBalancinPID:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Balancín - Barra 40cm (-20cm to +20cm)")
        self.root.geometry("800x600")

        # Variables
        self.arduino = None
        self.conectado = False
        self.puerto_seleccionado = tk.StringVar()

        # Datos para gráfica
        self.posiciones = deque(maxlen=100)  # Posición de la pelota (-20 a +20)
        self.tiempos = deque(maxlen=100)
        self.start_time = time.time()

        # Crear interfaz
        self.crear_interfaz()
        self.buscar_puertos()

    def crear_interfaz(self):
        # Frame de conexión
        frame_conexion = ttk.LabelFrame(self.root, text="Conexión Arduino", padding=10)
        frame_conexion.pack(fill="x", padx=10, pady=5)

        ttk.Label(frame_conexion, text="Puerto:").grid(row=0, column=0, padx=5)
        self.combo_puertos = ttk.Combobox(frame_conexion, textvariable=self.puerto_seleccionado, width=20)
        self.combo_puertos.grid(row=0, column=1, padx=5)

        ttk.Button(frame_conexion, text="Buscar puertos", command=self.buscar_puertos).grid(row=0, column=2, padx=5)
        self.btn_conectar = ttk.Button(frame_conexion, text="Conectar", command=self.toggle_conexion)
        self.btn_conectar.grid(row=0, column=3, padx=5)

        # Indicador de estado
        self.indicador = ttk.Label(frame_conexion, text="🔴 Desconectado", foreground="red")
        self.indicador.grid(row=0, column=4, padx=10)

        # Frame principal
        frame_principal = ttk.Frame(self.root)
        frame_principal.pack(fill="both", expand=True, padx=10, pady=5)

        # Frame de control PID
        frame_control = ttk.LabelFrame(frame_principal, text="Control PID - Posición pelota", padding=10)
        frame_control.pack(side="left", fill="y", padx=5)

        # Controles PID
        self.crear_controles_pid(frame_control)

        # Frame de gráfica
        frame_grafica = ttk.LabelFrame(frame_principal, text="Posición de la pelota en tiempo real", padding=10)
        frame_grafica.pack(side="right", fill="both", expand=True, padx=5)

        self.crear_grafica(frame_grafica)

        # Consola de mensajes
        frame_consola = ttk.LabelFrame(self.root, text="Mensajes", padding=10)
        frame_consola.pack(fill="x", padx=10, pady=5)

        self.consola = tk.Text(frame_consola, height=6, width=80)
        scrollbar = ttk.Scrollbar(frame_consola, command=self.consola.yview)
        self.consola.configure(yscrollcommand=scrollbar.set)

        self.consola.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def crear_controles_pid(self, parent):
        # Control KP
        ttk.Label(parent, text="KP (Proporcional):").grid(row=0, column=0, sticky="w", pady=5)
        self.kp_var = tk.DoubleVar(value=8.0)
        self.scale_kp = ttk.Scale(parent, from_=0, to=20, variable=self.kp_var,
                                  command=lambda v: self.actualizar_kp(float(v)))
        self.scale_kp.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        self.entry_kp = ttk.Entry(parent, textvariable=self.kp_var, width=8)
        self.entry_kp.grid(row=0, column=2, padx=5)
        ttk.Button(parent, text="Set", command=lambda: self.actualizar_kp(self.kp_var.get())).grid(row=0, column=3,
                                                                                                   padx=5)

        # Control KI
        ttk.Label(parent, text="KI (Integral):").grid(row=1, column=0, sticky="w", pady=5)
        self.ki_var = tk.DoubleVar(value=0.2)
        self.scale_ki = ttk.Scale(parent, from_=0, to=2, variable=self.ki_var,
                                  command=lambda v: self.actualizar_ki(float(v)))
        self.scale_ki.grid(row=1, column=1, sticky="ew", padx=5, pady=5)
        self.entry_ki = ttk.Entry(parent, textvariable=self.ki_var, width=8)
        self.entry_ki.grid(row=1, column=2, padx=5)
        ttk.Button(parent, text="Set", command=lambda: self.actualizar_ki(self.ki_var.get())).grid(row=1, column=3,
                                                                                                   padx=5)

        # Control KD
        ttk.Label(parent, text="KD (Derivativo):").grid(row=2, column=0, sticky="w", pady=5)
        self.kd_var = tk.DoubleVar(value=3100.0)
        self.scale_kd = ttk.Scale(parent, from_=0, to=6000, variable=self.kd_var,
                                  command=lambda v: self.actualizar_kd(float(v)))
        self.scale_kd.grid(row=2, column=1, sticky="ew", padx=5, pady=5)
        self.entry_kd = ttk.Entry(parent, textvariable=self.kd_var, width=8)
        self.entry_kd.grid(row=2, column=2, padx=5)
        ttk.Button(parent, text="Set", command=lambda: self.actualizar_kd(self.kd_var.get())).grid(row=2, column=3,
                                                                                                   padx=5)

        # Control Setpoint (POSICIÓN en la barra)
        ttk.Label(parent, text="Posición objetivo (cm):").grid(row=3, column=0, sticky="w", pady=5)
        self.setpoint_var = tk.DoubleVar(value=0.0)  # Centro por defecto
        self.scale_setpoint = ttk.Scale(parent, from_=-20, to=20, variable=self.setpoint_var,
                                        command=lambda v: self.actualizar_setpoint(float(v)))
        self.scale_setpoint.grid(row=3, column=1, sticky="ew", padx=5, pady=5)
        self.entry_setpoint = ttk.Entry(parent, textvariable=self.setpoint_var, width=8)
        self.entry_setpoint.grid(row=3, column=2, padx=5)
        ttk.Button(parent, text="Set", command=lambda: self.actualizar_setpoint(self.setpoint_var.get())).grid(row=3,
                                                                                                               column=3,
                                                                                                               padx=5)

        # Estado del PID
        ttk.Label(parent, text="Estado PID:").grid(row=4, column=0, sticky="w", pady=10)
        self.pid_state_var = tk.BooleanVar(value=True)
        self.check_pid = ttk.Checkbutton(parent, text="Activo", variable=self.pid_state_var,
                                         command=self.toggle_pid)
        self.check_pid.grid(row=4, column=1, sticky="w", padx=5)

        # Botones de control
        frame_botones = ttk.Frame(parent)
        frame_botones.grid(row=5, column=0, columnspan=4, pady=15)

        ttk.Button(frame_botones, text="Enviar todos", command=self.enviar_todos).pack(side="left", padx=5)
        ttk.Button(frame_botones, text="Leer estado", command=self.leer_estado).pack(side="left", padx=5)
        ttk.Button(frame_botones, text="Reset PID", command=self.reset_pid).pack(side="left", padx=5)

        # Labels de valores actuales
        ttk.Label(parent, text="Valores actuales:", font=('Arial', 10, 'bold')).grid(row=6, column=0, columnspan=4,
                                                                                     pady=(20, 5))

        self.label_posicion = ttk.Label(parent, text="Posición pelota: -- cm")
        self.label_posicion.grid(row=7, column=0, columnspan=4, sticky="w", pady=2)

        self.label_pid_out = ttk.Label(parent, text="Salida PID: --")
        self.label_pid_out.grid(row=8, column=0, columnspan=4, sticky="w", pady=2)

        self.label_setpoint = ttk.Label(parent, text="Objetivo: -- cm")
        self.label_setpoint.grid(row=9, column=0, columnspan=4, sticky="w", pady=2)

    def crear_grafica(self, parent):
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.line_pos, = self.ax.plot([], [], 'b-', label='Posición pelota (cm)', linewidth=2)
        self.line_setpoint, = self.ax.plot([], [], 'r--', label='Posición objetivo', linewidth=2)

        # Líneas de referencia para la barra
        self.ax.axhline(y=-20, color='gray', linestyle=':', alpha=0.5, label='Extremo -20cm')
        self.ax.axhline(y=20, color='gray', linestyle=':', alpha=0.5, label='Extremo +20cm')
        self.ax.axhline(y=0, color='green', linestyle='-', alpha=0.3, label='Centro (0cm)')

        self.ax.set_xlabel('Tiempo (s)')
        self.ax.set_ylabel('Posición en barra (cm)')
        self.ax.legend()
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(-25, 25)  # Rango ampliado para ver toda la barra

        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def buscar_puertos(self):
        puertos = [port.device for port in serial.tools.list_ports.comports()]
        self.combo_puertos['values'] = puertos
        if puertos:
            self.puerto_seleccionado.set(puertos[0])
            self.mostrar_mensaje(f"Puertos encontrados: {', '.join(puertos)}")
        else:
            self.mostrar_mensaje("⚠️ No se encontraron puertos. Conecta el Arduino.")

    def toggle_conexion(self):
        if not self.conectado:
            self.conectar_arduino()
        else:
            self.desconectar_arduino()

    def conectar_arduino(self):
        # Deshabilitar botón inmediatamente
        self.btn_conectar.config(state="disabled")
        self.mostrar_mensaje("Conectando...")

        # Ejecutar en hilo separado
        threading.Thread(target=self._conectar_arduino_thread, daemon=True).start()

    def _conectar_arduino_thread(self):
        try:
            # Verificar que el puerto existe
            puertos_disponibles = [port.device for port in serial.tools.list_ports.comports()]
            puerto_elegido = self.puerto_seleccionado.get()

            if not puerto_elegido or puerto_elegido not in puertos_disponibles:
                self.root.after(0, lambda: messagebox.showerror("Error",
                                                                f"Puerto '{puerto_elegido}' no encontrado.\n\n"
                                                                f"Puertos disponibles: {', '.join(puertos_disponibles) if puertos_disponibles else 'Ninguno'}"))
                self.root.after(0, self._reestablecer_conexion)
                return

            self.root.after(0, lambda: self.mostrar_mensaje(f"Intentando conectar a {puerto_elegido}..."))

            arduino = serial.Serial(puerto_elegido, 9600, timeout=1)
            time.sleep(2)  # Esperar inicialización

            # Verificación real
            arduino.reset_input_buffer()
            arduino.write(b"STATUS\n")
            time.sleep(1)

            if arduino.in_waiting > 0:
                respuesta = arduino.readline().decode().strip()
                self.root.after(0, lambda: self.mostrar_mensaje(f"Respuesta Arduino: {respuesta}"))

                if any(keyword in respuesta for keyword in ["ESTADO", "Balancin", "KP", "SET"]):
                    self.arduino = arduino
                    self.conectado = True

                    self.root.after(0, self._conexion_exitosa)

                    # Iniciar hilo de lectura
                    self.hilo_lectura = threading.Thread(target=self.leer_serial, daemon=True)
                    self.hilo_lectura.start()

                    # Solicitar estado actual
                    self.leer_estado()
                else:
                    arduino.close()
                    self.root.after(0, lambda: messagebox.showwarning("Advertencia",
                                                                      "Dispositivo conectado pero no responde como Arduino Balancín.\n"
                                                                      "¿Está cargado el código correcto?"))
                    self.root.after(0, self._reestablecer_conexion)
            else:
                arduino.close()
                self.root.after(0, lambda: messagebox.showwarning("Advertencia",
                                                                  "No hay respuesta del dispositivo.\n"
                                                                  "Verifica:\n"
                                                                  "1. Que el Arduino esté conectado\n"
                                                                  "2. Que tenga el código Balancín cargado\n"
                                                                  "3. Que el baud rate sea 9600"))
                self.root.after(0, self._reestablecer_conexion)

        except serial.SerialException as e:
            error_msg = str(e)
            if "FileNotFoundError" in error_msg or "could not open port" in error_msg:
                self.root.after(0, lambda: messagebox.showerror("Error",
                                                                f"No se puede abrir el puerto {self.puerto_seleccionado.get()}\n\n"
                                                                "Posibles causas:\n"
                                                                "• Arduino desconectado\n"
                                                                "• Puerto incorrecto\n"
                                                                "• Arduino en uso por otro programa"))
            else:
                self.root.after(0, lambda: messagebox.showerror("Error", f"Error de conexión: {error_msg}"))
            self.root.after(0, self._reestablecer_conexion)
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Error", f"Error inesperado: {str(e)}"))
            self.root.after(0, self._reestablecer_conexion)

    def _conexion_exitosa(self):
        self.btn_conectar.config(text="Desconectar", state="normal")
        self.indicador.config(text="🟢 Conectado", foreground="green")
        self.mostrar_mensaje("✅ Conectado exitosamente al Balancín PID")

    def _reestablecer_conexion(self):
        self.btn_conectar.config(text="Conectar", state="normal")
        self.indicador.config(text="🔴 Desconectado", foreground="red")

    def desconectar_arduino(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        self.conectado = False
        self.btn_conectar.config(text="Conectar")
        self.indicador.config(text="🔴 Desconectado", foreground="red")
        self.mostrar_mensaje("🔌 Desconectado del Arduino")

    def leer_serial(self):
        while self.conectado:
            try:
                if self.arduino and self.arduino.in_waiting > 0:
                    linea = self.arduino.readline().decode().strip()
                    if linea:
                        self.procesar_mensaje_arduino(linea)
            except Exception as e:
                self.mostrar_mensaje(f"Error en lectura: {e}")
                break
            time.sleep(0.01)

    def procesar_mensaje_arduino(self, mensaje):
        if mensaje.startswith("DATA:"):
            try:
                datos = mensaje[5:].split(",")
                if len(datos) >= 7:
                    posicion_pelota = float(datos[0])  # Esto ahora es la posición, no distancia
                    pid_out = float(datos[1])
                    pid_active = bool(int(datos[2]))
                    kp = float(datos[3])
                    ki = float(datos[4])
                    kd = float(datos[5])
                    setpoint = float(datos[6])

                    # Actualizar interfaz
                    self.actualizar_valores_ui(posicion_pelota, pid_out, pid_active, kp, ki, kd, setpoint)

                    # Actualizar gráfica
                    self.actualizar_grafica(posicion_pelota, setpoint)

            except ValueError as e:
                self.mostrar_mensaje(f"Error procesando DATA: {e}")

        elif mensaje.startswith("ESTADO:"):
            self.mostrar_mensaje(f"Estado: {mensaje}")
        else:
            self.mostrar_mensaje(f"Arduino: {mensaje}")

    def actualizar_valores_ui(self, posicion_pelota, pid_out, pid_active, kp, ki, kd, setpoint):
        def actualizar():
            # Mostrar posición con signo para indicar dirección
            signo = "+" if posicion_pelota >= 0 else ""
            self.label_posicion.config(text=f"Posición pelota: {signo}{posicion_pelota:.2f} cm")
            self.label_pid_out.config(text=f"Salida PID: {pid_out:.1f}")
            self.label_setpoint.config(text=f"Objetivo: {setpoint:.1f} cm")
            self.pid_state_var.set(pid_active)

            # Actualizar variables si son diferentes
            if abs(self.kp_var.get() - kp) > 0.01:
                self.kp_var.set(kp)
            if abs(self.ki_var.get() - ki) > 0.001:
                self.ki_var.set(ki)
            if abs(self.kd_var.get() - kd) > 1:
                self.kd_var.set(kd)
            if abs(self.setpoint_var.get() - setpoint) > 0.1:
                self.setpoint_var.set(setpoint)

        self.root.after(0, actualizar)

    def actualizar_grafica(self, posicion_pelota, setpoint):
        current_time = time.time() - self.start_time

        self.tiempos.append(current_time)
        self.posiciones.append(posicion_pelota)

        def actualizar_plot():
            if len(self.tiempos) > 0:
                self.line_pos.set_data(self.tiempos, self.posiciones)
                self.line_setpoint.set_data(self.tiempos, [setpoint] * len(self.tiempos))

                # Ajustar límites del tiempo
                if len(self.tiempos) > 1:
                    self.ax.set_xlim(self.tiempos[0], self.tiempos[-1])

                self.canvas.draw()

        self.root.after(0, actualizar_plot)

    def mostrar_mensaje(self, mensaje):
        def actualizar():
            self.consola.insert(tk.END, f"{mensaje}\n")
            self.consola.see(tk.END)

        self.root.after(0, actualizar)

    # Funciones de control PID
    def actualizar_kp(self, valor):
        if self.conectado:
            self.enviar_comando(f"KP:{valor}")

    def actualizar_ki(self, valor):
        if self.conectado:
            self.enviar_comando(f"KI:{valor}")

    def actualizar_kd(self, valor):
        if self.conectado:
            self.enviar_comando(f"KD:{valor}")

    def actualizar_setpoint(self, valor):
        if self.conectado:
            self.enviar_comando(f"SET:{valor}")

    def toggle_pid(self):
        if self.conectado:
            self.enviar_comando("TOGGLE")

    def reset_pid(self):
        if self.conectado:
            self.enviar_comando("RESET")

    def enviar_todos(self):
        if self.conectado:
            self.actualizar_kp(self.kp_var.get())
            self.actualizar_ki(self.ki_var.get())
            self.actualizar_kd(self.kd_var.get())
            self.actualizar_setpoint(self.setpoint_var.get())
            self.mostrar_mensaje("Todos los parámetros PID enviados")

    def leer_estado(self):
        if self.conectado:
            self.enviar_comando("STATUS")

    def enviar_comando(self, comando):
        if self.conectado and self.arduino:
            try:
                self.arduino.write(f"{comando}\n".encode())
                self.mostrar_mensaje(f"Enviado: {comando}")
            except Exception as e:
                self.mostrar_mensaje(f"Error enviando: {str(e)}")


if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    style.theme_use("clam")  # Tema base para personalizar

    # Colores base
    color_fondo = "#f6e3f4"  
    color_acento = "#d684ce"  
    color_texto = "#000000"  

    # Fondo general de la ventana
    root.configure(bg=color_fondo)

    # ---- Personalizar estilos ttk ----
    style.configure("TFrame", background=color_fondo)
    style.configure("TLabel", background=color_fondo, foreground=color_texto, font=("Arial", 10))
    style.configure("TLabelFrame", background=color_fondo, foreground=color_texto, font=("Arial", 10, "bold"))
    style.configure("TButton", background=color_acento, foreground="white", font=("Arial", 10, "bold"))
    style.map("TButton",
              background=[("active", "#c06ab8")],
              foreground=[("disabled", "#999999")])
    style.configure("TCheckbutton", background=color_fondo, foreground=color_texto)
    style.configure("TEntry", fieldbackground="white", foreground=color_texto)
    style.configure("TCombobox", fieldbackground="white", background=color_fondo, foreground=color_texto)
    style.configure("Horizontal.TScale", background=color_fondo, troughcolor="#e4b5de")
    #_
    
    app = InterfazBalancinPID(root)
    root.mainloop()
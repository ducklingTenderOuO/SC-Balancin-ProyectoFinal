import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import threading
import time
from collections import deque


class PIDControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control PID - Sensor Ultrasónico")
        self.root.geometry("1000x700")

        self.serial_port = None
        self.is_connected = False
        self.is_reading = False

        # Datos para graficar
        self.max_points = 200
        self.time_data = deque(maxlen=self.max_points)
        self.distance_data = deque(maxlen=self.max_points)
        self.error_data = deque(maxlen=self.max_points)
        self.start_time = time.time()

        self.setup_ui()

    def setup_ui(self):
        # Frame superior para conexión
        conn_frame = ttk.LabelFrame(self.root, text="Conexión Serial", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(conn_frame, text="Puerto:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()

        ttk.Button(conn_frame, text="🔄", command=self.refresh_ports, width=3).grid(row=0, column=2, padx=2)

        self.connect_btn = ttk.Button(conn_frame, text="Conectar", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5)

        self.status_label = ttk.Label(conn_frame, text="● Desconectado", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)

        # Frame para controles PID
        control_frame = ttk.LabelFrame(self.root, text="Parámetros PID", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        # KP
        ttk.Label(control_frame, text="Kp:").grid(row=0, column=0, sticky='e', padx=5)
        self.kp_var = tk.DoubleVar(value=15.0)
        self.kp_entry = ttk.Entry(control_frame, textvariable=self.kp_var, width=10)
        self.kp_entry.grid(row=0, column=1, padx=5)
        self.kp_scale = ttk.Scale(control_frame, from_=0, to=50, orient='horizontal',
                                  length=200, command=lambda v: self.kp_var.set(float(v)))
        self.kp_scale.set(15.0)
        self.kp_scale.grid(row=0, column=2, padx=5)
        ttk.Button(control_frame, text="Enviar", command=lambda: self.send_param("kp", self.kp_var.get())).grid(row=0,
                                                                                                                column=3,
                                                                                                                padx=5)

        # KI
        ttk.Label(control_frame, text="Ki:").grid(row=1, column=0, sticky='e', padx=5)
        self.ki_var = tk.DoubleVar(value=0.6)
        self.ki_entry = ttk.Entry(control_frame, textvariable=self.ki_var, width=10)
        self.ki_entry.grid(row=1, column=1, padx=5)
        self.ki_scale = ttk.Scale(control_frame, from_=0, to=5, orient='horizontal',
                                  length=200, command=lambda v: self.ki_var.set(float(v)))
        self.ki_scale.set(0.6)
        self.ki_scale.grid(row=1, column=2, padx=5)
        ttk.Button(control_frame, text="Enviar", command=lambda: self.send_param("ki", self.ki_var.get())).grid(row=1,
                                                                                                                column=3,
                                                                                                                padx=5)

        # KD
        ttk.Label(control_frame, text="Kd:").grid(row=2, column=0, sticky='e', padx=5)
        self.kd_var = tk.DoubleVar(value=48.0)
        self.kd_entry = ttk.Entry(control_frame, textvariable=self.kd_var, width=10)
        self.kd_entry.grid(row=2, column=1, padx=5)
        self.kd_scale = ttk.Scale(control_frame, from_=0, to=200, orient='horizontal',
                                  length=200, command=lambda v: self.kd_var.set(float(v)))
        self.kd_scale.set(48.0)
        self.kd_scale.grid(row=2, column=2, padx=5)
        ttk.Button(control_frame, text="Enviar", command=lambda: self.send_param("kd", self.kd_var.get())).grid(row=2,
                                                                                                                column=3,
                                                                                                                padx=5)

        # Referencia
        ttk.Label(control_frame, text="Referencia:").grid(row=3, column=0, sticky='e', padx=5)
        self.ref_var = tk.DoubleVar(value=0.0)
        self.ref_entry = ttk.Entry(control_frame, textvariable=self.ref_var, width=10)
        self.ref_entry.grid(row=3, column=1, padx=5)
        self.ref_scale = ttk.Scale(control_frame, from_=-20, to=20, orient='horizontal',
                                   length=200, command=lambda v: self.ref_var.set(float(v)))
        self.ref_scale.set(0.0)
        self.ref_scale.grid(row=3, column=2, padx=5)
        ttk.Button(control_frame, text="Enviar", command=lambda: self.send_param("r", self.ref_var.get())).grid(row=3,
                                                                                                                column=3,
                                                                                                                padx=5)

        # Botón enviar todos
        ttk.Button(control_frame, text="Enviar Todos", command=self.send_all_params).grid(row=4, column=1, columnspan=2,
                                                                                          pady=10)

        # Frame para valores actuales
        values_frame = ttk.LabelFrame(self.root, text="Valores Actuales", padding=10)
        values_frame.pack(fill='x', padx=10, pady=5)

        self.dist_label = ttk.Label(values_frame, text="Distancia: -- cm", font=('Arial', 10, 'bold'))
        self.dist_label.grid(row=0, column=0, padx=15)

        self.error_label = ttk.Label(values_frame, text="Error: --", font=('Arial', 10, 'bold'))
        self.error_label.grid(row=0, column=1, padx=15)

        self.servo_label = ttk.Label(values_frame, text="Servo: --°", font=('Arial', 10, 'bold'))
        self.servo_label.grid(row=0, column=2, padx=15)

        # Frame para gráficas
        graph_frame = ttk.Frame(self.root)
        graph_frame.pack(fill='both', expand=True, padx=10, pady=5)

        # Crear figura con subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.tight_layout(pad=3.0)

        # Configurar primer gráfico (solo distancia)
        self.ax1.set_title('Distancia del Sensor')
        self.ax1.set_xlabel('Tiempo (s)')
        self.ax1.set_ylabel('Distancia (cm)')
        self.ax1.grid(True, alpha=0.3)
        self.line_dist, = self.ax1.plot([], [], 'b-', label='Distancia', linewidth=2)
        self.ax1.legend(loc='upper right')

        # Configurar segundo gráfico (Error)
        self.ax2.set_title('Error del Sistema')
        self.ax2.set_xlabel('Tiempo (s)')
        self.ax2.set_ylabel('Error (cm)')
        self.ax2.grid(True, alpha=0.3)
        self.line_error, = self.ax2.plot([], [], 'g-', label='Error', linewidth=2)
        self.ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.ax2.legend(loc='upper right')

        # Integrar matplotlib en tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # Iniciar animación
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Error", "Selecciona un puerto serial")
            return

        try:
            self.serial_port = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Esperar inicialización
            self.is_connected = True
            self.is_reading = True

            self.connect_btn.config(text="Desconectar")
            self.status_label.config(text="● Conectado", foreground="green")
            self.port_combo.config(state='disabled')

            # Iniciar hilo de lectura
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

            messagebox.showinfo("Conectado", f"Conectado a {port}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar: {str(e)}")

    def disconnect(self):
        self.is_reading = False
        time.sleep(0.5)

        if self.serial_port:
            self.serial_port.close()

        self.is_connected = False
        self.connect_btn.config(text="Conectar")
        self.status_label.config(text="● Desconectado", foreground="red")
        self.port_combo.config(state='readonly')

    def read_serial(self):
        while self.is_reading:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    self.parse_data(line)
            except Exception as e:
                print(f"Error leyendo serial: {e}")
                time.sleep(0.1)

    def parse_data(self, line):
        try:
            parts = line.split()
            data = {}

            for i in range(0, len(parts), 2):
                if i + 1 < len(parts):
                    key = parts[i].rstrip(':')
                    value = parts[i + 1]
                    data[key] = float(value)

            if 'Dist' in data:
                current_time = time.time() - self.start_time
                self.time_data.append(current_time)
                self.distance_data.append(data.get('Dist', 0))
                self.error_data.append(data.get('Error', 0))

                # Actualizar labels
                self.dist_label.config(text=f"Distancia: {data.get('Dist', 0):.2f} cm")
                self.error_label.config(text=f"Error: {data.get('Error', 0):.2f}")
                self.servo_label.config(text=f"Servo: {int(data.get('Servo', 90))}°")
        except:
            pass

    def update_plot(self, frame):
        if len(self.time_data) > 0:
            # Actualizar línea de distancia
            self.line_dist.set_data(list(self.time_data), list(self.distance_data))
            self.line_error.set_data(list(self.time_data), list(self.error_data))

            # Ajustar límites
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()

        return self.line_dist, self.line_error

    def send_param(self, param, value):
        if not self.is_connected:
            messagebox.showwarning("Advertencia", "No estás conectado al Arduino")
            return

        try:
            command = f"{param} {value}\n"
            self.serial_port.write(command.encode())
            print(f"Enviado: {command.strip()}")
        except Exception as e:
            messagebox.showerror("Error", f"Error enviando comando: {str(e)}")

    def send_all_params(self):
        if not self.is_connected:
            messagebox.showwarning("Advertencia", "No estás conectado al Arduino")
            return

        self.send_param("kp", self.kp_var.get())
        time.sleep(0.1)
        self.send_param("ki", self.ki_var.get())
        time.sleep(0.1)
        self.send_param("kd", self.kd_var.get())
        time.sleep(0.1)
        self.send_param("r", self.ref_var.get())


if __name__ == "__main__":
    root = tk.Tk()
    app = PIDControllerGUI(root)
    root.mainloop()

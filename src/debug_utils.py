import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits import mplot3d
import jsbsim_properties as prp
import math
import numpy as np
from airsim import Vector3r
import matplotlib.animation as animation
import threading
import time

class DebugGraphs:
    def __init__(self, sim):
        self.sim = sim
        self.time = []
        self.lat = []
        self.long = []
        self.lat_abs = []
        self.long_abs = []
        self.alt = []
        self.yaw = []
        self.pitch = []
        self.roll = []
        self.airspeed = []
        self.vs = []

        self.alpha = []

        self.clo = []
        self.clalpha = []
        self.clq = []
        self.clde = []

        self.cmo = []
        self.cmalpha = []
        self.cmq = []
        self.cmde = []

        self.aileron_cmd = []
        self.elevator_cmd = []
        self.throttle_cmd = []
        self.rudder_cmd = []

        self.aileron_left = []
        self.aileron_right = []
        self.aileron_combined = []
        self.elevator = []
        self.throttle = []
        self.rudder = []

        self.p = []
        self.q = []
        self.r = []

        #Agregado de propiedades de simulador que no fueron declarados 
        self.trayec_def = [] #Trayectoria con resolucion definida. Base para puntos de aplicacion de vectores de propiedades fisicas.

        self.pressure_sim = []
        self.alt_pressureAirsim = []

        self.lineal_velocity_airsim_x = []
        self.lineal_velocity_airsim_y = []
        self.lineal_velocity_airsim_z = []

        self.lineal_velocity_airsim_x_derivate = []
        self.lineal_velocity_airsim_y_derivate = []
        self.lineal_velocity_airsim_z_derivate = []

        self.angular_vel_jsbsim_x = []
        self.angular_vel_jsbsim_y = []
        self.angular_vel_jsbsim_z = []

        self.angular_vel_airsim_x = []
        self.angular_vel_airsim_y = []
        self.angular_vel_airsim_z = []

        self.lineal_acc_airsim_x = []
        self.lineal_acc_airsim_y = []
        self.lineal_acc_airsim_z = []

        self.lineal_acc_airsim_modulo = [] 

        self.lineal_acc_jsbsim_x = []
        self.lineal_acc_jsbsim_y = []
        self.lineal_acc_jsbsim_z = []
        self.lineal_acc_jsbsim_modulo = []


        #SENSORES (inicializo array para guardar datos de sensores):
        self.barometric_altitude = [] #Barometric Altitude array
        self.barometric_pressure = [] #Barometric Pressure array}

        self.gps_latitude = [] 
        self.gps_longitude = []
        self.gps_altitude = []
        self.gps_velocity_x = []
        self.gps_velocity_y = []
        self.gps_velocity_z = []
        self.gps_velocity_mod= []   

        self.imu_lineal_acc_x = [] #IMU X Linear Acceleration array
        self.imu_lineal_acc_y = [] #IMU Y Linear Acceleration array
        self.imu_lineal_acc_z = [] #IMU Z Linear Acceleration array


        self.imu_angular_vel_x = [] #IMU X Angular Velocity array
        self.imu_angular_vel_y = [] #IMU Y Angular Velocity array
        self.imu_angular_vel_z = [] #IMU Z Angular Velocity array

        self.imu_lineal_acc_modulo = [] #IMU Linear Acceleration Modulo array

        self.magnetometer_x = [] #Magnetometer X array
        self.magnetometer_y = [] #Magnetometer Y array
        self.magnetometer_z = [] #Magnetometer Z array


       
        # ANIMACION CON MATPLOTLIB
       #  Crear la figura y ejes UNA SOLA VEZ
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 10))
        self.fig.suptitle('Sensores en tiempo real')

        #  Inicializar líneas de los gráficos
        self.line_gps_vel_mod, = self.axs[0].plot([], [], 'm', label='Módulo de Velocidad con GPS')
        self.axs[0].set_ylabel('Velocidad (m/s)')
        self.axs[0].set_xlabel('Tiempo (s)')
        self.axs[0].legend()
        self.line_baro, = self.axs[1].plot([], [], 'c', label='Altitud Barométrica')
        self.axs[1].set_ylabel('Altitud (m)')
        self.axs[1].set_xlabel('Tiempo (s)')
        self.axs[1].legend()
        self.line_gps_lat_long, =self.axs[2].plot([], [], 'g', label='Posición con GPS')
        self.axs[2].legend()
        self.axs[2].set_ylabel('Longitud')
        self.axs[2].set_xlabel('Latitud')


    

    #función para llenar el array con las propiedades del simulador
    def get_pressure_sim(self):
        pressure_Pa = self.sim.get_static_pressure() #AirSim
        self.pressure_sim.append(pressure_Pa)
    
    def get_alt_pressure_Airsim(self):
        pressure_Pa = self.sim.get_static_pressure() #AirSim
        self.alt_pressureAirsim.append(44330.77 * (1 - (pressure_Pa[0] / 101325)**(1/5.255))) #Calculo de altitud de presion. En metros.  
    
    def get_linear_velocity_airsim(self):
        lineal_velocity = self.sim.get_linear_velocity_airsim()
        self.lineal_velocity_airsim_x.append(lineal_velocity[0])
        self.lineal_velocity_airsim_y.append(lineal_velocity[1])
        self.lineal_velocity_airsim_z.append(lineal_velocity[2])

    def get_angular_vel_jsbsim(self):
        angular_vel_x = self.sim[prp.p_radps]
        angular_vel_y = self.sim[prp.q_radps]
        angular_vel_z = self.sim[prp.r_radps] 
        self.angular_vel_jsbsim_x.append(angular_vel_x)
        self.angular_vel_jsbsim_y.append(angular_vel_y)
        self.angular_vel_jsbsim_z.append(angular_vel_z)
        

    def get_lineal_acc_jsbsim(self):
        lineal_acc_x = self.sim[prp.acceleration_body_axis_x_ft_sec2]
        lineal_acc_y = self.sim[prp.acceleration_body_axis_y_ft_sec2]
        lineal_acc_z = self.sim[prp.acceleration_body_axis_z_ft_sec2]
        self.lineal_acc_jsbsim_x.append(lineal_acc_x / 3.28084) #Conversion de ft/s2 a m/s2
        self.lineal_acc_jsbsim_y.append(lineal_acc_y / 3.28084)
        self.lineal_acc_jsbsim_z.append(lineal_acc_z / 3.28084)
        self.lineal_acc_jsbsim_modulo.append(math.sqrt((lineal_acc_x/3.28084)**2 + (lineal_acc_y/3.28084)**2 + (lineal_acc_z/3.28084)**2)) #Calculo de modulo de aceleración lineal. En m/s2

    def get_angular_vel_Airsim(self):
        angular_vel = self.sim.get_angular_velocity_airsim()
        self.angular_vel_airsim_x.append(angular_vel[0])
        self.angular_vel_airsim_y.append(angular_vel[1])
        self.angular_vel_airsim_z.append(angular_vel[2])
    
    def get_lineal_acc_Airsim(self):
        lineal_acc = self.sim.get_linear_acceleration_airsim()
        self.lineal_acc_airsim_x.append(lineal_acc[0])
        self.lineal_acc_airsim_y.append(lineal_acc[1])
        self.lineal_acc_airsim_z.append(lineal_acc[2])

        self.lineal_acc_airsim_modulo.append(math.sqrt(lineal_acc[0]**2 + lineal_acc[1]**2 + lineal_acc[2]**2)) #Calculo de modulo de aceleración lineal. En m/s2  

    
    #Funciones que rellenan los arrays con los datos de los sensores.
    def get_barometric_alt(self):
        barometer_data= self.sim.getBarometerData()
        self.barometric_altitude.append(barometer_data[0])#Barometric Altitude en metros 
    
    def get_gps_data(self):
        gps_data = self.sim.get_gps_data()
        self.gps_latitude.append(gps_data[0][0]) #GPS Latitude
        self.gps_longitude.append(gps_data[0][1]) #GPS Longitude 
        self.gps_altitude.append(gps_data[0][2])
        self.gps_velocity_x.append(gps_data[1][0])
        self.gps_velocity_y.append(gps_data[1][1])
        self.gps_velocity_z.append(gps_data[1][2])
        self.gps_velocity_mod.append(math.sqrt(gps_data[1][0]**2 + gps_data[1][1]**2 + gps_data[1][2]**2)) #Calculo de modulo de velocidad. En m/s

    def get_barometric_pressure(self):
        barometer_data= self.sim.getBarometerData()
        self.barometric_pressure.append(barometer_data[1])#Barometric Pressure    

    def get_imu_lineal_acc(self):
        imu_data = self.sim.getImuData()
        self.imu_lineal_acc_x.append(imu_data[1][0])#IMU X Linear Acceleration
        self.imu_lineal_acc_y.append(imu_data[1][1])#IMU Y Linear Acceleration
        self.imu_lineal_acc_z.append(imu_data[1][2])#IMU Z Linear Acceleration
        self.imu_lineal_acc_modulo.append(math.sqrt(imu_data[1][0]**2 + imu_data[1][1]**2 + imu_data[1][2]**2))#IMU Linear Acceleration Modulo

    def get_imu_angular_vel(self):
        imu_data = self.sim.getImuData()
        self.imu_angular_vel_x.append(imu_data[0][0])#IMU x Angular Velocity
        self.imu_angular_vel_y.append(imu_data[0][1])#IMU y Angular Velocity
        self.imu_angular_vel_z.append(imu_data[0][2])#IMU z Angular Velocity

    def get_magnetometer(self):
        magnetometer_data = self.sim.getMagnetoData()
        self.magnetometer_x.append(magnetometer_data[0])#Magnetometer X
        self.magnetometer_y.append(magnetometer_data[1])#Magnetometer Y
        self.magnetometer_z.append(magnetometer_data[2])#Magnetometer Z


    #Funciones para plotear los datos de los sensores y simulador.
    def barometric_alt_plot(self): #Ploteo de altitud de presión
        fig, ax = plt.subplots()
        ax.set_title('Altitud de Barómetro')
        ax.plot(self.time, self.barometric_altitude,color='c')
        plt.grid(True)
        plt.show()

    def alt_pressure_Airsim_plot(self): #Ploteo de altitud de presión de AirSim
        fig, ax = plt.subplots()
        ax.set_title('Altitude: - Ideal AirSim ')
        ax.plot(self.time, self.alt_pressureAirsim)
        plt.grid(True)
        plt.show()


    def barometric_pressure_plot(self): #Ploteo de presión barométrica
        fig, ax = plt.subplots()
        ax.set_title('Presión de Barómetro')
        ax.plot(self.time, self.barometric_pressure)
        plt.grid(True)
        plt.show()    

    def pressure_sim_plot(self): #Ploteo de presión del simulador
        fig, ax = plt.subplots()
        ax.set_title('Pressure: - AirSim ')
        ax.plot(self.time, self.pressure_sim) 
        plt.grid(True)
        plt.show()
    
    def angular_vel_airsim_plot(self): #Ploteo de velocidad angular de AirSim
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.angular_vel_airsim_x, color='blue')
        axs[0].set_ylabel("AirSim X Angular Velocity (rad/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.angular_vel_airsim_y, color='red')
        axs[1].set_ylabel("AirSim Y Angular Velocity (rad/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.angular_vel_airsim_z, color='black')
        axs[2].set_ylabel("AirSim Z Angular Velocity (rad/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()

    def imu_lineal_acc_plot(self): #Ploteo de aceleración lineal del IMU
        fig, axs = plt.subplots(4, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.imu_lineal_acc_x, color='blue')
        axs[0].set_ylabel("Aceleración lineal X-IMU  (m2/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.imu_lineal_acc_y, color='red')
        axs[1].set_ylabel("Aceleración lineal Y-IMU (m2/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.imu_lineal_acc_z, color='black')
        axs[2].set_ylabel("Aceleración lineal Z-IMU(m2/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        axs[3].plot(self.time, self.imu_lineal_acc_modulo, color='black')
        axs[3].set_ylabel("Mod.Acel.Lineal IMU (m2/s)", fontsize=12, color="black")
        axs[3].set_xlabel("Time in s", fontsize=12)
        axs[3].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()
    
    def lineal_acc_airsim_plot(self): #Ploteo de aceleración lineal de AirSim
        fig, axs = plt.subplots(4, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.lineal_acc_airsim_x, color='blue')
        axs[0].set_ylabel("AirSim X Lineal Acceleration (m2/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.lineal_acc_airsim_y, color='red')
        axs[1].set_ylabel("AirSim Y Lineal Acceleration (m2/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.lineal_acc_airsim_z, color='black')
        axs[2].set_ylabel("AirSim Z Linear Acceleration (m2/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        axs[3].plot(self.time, self.lineal_acc_airsim_modulo, color='black')
        axs[3].set_ylabel("AirSim Linear Acceleration MOD (m2/s)", fontsize=12, color="black")
        axs[3].set_xlabel("Time in s", fontsize=12)
        axs[3].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()
    
    def lineal_acc_jsbsim_plot(self): #Ploteo de aceleración lineal de JSBSim
        fig, axs = plt.subplots(4, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.lineal_acc_jsbsim_x, color='blue')
        axs[0].set_ylabel("JSBSim X Lineal Acceleration (m2/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.lineal_acc_jsbsim_y, color='red')
        axs[1].set_ylabel("JSBSim Y Lineal Acceleration (m2/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.lineal_acc_jsbsim_modulo, color='black')
        axs[2].set_ylabel("JSBSim  Linear Acceleration Módulo (m2/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        axs[3].plot(self.time, self.lineal_acc_jsbsim_z, color='black')
        axs[3].set_ylabel("JSBSim Z Linear Acceleration (m2/s)", fontsize=12, color="black")
        axs[3].set_xlabel("Time in s", fontsize=12)
        axs[3].grid(True, linestyle='--', alpha=0.6)
        

        plt.tight_layout()

        plt.show()
    
    def imu_angular_vel_plot(self): #Ploteo de velocidad angular del IMU
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.imu_angular_vel_x, color='blue')
        axs[0].set_ylabel("Vel.Angular X-IMU(rad/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.imu_angular_vel_y, color='red')
        axs[1].set_ylabel("Vel.Angular Y-IMU(rad/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.imu_angular_vel_z, color='black')
        axs[2].set_ylabel("Vel.Angular Z-IMU(rad/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()

    def angular_vel_jsbsim_plot(self): #Ploteo de velocidad angular del JSBSim
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.angular_vel_jsbsim_x, color='blue')
        axs[0].set_ylabel("JSBSim X Angular Velocity (rad/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.angular_vel_jsbsim_y, color='red')
        axs[1].set_ylabel("JSBSim Y Angular Velocity (rad/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.angular_vel_jsbsim_z, color='black')
        axs[2].set_ylabel("JSBSim Z Angular Velocity (rad/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()

    def lineal_velocity_airsim_plot(self):
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.lineal_velocity_airsim_x, color='blue')
        axs[0].set_ylabel("AirSim X Linear Velocity (m/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.lineal_velocity_airsim_y, color='red')
        axs[1].set_ylabel("AirSim Y Linear Velocity (m/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.lineal_velocity_airsim_z, color='black')
        axs[2].set_ylabel("AirSim Z Linear Velocity (m/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()

    def lineal_velocity_gps_plot(self):
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.gps_velocity_x, color='blue')
        axs[0].set_ylabel("Vel. Lineal X-GPS (m/s)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.gps_velocity_y, color='red')
        axs[1].set_ylabel("Vel. Lineal Y-GPS (m/s)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.gps_velocity_z, color='black')
        axs[2].set_ylabel("Vel. Lineal Z-GPS (m/s)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()
    
    def three_d_scene_gps_plot(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_title('Escena 3D del GPS')
        ax.set_xlabel('Latitud [m]')
        ax.set_ylabel('Longitud Y [m]')
        ax.set_zlabel('Altitud [m]')
        zline = [x/3.28 for x in self.gps_altitude]
        xline = self.gps_latitude
        yline = self.gps_longitude
        ax.plot3D(xline, yline, zline, 'red')
        plt.savefig("threed")
        plt.show()

    def lineal_velocity_airsim_derivate_plot(self):
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, np.gradient(self.lineal_velocity_airsim_x, self.time), color='blue')
        axs[0].set_ylabel("AirSim X Linear Velocity Derivate (m/s2)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, np.gradient(self.lineal_velocity_airsim_y, self.time), color='red')
        axs[1].set_ylabel("AirSim Y Linear Velocity Derivate (m/s2)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, np.gradient(self.lineal_velocity_airsim_z, self.time), color='black')
        axs[2].set_ylabel("AirSim Z Linear Velocity Derivate (m/s2)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()
    
    def magnetic_field_plot(self): #Ploteo del campo magnetico
        fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        axs[0].plot(self.time, self.magnetometer_x, color='blue')
        axs[0].set_ylabel("Magnetometer X (uT)", fontsize=12, color="blue")
        axs[0].grid(True, linestyle='--', alpha=0.6)

        axs[1].plot(self.time, self.magnetometer_y, color='red')
        axs[1].set_ylabel("Magnetometer Y (uT)", fontsize=12, color="red")
        axs[1].grid(True, linestyle='--', alpha=0.6)

        axs[2].plot(self.time, self.magnetometer_z, color='black')
        axs[2].set_ylabel("Magnetometer Z (uT)", fontsize=12, color="black")
        axs[2].set_xlabel("Time in s", fontsize=12)
        axs[2].grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()

        plt.show()
    
    #def trayec_def_plot(self): #Ploteo de la trayectoria con resolución definida
        #t_final = self.time[-1]
        #intervalo = (2 * cant_puntos) / t_final

        #trayectoria = self.trayec_def[0:-1:int(intervalo)]

        # Convertir cada fila en un Vector3r
        #trayectoria = [Vector3r(*fila) for fila in trayectoria]

        # 🔹 Extraer coordenadas
        #x = np.array([p.x_val for p in trayectoria])
        #y = np.array([p.y_val for p in trayectoria])
        #z = np.array([p.z_val for p in trayectoria])

        #aceleraciones = np.column_stack((self.magnetometer_x, self.magnetometer_y, self.trayec_def[:,2])) #Aceleraciones en cada punto de la trayectoria
        #cant_puntos = len(aceleraciones)
        #intervalo = (2 * cant_puntos) / t_final

        #aceleraciones = aceleraciones[0:-1:int(intervalo)]
        
       # print(aceleraciones)

        # Convertir cada fila en un Vector3r
        #aceleraciones = [Vector3r(*fila) for fila in aceleraciones]

        #ax = plt.figure().add_subplot(111, projection='3d')

        # 🔹 Graficar trayectoria
        #ax.plot(x, y, z, label="Trayectoria", color='b', linestyle='-', marker='o')

        # 🔹 Graficar vectores de aceleración en cada punto de la trayectoria
       # for pos, acc in zip(trayectoria, aceleraciones):
           # ax.quiver(pos.x_val, pos.y_val, pos.z_val, 
               #     acc.x_val, acc.y_val, acc.z_val, 
               #     color='r', length=10, normalize=False)

        #  Configuración del gráfico
        #ax.set_xlabel('X')
        #ax.set_ylabel('Y')
        #ax.set_zlabel('Z')
        #ax.set_title("Aceleraciones sobre la trayectoria de la partícula")
        #ax.legend()

        # Mostrar gráfico
        #plt.show()
    #gráfico de sensor IMU y barometro en tiempo real 
 
    def update_graphs(self, frame=None):  # Ahora acepta el argumento 'frame'
        # Ensure that the lengths of the arrays are the same
        min_length = min(len(self.time), len(self.gps_velocity_mod),len(self.barometric_altitude), len(self.gps_latitude), len(self.gps_longitude))
        # Update data of the lines in the graph
        self.line_gps_vel_mod.set_data(self.time[-min_length:], self.gps_velocity_mod[-min_length:])   
        self.line_baro.set_data(self.time[-min_length:], self.barometric_altitude[-min_length:])
        self.line_gps_lat_long.set_data(self.gps_latitude[-min_length:], self.gps_longitude [-min_length:] )

        
        for ax in self.axs:
            ax.relim()
            ax.autoscale_view()
       
        



    #--------------------------------------------------------------------------------------

    def get_time_data(self):
        self.time.append(self.sim.get_time())

    def get_pos_data(self):
        self.lat.append(self.sim.get_local_position()[0])
        self.long.append(self.sim.get_local_position()[1])
        self.alt.append(self.sim.get_local_position()[2])
    
    def get_trayec_def(self):
        self.trayec_def.append([self.sim.get_local_position()[0], self.sim.get_local_position()[1], self.sim.get_local_position()[2]])

    def get_abs_pos_data(self):
        self.lat_abs.append(self.sim[prp.lat_geod_deg])
        self.long_abs.append(self.sim[prp.lng_geoc_deg])

    def get_angle_data(self):
        self.pitch.append(self.sim.get_local_orientation()[0])
        self.roll.append(self.sim.get_local_orientation()[1])
        self.yaw.append(self.sim.get_local_orientation()[2] * (180 / math.pi))

    def get_lift_data(self):
        # normalized to ignore the aircrafts velocity
        self.clo.append(self.sim[prp.Clo] / self.sim[prp.qbar_area])
        self.clalpha.append(self.sim[prp.Clalpha] / self.sim[prp.qbar_area])
        self.clq.append(self.sim[prp.Clq] / self.sim[prp.qbar_area])
        self.clde.append(self.sim[prp.ClDe] / self.sim[prp.qbar_area])

    def get_pitch_data(self):
        self.cmo.append(self.sim[prp.Cmo] / self.sim[prp.qbar_area])
        self.cmalpha.append(self.sim[prp.Cmalpha] / self.sim[prp.qbar_area])
        self.cmq.append(self.sim[prp.Cmq] / self.sim[prp.qbar_area])
        self.cmde.append(self.sim[prp.CmDe] / self.sim[prp.qbar_area])

    def get_control_data(self):
        self.elevator_cmd.append(self.sim[prp.elevator_cmd])
        self.aileron_cmd.append(self.sim[prp.aileron_cmd])
        self.throttle_cmd.append(self.sim[prp.throttle_cmd])
        self.rudder_cmd.append(self.sim[prp.rudder_cmd])
        self.elevator.append(self.sim[prp.elevator_rad])
        self.aileron_left.append(self.sim[prp.aileron_left_rad])
        self.aileron_right.append(self.sim[prp.aileron_right_rad])
        self.aileron_combined.append(self.sim[prp.aileron_combined_rad])
        self.throttle.append(self.sim[prp.throttle])
        self.rudder.append(self.sim[prp.rudder_rad])

    def get_rate_data(self):
        self.p.append(self.sim[prp.p_radps])
        self.q.append(self.sim[prp.q_radps])
        self.r.append(self.sim[prp.r_radps])

    def get_alpha(self):
        self.alpha.append(self.sim[prp.alpha])

    def get_airspeed(self):
        self.airspeed.append(self.sim[prp.airspeed] * 0.5925)
        self.vs.append(self.sim[prp.v_down_fps] * -1 * 60)  # multiplied to fpm from fps

    def pos_plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.time, self.lat)
        ax.plot(self.time, self.long)
        ax.plot(self.time, self.alt)
        plt.show()

    def att_plot(self):
        fig, ax = plt.subplots()
        # ax.plot(self.time, self.pitch)
        ax.plot(self.time, self.roll)
        ax.plot(self.time, self.yaw)
        plt.show()

    def lift_plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.alpha, self.clo, color='green', marker='.')
        ax.plot(self.alpha, self.clalpha, color='blue', marker='.')
        ax.plot(self.alpha, self.clq, color='orange', marker='.')
        ax.plot(self.alpha, self.clde, color='red', marker='.',)
        plt.show()

    def pitch_plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.alpha, self.cmo, color='green', marker='.')
        ax.plot(self.alpha, self.cmalpha, color='blue', marker='.')
        ax.plot(self.alpha, self.cmq, color='orange', marker='.')
        ax.plot(self.alpha, self.cmde, color='red', marker='.',)
        plt.show()

    def roll_rate_plot(self):
        fig, ax = plt.subplots()
        ax.set_title('roll rate')
        ax.plot(self.time, self.p)
        plt.show()

    def pitch_rate_plot(self):
        fig, ax = plt.subplots()
        ax.set_title('pitch rate')
        ax.plot(self.time, self.q)
        plt.show()

    def control_plot(self):
        fig, ax = plt.subplots()
        ax.set_title('Throttle Control Plot')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Control deflection [-]')
        # ax.plot(self.time, self.elevator_cmd)
        # ax.plot(self.time, self.aileron_cmd)
        # ax.plot(self.time, self.throttle_cmd)
        # ax.plot(self.time, self.rudder_cmd)
        # ax.plot(self.time, self.aileron_left)
        # ax.plot(self.time, self.aileron_right)
        # ax.plot(self.time, self.aileron_combined)
        # ax.plot(self.time, self.roll)
        # ax.plot(self.time, [x * 10 for x in self.elevator])
        # ax.plot(self.time, [x * (180.0 / math.pi) for x in self.pitch])
        # ax.plot(self.time, self.throttle)
        # ax.plot(self.time, self.rudder)
        # ax.plot(self.time, self.airspeed)
        ax.plot(self.time, self.alt)
        # ax.plot(self.time, self.vs)
        # ax.plot(self.time, self.lat)
        # ax.plot(self.time, self.long)
        # ax.plot(self.time, self.yaw)
        plt.savefig("Control_plot.eps", format='eps')
        plt.show()

    def trace_plot(self):
        fig, ax = plt.subplots()
        ax.set_title('Trace Plot')
        ax.set_xlabel('Latitude [degs]')
        ax.set_ylabel('Longitude [degs]')
        ax.plot(self.lat, self.long)
        plt.savefig("grafico.png")
        print("PLOTEE")
        #plt.show()


    def trace_plot_abs(self):
        fig, ax = plt.subplots()
        ax.set_title('Trace Plot')
        ax.set_xlabel('Latitude [degs]')
        ax.set_ylabel('Longitude [degs]')
        long_m = [x * 111120.0 for x in self.long_abs]
        lat_m = [x * 111120.0 for x in self.lat_abs]
        ax.plot(long_m, lat_m)
        plt.savefig("Trace_plot")
        plt.show()

    def three_d_scene(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_title('3D plot')
        ax.set_xlabel('Latitude [degs]')
        ax.set_ylabel('Longitude [degs]')
        ax.set_zlabel('Altitude [m]')
        zline = [x / 3.28 for x in self.alt]
        xline = self.lat_abs
        yline = self.long_abs
        ax.plot3D(xline, yline, zline, 'gray')
        plt.savefig("threed")
        plt.show()


class DebugFDM:
    def __init__(self, sim):
        self.sim = sim

    def get_lift_values(self):
        print('vt: ', self.sim[prp.airspeed] * 0.5925)
        # print('Sw: ', self.sim[prp.Sw])
        # print('density: ', self.sim[prp.rho])
        # print('qbar_area: ', self.sim[prp.qbar_area])
        # print('ci2vel: ', self.sim[prp.ci2vel])
        print('alpha: ', self.sim[prp.alpha])
        print('Clo: ', self.sim[prp.Clo])
        print('Clalpha: ', self.sim[prp.Clalpha])
        print('Clq: ', self.sim[prp.Clq])
        print('ClDe: ', self.sim[prp.ClDe])
        total_lift = self.sim[prp.Clo] + self.sim[prp.Clalpha] + self.sim[prp.Clq] + self.sim[prp.ClDe]
        print('Lift = ', total_lift)

    def get_roll_values(self):
        print('vt: ', self.sim[prp.airspeed] * 0.5925)
        print('p: ', self.sim[prp.p_radps])

    def get_pitch_values(self):
        print('Cmo: ', self.sim[prp.Cmo])
        print('Cmalpha: ', self.sim[prp.Cmalpha])
        print('Cmq: ', self.sim[prp.Cmq])
        print('CmDe: ', self.sim[prp.CmDe])
        print('q: ', self.sim[prp.q_radps])
        print('ROC: ', self.sim[prp.v_down_fps] * -1 * 60)

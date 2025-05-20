
from abc import ABC
import numpy as np
import airsim
import gym
import pprint
# from tasks import Shaping
from jsbsim_simulator import Simulation
from jsbsim_aircraft import Aircraft, cessna172P, ball, x8
from debug_utils import *
import jsbsim_properties as prp
from simple_pid import PID
from autopilot import X8Autopilot
from navigation import WindEstimation#, BarometricSensor#, SensorGPS, SensorIMU #INSTANCIA SENSORES '''
from report_diagrams import ReportGraphs
from image_processing import AirSimImages, SemanticImageSegmentation
from typing import Type, Tuple, Dict
import threading
import queue
from matplotlib.animation import FuncAnimation

class ClosedLoop:
    """
    A class to run airsim, JSBSim and join the other classes together

    ...

    Attributes:
    ----------
    sim_time : float
        how many seconds to run the simulation for
    display_graphics : bool
        decides whether to run the airsim graphic update in unreal, required for image_processing input
    airspeed : float
        fixed airspeed used to fly the aircraft if airspeed_hold_w_throttle a/p used
    agent_interaction_frequency_hz : float
        how often the agent selects a new action, should be equal to or the lowest frequency
    airsim_frequency_hz : float
        how often to update the airsim graphic simulation
    sim_frequency_hz : float
        how often to update the JSBSim input, should not be less than 120Hz to avoid unexpected behaviour
    aircraft : Aircraft
        the aircraft type used, x8 by default, changing this will likely require a change in the autopilot used
    init_conditions : Dict[prp.Property, float] = None
        the simulations initial conditions None by default as in basic_ic.xml
    debug_level : int
        the level of debugging sent to the terminal by JSBSim
        - 0 is limited
        - 1 is core values
        - 2 gives all calls within the C++ source code

    Methods:
    ------
    simulation_loop(profile : tuple(tuple))
        updates airsim and JSBSim in the loop
    get_graph_data()
        gets the information required to produce debug type graphics
    generate_figures()
        produce required graphics
    """
    def __init__(self, sim_time: float,
                 display_graphics: bool = True,
                 airspeed: float = 30.0,
                 agent_interaction_frequency: float = 12.0,
                 airsim_frequency_hz: float = 392.0,    # 392 Hz is the maximum for airsim
                 sim_frequency_hz: float = 240.0,       # 240 Hz is the maximum for JSBSim
                 aircraft: Aircraft = x8,
                 init_conditions: bool = None,
                 debug_level: int = 0):
        self.sim_time = sim_time
        self.display_graphics = display_graphics
        self.airspeed = airspeed
        self.aircraft = aircraft
        self.sim: Simulation = Simulation(sim_frequency_hz, aircraft, init_conditions, debug_level)
        self.agent_interaction_frequency = agent_interaction_frequency
        self.sim_frequency_hz = sim_frequency_hz
        self.airsim_frequency_hz = airsim_frequency_hz
        self.ap: X8Autopilot = X8Autopilot(self.sim)
        self.graph: DebugGraphs = DebugGraphs(self.sim)
        self.report: ReportGraphs = ReportGraphs(self.sim)
        self.debug_aero: DebugFDM = DebugFDM(self.sim)
        self.wind_estimate: WindEstimation = WindEstimation(self.sim)

        self.over: bool = False
        self.running = False  # Control para iniciar/detener los hilos


    def simulation_loop(self, profile: tuple) -> None:
        """
        Runs the closed loop simulation and updates to airsim simulation based on the class level definitions

        :param profile: a tuple of tuples of the aircraft's profile in (lat [m], long [m], alt [feet])
        :return: None
        """
        update_num = int(self.sim_time * self.sim_frequency_hz)  # how many simulation steps to update the simulation
        relative_update = self.airsim_frequency_hz / self.sim_frequency_hz  # rate between airsim and JSBSim
        graphic_update = 0
        image = AirSimImages(self.sim)
        image.get_np_image(image_type=airsim.ImageType.Scene)
       
        for i in range(update_num):
            if not self.running:
                break
            graphic_i = relative_update * i
            graphic_update_old = graphic_update
            graphic_update = graphic_i // 1.0

            #  print(graphic_i, graphic_update_old, graphic_update)
            #  print(self.display_graphics)
           
            self.ap.airspeed_hold_w_throttle(self.airspeed)
            self.get_graph_data()

            if self.display_graphics and graphic_update > graphic_update_old:
                self.sim.update_airsim()
                # Avanzar la simulación x2
                self.sim.run()
               
            if not self.over:
                self.over = self.ap.arc_path(profile, 400)
            if self.over:
                print('over and out!')
                break
            # Avanzar la simulación x1
            self.sim.run()
            

    

    def test_loop(self) -> None:
        """
        A loop to test the aircraft's flight dynamic model

        :return: None
        """

        update_num = int(self.sim_time * self.sim_frequency_hz)  # how many simulation steps to update the simulation
        relative_update = self.airsim_frequency_hz / self.sim_frequency_hz  # rate between airsim and JSBSim
        graphic_update = 0

        for i in range(update_num):
            graphic_i = relative_update * i
            graphic_update_old = graphic_update
            graphic_update = graphic_i // 1.0
            #  print(graphic_i, graphic_update_old, graphic_update)
            #  print(self.display_graphics)
            #if self.display_graphics and graphic_update > graphic_update_old:
            #    self.sim.update_airsim()
                # print('update_airsim')
            # elevator = 0.0
            # aileron = 0.0
            # tla = 0.0
            # self.ap.test_controls(elevator, aileron, tla)
            # self.ap.altitude_hold(1000)
            # self.ap.heading_hold(0)
            # self.ap.roll_hold(5 * math.pi / 180)
            # self.ap.pitch_hold(5.0 * math.pi / 180.0)
            if self.sim[prp.sim_time_s] >= 5.0:
            # self.ap.heading_hold(120.0)
            #     self.ap.roll_hold(0.0 * math.pi / 180.0)
                self.ap.airspeed_hold_w_throttle(self.airspeed)
                # self.ap.pitch_hold(10.0 * (math.pi / 180.0))
                self.ap.altitude_hold(800)
            self.get_graph_data()
            self.sim.run()



    def get_graph_data(self) -> None:
        """
        Gets the information required to produce debug type graphics

        :return:
        """

        self.graph.get_barometric_alt() #INSTANCIA SENSOR BAROMETRICO
        self.graph.get_alt_pressure_Airsim() #INSTANCIA DE PARAMETRO DE ALTITUD DE Airsim
        self.graph.get_barometric_pressure() #INSTANCIA SENSOR BAROMETRICO
        self.graph.get_pressure_sim() #INSTANCIA DE PARAMETRO DE PRESION DEL SIMULADOR
        self.graph.get_imu_lineal_acc() #INSTANCIA DE ACELERACION LINEAL
        self.graph.get_imu_angular_vel() #INSTANCIA DE VELOCIDAD ANGULAR
        self.graph.get_angular_vel_jsbsim() #INSTANCIA DE VELOCIDAD ANGULAR DE JSBSIM
        self.graph.get_angular_vel_Airsim() #INSTANCIA DE VELOCIDAD ANGULAR DE Airsim
        self.graph.get_lineal_acc_Airsim() #INSTANCIA DE ACELERACION LINEAL DE Airsim
        self.graph.get_lineal_acc_jsbsim() #INSTANCIA DE ACELERACION LINEAL DE JSBSIM
        self.graph.get_linear_velocity_airsim() #INSTANCIA DE VELOCIDAD LINEAL DE Airsim
        self.graph.get_magnetometer() #INSTANCIA DE MAGNETOMETRO
        self.graph.get_gps_data() #INSTANCIA DE DATOS GPS
        self.graph.get_abs_pos_data()
        self.graph.get_airspeed()
        self.graph.get_alpha()
        self.graph.get_control_data()
        self.graph.get_time_data()
        self.graph.get_pos_data()
        #self.graph.get_trayec_def()
        self.graph.get_angle_data()
        self.graph.get_rate_data()
        self.report.get_graph_info()
        

       
     

    def generate_figures(self) -> None:
        """
        Produce required graphics, outputs them in the desired graphic environment

        :return: None
        """
        #self.graph.trayec_def_plot() #Ploteo de trayectoria       
        self.graph.barometric_alt_plot() #Ploteo de altitud barométrica
        #self.graph.alt_pressure_Airsim_plot() #Ploteo de altitud de JSBSim
        self.graph.barometric_pressure_plot() #Ploteo de presión barométrica
        #self.graph.pressure_sim_plot() #Ploteo de presión de JSBSim
        #self.graph.lineal_velocity_airsim_plot() #Ploteo de velocidad lineal de Airsim
        self.graph.lineal_velocity_gps_plot() #Ploteo de velocidad lineal de GPS
        self.graph.imu_lineal_acc_plot() #Ploteo de aceleración lineal
        #self.graph.lineal_acc_airsim_plot() #Ploteo de aceleración lineal de Airsim
        #self.graph.lineal_velocity_airsim_derivate_plot() #Ploteo de velocidad lineal de Airsim derivada
        #self.graph.lineal_acc_jsbsim_plot() #Ploteo de aceleración lineal de JSBSim 
        self.graph.imu_angular_vel_plot() #Ploteo de velocidad angular
        #self.graph.angular_vel_jsbsim_plot() #Ploteo de velocidad angular de JSBSim
        #self.graph.angular_vel_airsim_plot() #Ploteo de velocidad angular de Airsim
        self.graph.magnetic_field_plot() #Ploteo de campo magnético
        #self.graph.control_plot()
        #self.graph.trace_plot_abs()
        self.graph.three_d_scene()
        self.graph.three_d_scene_gps_plot()
        #self.graph.pitch_rate_plot()
        #self.graph.roll_rate_plot()
        #self.graph.roll_rate_plot()
        #self.debug_aero.get_pitch_values()


    
      

#controla el entorno de simulación
def run_simulator() -> None:
    """
    Runs the JSBSim and Airsim in the loop when executed as a script

    :return: None
    """
    env = ClosedLoop(750, True)
    
    #Trayectoria helicoidal:
    A = 250 #radio de helicoide 
    h = 100 #altura de helicoide
    N = 2 #número de vueltas
    t_values = np.linspace(0, 2*np.pi*N, 12) #12 puntos hacen una estrella aprox   
    scale = 1

    helicoid_profile = [(A*np.cos(t) * scale, A*np.sin(t)*scale, h*t/(2*np.pi*N)*scale) for t in t_values]

    square_mytest = ((0, 0, 0), (150, 150, 20), (0, 300, 20))#, (-300, 300, 30), (-300, -300, 30), (300, -300, 30), (300, 300, 30), (-300,300,30))

    circuit_profile = ((0, 0, 0), (400, 0, 100), (400, 400, 100), (0, 400, 100), (0, 0, 200), (400, 0, 200),
                       (400, 400, 200), (0, 400, 200), (0, 0, 300), (400, 0, 300),
                       (400, 400, 300), (0, 400, 300), (0, 0, 400), (400, 0, 400),
                    (400, 400, 400), (0, 400, 400))
    circuit_profile_mod = ((0, 0, 0), (0, -400, 100), (400, 400, 100))
    ice_profile = ((0, 0, 0), (1200, 0, 0), (1300, 150, 0), (540, 530, -80), (0, 0, -150), (100, 100, -100))
    square = ((0, 0, 0), (1000, 0, 30), (1000, 1000, 30), (0, 1000, 30), (0, 0, 30), (1000, 0, 30), (200, 200, 30))

    approach = ((0, 0, 0), (2000, 0, 800), (2000, 2000, 600), (0, 2000, 400), (0, 0, 200), (2000, 0, 100),
                (2000, 2000, 100), (0, 2000, 100), (0, 0, 100))
    rectangle = ((0, 0, 0), (0, 300, 30), (-350, 151, 30), (-700, 0, 30), (-350, 1, 30), (0, 0, 30), (0, 300, 50),(-350, 151, 100))
    straight = ((0, 0, 0), (2000, 0, 1000), (2000, 2000, 500))


    #env.simulation_loop(circuit_profile_mod)

    # FUNCIÓN DE ACTUALIZACIÓN DE GRÁFICOS CON FUNCANIMATION
    def update_frame(frame):
        if not env.running:
            ani.event_source.stop()
            return
        env.graph.update_graphs()
    # Inicio de simulación en hilo principal 
    env.running = True
    sim_thread = threading.Thread(target=env.simulation_loop, args=(helicoid_profile,))
    sim_thread.start()
    #Declaración de  FuncAnimation(hilo secundario)
    ani = FuncAnimation(env.graph.fig, update_frame, interval=3250)  # Actualiza cada 100 ms
    plt.show(block=True)

    sim_thread.join()
    env.running = False
    env.generate_figures()
    env.report.trace_plot(helicoid_profile)
    env.report.control_response(0, 750, 240)

    plt.show(block=True)  # Mantener ventana abierta
    #env.report.three_d_plot(0, 3500, 240)
    print('Simulation ended')


def run_simulator_test() -> None:
    """
    Runs JSBSim in the test loop when executed as a script to test the FDM

    :return: None
    """
    sim_frequency = 240
    env = ClosedLoop(65.0, True, 30, 12, 24, sim_frequency)
    env.test_loop()
    env.generate_figures()
    print('Simulation ended')


if __name__ == '__main__':
    #run_simulator_test()
    run_simulator()




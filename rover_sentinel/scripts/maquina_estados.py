#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs import *
from nav_msgs import *
#from rover_slam import *
from sensor_msgs import *
import smach
import smach_ros


class StateMachine:
    def __init__(self):
        rospy.init_node("Maquina_estados_py")
        
        self.state = "EXPLORACION"
        
        self.sound_subscriber = rospy.Subscriber("/sound_sensor", bool, self.sound_callback)
        self.detect_subscriber = rospy.Subscriber("/detect_person", bool, self.image_callback)
        self.map_subscriber = rospy.Subscriber("/map", bool, self.map_callback)
        self._manual_subscriber = rospy.Subscriber("/manual", bool, self.manual_callback)


        self.map_built = False
        self.sound_detected = False
        self.person_detected = False
        self.manual_detected = False

    def manual_callback(self, msg:bool) -> None:
        # Lógica para determinar si se ha detectado un sonido
        if msg == True:
            self.manual_detected = True
        else:
            self.manual_detected = False

    def sound_callback(self, msg:bool) -> None:
        # Lógica para determinar si se ha detectado un sonido
        if msg == True:
            self.sound_detected = True
        else:
            self.sound_detected = False

    def image_callback(self, msg:bool) -> None:
        # Lógica para determinar si se ha detectado una persona
        if msg == True:
            self.person_detected = True
        else:
            self.person_detected = False


    def map_callback(self,msg:bool) -> None:
        # Lógica para determinar si el mapa está construido
        if msg == True:
            self.map_built = True
        else:
            self.map_built = False
        

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.state == "EXPLORACION":
                if self.map_built:
                    self.state = "NAVEGACION"
                self.exploracion()
            
            elif self.state == "NAVEGACION":
                if self.manual_detected:
                    self.state = "MANUAL"
                elif self.person_detected:
                    self.state = "ALERTA2"
                elif self.sound_detected:
                    self.state = "ALERTA1"
                self.navegacion()
            
            elif self.state == "MANUAL":
                if self.manual_detected == False:
                    self.state = "NAVEGACION"
                self.manual()

            elif self.state == "ALERTA1":
                if self.person_detected:
                    self.state = "ALERTA2"
                elif not self.sound_detected:
                    self.state = "NAVEGACION"
                self.alerta1()
            
            elif self.state == "ALERTA2":
                self.alerta2()
            
            rate.sleep()

    def exploracion(self):
        rospy.loginfo("Estado: EXPLORACION - Construyendo el mapa...")
        # Lógica de exploración para construir el mapa
        pass

    def navegacion(self):
        rospy.loginfo("Estado: NAVEGACION - Navegando...")
        # Lógica de navegación
        pass

    def manual(self):
        rospy.loginfo("Estado: MANUAL - Manual...")
        pass

    def alerta1(self):
        rospy.loginfo("Estado: ALERTA1 - Sonido detectado, investigando...")
        # Lógica para manejar la alerta por sonido
        self.sound_detected = False  # Simulamos que el sonido ya no se detecta

    def alerta2(self):
        rospy.loginfo("Estado: ALERTA2 - Persona detectada, no puede salir de este estado.")
        # Lógica para manejar la alerta por detección de persona
        pass

if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.run()

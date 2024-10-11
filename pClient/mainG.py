
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import statistics
import time

CELLROWS=7
CELLCOLS=14


# Filtros
###################################################################
class FilterSensor_left:
    def __init__(self, amostragem):
        self.amostragem = amostragem
        self.values = []

    def add_value_l(self, new_value):
        if len(self.values) >= self.amostragem:
            self.values.pop(0)  #pop remove o valor mais antigo === FIFO
        self.values.append(new_value)
        #sensor_l_string = self.values
        #return sensor_l_string
        return self.values

    def get_filtered_value_l(self):
        #return sum(self.values) / len(self.values) if self.values else 0
        return statistics.median(self.values) if self.values else 1/0.9

filter_ir_sensor_l = FilterSensor_left(amostragem=5)

class FilterSensor_right:
    def __init__(self, amostragem):
        self.amostragem = amostragem
        self.values = []

    def add_value_r(self, new_value):
        if len(self.values) >= self.amostragem:
            self.values.pop(0)  
        self.values.append(new_value)
        #sensor_r_string = self.values
        #return sensor_r_string
        return self.values


    def get_filtered_value_r(self):
        #return sum(self.values) / len(self.values) if self.values else 0
        return statistics.median(self.values) if self.values else 1/0.9

filter_ir_sensor_r = FilterSensor_right(amostragem=5)

class FilterSensor_center:
    def __init__(self, amostragem):
        self.amostragem = amostragem
        self.values = []

    def add_value_c(self, new_value):
        if len(self.values) >= self.amostragem:
            self.values.pop(0)  
        self.values.append(new_value)
        #sensor_c_string = self.values
        #return sensor_c_string
        return self.values

    def get_filtered_value_c(self):
        #return sum(self.values) / len(self.values) if self.values else 0
        return statistics.median(self.values) if self.values else 1/1.6

filter_ir_sensor_c = FilterSensor_center(amostragem=5)
#############################################################################


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.previous_error = 0  # Last error (for derivative)
        self.integral = 0  # Sum of errors (for integral)

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.pid_controller = PIDController(0.25, 0.0, 0.0)
        self.last_time = time.time()

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def printData(sensor_data):
        print("\033[H", end="")  # Move o cursor para o topo da tela
        print("Sensores")
        print("=====================")
        for sensor, value in sensor_data.items():
            print(f"{sensor}: {value}")


    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        self.last_time = time.time()

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                #____________________# 
                # TESTATANDO BASES
                # if self.measures.ground==0:
                #     #print(self.measures.ground)
                #     #print('base 1')
                #     self.base = '1'
                #     #self.setVisitingLed(True);
                # if self.measures.ground==1:
                #     #print(self.measures.ground)
                #     #print('base 2')
                #     #self.setVisitingLed(True);
                #     self.base = '2'
                # elif self.measures.ground==2:
                #     #print(self.measures.ground)
                #     #print('base 3')
                #     #self.setVisitingLed(True)
                #     self.base = '3'
                if self.measures.ground != -1:
                    self.base = self.measures.ground + 1
                else:
                    self.base = '-'
                #____________________#
                
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            
    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        base_velocity = 0.15
        danger_walls = 1.8
        danger_front = 0.95
        # Para identificar no crusamento
        commun_dist = 0.576

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        filter_ir_sensor_c.add_value_c(self.measures.irSensor[center_id])
        filter_ir_sensor_l.add_value_l(self.measures.irSensor[left_id])
        filter_ir_sensor_r.add_value_r(self.measures.irSensor[right_id])        
        
        filtered_left_value = filter_ir_sensor_l.get_filtered_value_l()
        filtered_right_value = filter_ir_sensor_r.get_filtered_value_r()
        filtered_center_value = filter_ir_sensor_c.get_filtered_value_c()    

        # Pegar a copia da sting do sensor para usar no crusamento
        #sensor_string_c = filter_ir_sensor_c.add_value_c()
        sensor_string_c = filter_ir_sensor_c.values
        #sensor_string_l = filter_ir_sensor_l.add_value_l()
        sensor_string_l = filter_ir_sensor_l.values
        sensor_string_r = filter_ir_sensor_r.values


        
        # last_value_c = sensor_string_c[-1]         
        # last_value_l = sensor_string_l[-1] 
        # last_value_r = sensor_string_r[-1]
        last_value_c = self.measures.irSensor[center_id]         
        last_value_l = self.measures.irSensor[left_id] 
        last_value_r = self.measures.irSensor[right_id] 

        if dt <= 0:
            dt = 0.01
        
        # Em casos de curvas
        if (last_value_c > filtered_center_value + 1/0.4) | (last_value_c < filtered_center_value + 1/0.4):
            front_proximity = last_value_c
        else:
            front_proximity = filtered_center_value    

        if (last_value_r > filtered_right_value + 1/0.8) | (last_value_r < filtered_right_value - 1/0.8):
            right_proximity = last_value_r
        else:
            right_proximity = filtered_right_value

        if (last_value_l > filtered_left_value + 1/0.8) | (last_value_l < filtered_left_value - 1/0.8):
            left_proximity = last_value_l
        else:
            left_proximity = filtered_left_value

        # front_proximity = self.measures.irSensor[center_id]
        # left_proximity = self.measures.irSensor[left_id]
        # right_proximity = self.measures.irSensor[right_id]

        #print('Front: '+str(front_proximity)+' Left: '+str(left_proximity)+' Right: '+str(right_proximity)+' Colision: '+str(self.measures.collision))

        error = right_proximity-left_proximity

        pid_output = self.pid_controller.compute(error, dt)
        #print(pid_output)

        # if (left_proximity < danger_walls) & (right_proximity < danger_walls) & (front_proximity < danger_front): # Crossways logic
        #     left_proximity = 1/0.9
        #     right_proximity = 1/0.9 
        #     self.situation = 'Crossway'

        if (left_proximity < danger_walls) & (right_proximity < danger_walls) & (front_proximity < danger_front): # Crossways logic
            #print('Crossway')
            self.situation = 'Crossway'
            self.driveMotors(base_velocity,base_velocity)
        elif front_proximity > danger_front:
            if error > 0:
                #print('Sharp left')
                self.situation = 'Sharp Left'
                self.driveMotors(max(base_velocity - pid_output,-base_velocity),+base_velocity)
            elif error < 0:
                #print ('Sharp right')
                self.situation = 'Sharp right'
                self.driveMotors(+base_velocity,max(base_velocity+pid_output,-base_velocity))
        else:
            if error > 1.0:
                #print('Smooth left')
                self.situation = 'Smooth left'
                self.driveMotors(max(base_velocity - pid_output,-0),+base_velocity)
            elif error < -1.0:
                #print ('Smooth right')
                self.situation = 'Smooth right'
                self.driveMotors(+base_velocity,max(base_velocity+pid_output,-0))
            else:
                #print('Go')
                self.situation = 'Go'
                self.driveMotors(base_velocity,base_velocity)
            
        sensor_data = {
            "Sensor data Left" :sensor_string_l,
            "Sensor left" :filtered_left_value,
            "Sensor data Right" :sensor_string_r,
            "Sensor Right" :filtered_right_value,
            "Sensor data Center" :sensor_string_c,
            "Sensor Center" :filtered_center_value,
            "Base":self.base,
            "situação" : self.situation,
        }
        MyRob.printData(sensor_data)



class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

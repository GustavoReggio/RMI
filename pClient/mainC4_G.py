import sys
from croblink import *
import math
#from math import ceil ,floor
import xml.etree.ElementTree as ET
import time
import threading

CELLROWS=7
CELLCOLS=14


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

class FilterSensor:
    def __init__(self, amostragem):
        self.amostragem = amostragem
        self.sensors = {
            'left': [],
            'right': [],
            'center': [],
            'back': [],
        }

    def add_value(self, sensor_id, new_value):
        values = self.sensors[sensor_id]
        if len(values) >= self.amostragem:
            values.pop(0)  # Remove o valor mais antigo
        values.append(new_value)

    def get_filtered_value(self, sensor_id):
        values = self.sensors[sensor_id]
        if values:
            mean = sum(values)/len(values)
        else:
            mean = 0 
        return  mean

sensor_filter = FilterSensor(amostragem = 5)

class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        # Initialize state and covariance
        self.x = initial_state  # Initial state (orientation)
        self.P = initial_covariance  # Initial uncertainty

        # Process noise (Q) and measurement noise (R)
        self.Q = process_noise
        self.R = measurement_noise

    def predict(self, control_input, delta_t):
        """
        Prediction step of the Kalman Filter.

        Parameters:
        control_input (float): Angular velocity (e.g., control command).
        delta_t (float): Time step duration.
        """
        # Update the state based on the motion model
        self.x += control_input * delta_t

        # Update the covariance estimate
        self.P += self.Q

    def update(self, measurement):
        """
        Update step of the Kalman Filter.

        Parameters:
        measurement (float): Noisy orientation measurement.
        """
        # Measurement residual
        y = measurement - self.x

        # Kalman gain
        S = self.P + self.R
        K = self.P / S

        # Update state estimate
        self.x += K * y

        # Update covariance
        self.P = (1 - K) * self.P

    def get_state(self):
        """
        Returns the current estimated state.
        """
        return self.x

kf = KalmanFilter(
    initial_state = 0,  # Start at orientation 0
    initial_covariance = 0,  # Initial uncertainty
    process_noise = 1.5,  # Process noise covariance
    measurement_noise = 2 # Measurement noise covariance
)


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.pid_controller = PIDController(0.25, 0.0, 0.0)
        self.last_time = time.time()
        self.previous_out_left = 0
        self.previous_out_right = 0
        self.previous_x = 0
        self.previous_y = 0
        self.previous_orientation = 0
        self.flag = False
        self.first_loop = True
        self.wall_vertical_pos_aprox = [0,0]
        self.front_distance = 0 
        self.counter = 0  
        
        

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def calibratePosition(self):
       self.x_offset = self.measures.x
       self.y_offset = -self.measures.y

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
                if self.measures.ground==0:
                    self.setVisitingLed(True);
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
        front_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        # base_velocity = 0.15
        pi = math.pi

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 0.01

        orientation_rad = self.measures.compass * pi / 180

        sensor_filter.add_value('center', self.measures.irSensor[front_id])
        sensor_filter.add_value('left', self.measures.irSensor[left_id])
        sensor_filter.add_value('right', self.measures.irSensor[right_id])
        sensor_filter.add_value('back', self.measures.irSensor[back_id])

        front_proximity = sensor_filter.get_filtered_value('center')
        left_proximity = sensor_filter.get_filtered_value('left')
        right_proximity = sensor_filter.get_filtered_value('right')
        back_proximity = sensor_filter.get_filtered_value('back')

        try :
            self.front_distance = 1 / front_proximity
        except:
            self.front_distance = 20
        try :
            self.left_distance = 1 / left_proximity
        except:
            self.left_distance = 20
        try :
            right_distance = 1 / right_proximity
        except:
            right_distance = 20
        try :
            back_distance = 1 / back_proximity
        except:
            back_right_distance = 20



        if self.front_distance < 1:
            speed_left = -0.1
            speed_right = 0.1
            self.flag = True
        elif self.flag:
            speed_left = 0
            speed_right = 0
        else:
            speed_left = 0.1
            speed_right = 0.1

        self.driveMotors(speed_left, speed_right)

        #-------------- Movement Model --------------#
        out_left = (speed_left + self.previous_out_left) / 2
        out_right = (speed_right + self.previous_out_right) / 2

        lin_speed = (out_left + out_right) / 2
        self.x_estimate = self.previous_x + lin_speed * math.cos(self.previous_orientation)
        self.y_estimate = self.previous_y + lin_speed * math.sin(self.previous_orientation)

        rot_speed = (out_right - out_left) * 180 / pi
        orientation_estimation = self.previous_orientation + rot_speed

        self.previous_out_left = out_left
        self.previous_out_right = out_right
        # if orientation_estimation > 3.14:
        #     self.previous_orientation = orientation_estimation - 6.28
        # elif orientation_estimation < -3.14:
        #     self.previous_orientation = orientation_estimation + 6.28
        # else:
        #     self.previous_orientation = orientation_estimation
        if orientation_estimation >-90 and orientation_estimation < 90:
            if self.x_estimate < self.previous_x:
                self.x_estimate = self.previous_x

            self.previous_x = self.x_estimate
            self.previous_y = self.y_estimate

        orientation_rad = self.measures.compass #* pi 
        #-------------- --------------#

        #-------------- mplement and test compass and beacon sensor integration for position estimation. --------------#
        if self.first_loop:
            print('VALOR INICIAL X GLOBAL')
            print('GLOBALX:'+ str(self.measures.x) +'  Y:'+ str(self.measures.y)+'\n\n')
            self.calibratePosition()
            self.first_loop = False

        
        self.x_position = int(round(self.measures.x - self.x_offset))
        self.y_position = int(round(-self.measures.y - self.y_offset))
        print('GPS->  X:'+ str(self.x_position) +'  Y:'+ str(self.y_position))
        #print('X: ' + str(x_estimate) + ' Y: ' + str(y_estimate) + ' Ori: ' + str(orientation_rad) + ' Ori_est: ' + str(orientation_estimation))
        print(f'X: {self.x_estimate:.4f} Y: {self.y_estimate:.4f} Ori: {orientation_rad:.4f} Ori_est: {orientation_estimation:.4f}')
        # print(self.measures.beacon)

        kf.predict(rot_speed, dt)
        kf.update(orientation_rad)
        filteredOrientation = kf.get_state()
        self.previous_orientation = filteredOrientation
        print(filteredOrientation)

        print(f'sensor front: {self.front_distance}')
        print(f'sensor left: {self.left_distance}')
        self.wall_vertical_pos = self.x_estimate + self.front_distance + 0.6
        self.wall_vertical_pos_aprox = [math.ceil(self.wall_vertical_pos),math.floor(self.wall_vertical_pos)]
        print(f'Posição x paredes {self.wall_vertical_pos_aprox}')
        
        #Cheking X axis
        # if orientation_estimation > -3 and orientation_estimation < 3:
        #     if 
        if self.front_distance <=0.96:
            self.counting_time()
            self.counter=0

        # self.counter = self.counter+1
        # if self.counter ==10:
        #     self.counting_time()
        #     self.counter=0
        

        print('\n')
        # intervalo_verificacao = 3
        # # Cria e inicia um thread para a verificação dos sensores
        # sensor_thread = threading.Thread(target=self.counting_time, args=(intervalo_verificacao,))
        # sensor_thread.daemon = True  # O programa principal pode encerrar mesmo com o thread rodando
        # sensor_thread.start()
        #-------------- --------------#
    
    def counting_time(self):

        # #mudar logica do par da parede 
        if self.wall_vertical_pos_aprox[0] %2!=0:
            self.x_checked = self.wall_vertical_pos_aprox[0] - self.front_distance -0.6
        else:
            self.x_checked = self.wall_vertical_pos_aprox[1] - self.front_distance -0.6
        print('!!!!!!!!!!!!!!!!!!!')
        print(f'X checado: {self.x_checked:.4f}')
        if self.x_checked <=self.x_estimate:
            self.x_checked= self.x_estimate
        self.previous_x  = self.x_checked
       


       

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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

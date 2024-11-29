import sys
from croblink import *
# import math
from math import *
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
        # pi = math.pi

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

        front_proximity = abs(sensor_filter.get_filtered_value('center'))
        left_proximity = abs(sensor_filter.get_filtered_value('left'))
        right_proximity = abs(sensor_filter.get_filtered_value('right'))
        back_proximity = abs(sensor_filter.get_filtered_value('back'))

        try :
            self.front_distance = 1 / front_proximity
        except:
            self.front_distance = 20
        try :
            self.left_distance = 1 / left_proximity
        except:
            self.left_distance = 20
        try :
            self.right_distance = 1 / right_proximity
        except:
            self.right_distance = 20
        try :
            self.back_distance = 1 / back_proximity
        except:
            self.back_distance = 20



        if self.front_distance < 1:
            speed_left = -0.1
            speed_right = 0.1
            # self.flag = True
        elif self.flag:
            speed_left = 0
            speed_right = 0
        else:
            speed_left = 0.1
            speed_right = 0.1


        #-------------- Movement Model --------------#

        speed_wheel_L =  speed_left
        speed_wheel_R =  speed_right
        self.driveMotors(speed_wheel_L, speed_wheel_R)
       
        # out_left = (speed_left + self.previous_out_left) / 2
        out_left = (speed_wheel_L + self.previous_out_left) / 2
        # out_right = (speed_right + self.previous_out_right) / 2
        out_right = (speed_wheel_R + self.previous_out_right) / 2

        lin_speed = (out_left + out_right) / 2
        self.x_estimate = self.previous_x + lin_speed * cos(radians(self.previous_orientation))
        self.y_estimate = self.previous_y + lin_speed * sin(radians(self.previous_orientation))

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

        self.previous_x = self.x_estimate
        self.previous_y = self.y_estimate
        orientation_rad = self.measures.compass #* pi 
        #----------------------------#

        #-------------- Implement and test compass and beacon sensor integration for position estimation. --------------#
        if self.first_loop:
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
        print(f'filter ori: {filteredOrientation:.4f}')
        print('')
        print(f'sensor front: {self.front_distance}')
        print(f'sensor back: {self.back_distance}')
        print(f'sensor left: {self.left_distance}')
        print(f'sensor right {self.right_distance}')
        print('')
        
        #Cheking X and Y horizontaly
        if orientation_estimation > -3 and orientation_estimation  < 3:
            ori_ = 'horizontal'
            print('!!!!!!!!- HORIZONTAL -!!!!!!!!!!!')
            if self.front_distance <=1.125 or self.back_distance <=1.125:
                self.cheking_X(pos=ori_)
            if self.right_distance <=1.125 or self.left_distance <1.125:
                self.cheking_Y(pos=ori_)

        #Cheking X and Y verticaly
        if orientation_estimation > 87 and orientation_estimation  < 93:
            ori_ = 'vertical'
            print('!!!!!!!!- Vertical -!!!!!!!!!!!')
            if self.front_distance <=1.125 or self.back_distance <=1.125:
                self.cheking_Y(pos=ori_)
            if self.right_distance <=1.125 or self.left_distance <1.125:
                self.cheking_X(pos=ori_)
                                      
        

        print('\n')
        #-------------- --------------#
    
    def cheking_X(self,pos):
        match pos:
            #----------------------------#
            case 'horizontal':
                #correctinf sensor                
                if self.front_distance <=1.125:
                    distancia_robo = self.front_distance + 0.6
                else:
                    distancia_robo = -(self.back_distance + 0.6)
                print('!! - X - !!')
                self.wall_vertical_pos = self.x_estimate + distancia_robo
                self.wall_vertical_pos_aprox = [ceil(self.wall_vertical_pos),floor(self.wall_vertical_pos)]
                print(f'Posição x paredes {self.wall_vertical_pos_aprox}')

                # Rever este critério
                if self.wall_vertical_pos_aprox[0] %2==0:
                    self.x_checked = self.wall_vertical_pos_aprox[0] - distancia_robo
                else:
                    self.x_checked = self.wall_vertical_pos_aprox[1] - distancia_robo
                # if self.x_checked <=self.x_estimate:
                #     self.x_checked= self.x_estimate
                # self.previous_x  = self.x_checked
                print(f'X = {self.x_checked:.4f}')
                # if abs(self.x_checked )- floor(abs(self.x_checked)) <= 0.5:
                #     self.previous_x  = floor(self.x_checked)
                # else:
                #     self.previous_x  = ceil(self.x_checked)
                if abs(self.x_checked) - floor(abs(self.x_checked)) <= 0.5:
                    self.previous_x = self.round_towards_zero(self.x_checked)
                else:
                    self.previous_x = self.round_towards_zero(self.x_checked) + (1 if self.x_checked > 0 else -1)
                print(f'X checado: {self.previous_x:.4f}')
            #----------------------------#
            case 'vertical':
                print('!! - X - !!')
                if self.right_distance > 1.125:
                    distancia_r = self.left_distance
                else:
                    distancia_r = self.right_distance

                if self.left_distance > 1.125:
                    disancia_l = self.right_distance
                else:
                    disancia_l = self.left_distance
                #------------------- Left
                self.wall_vertical_pos_l = self.x_estimate -(disancia_l + 0.6)
                self.wall_vertical_pos_aprox_l = [ceil(self.wall_vertical_pos_l),floor(self.wall_vertical_pos_l)]
                print(f'Posição X paredes L {self.wall_vertical_pos_aprox_l}')
                # Rever este critério
                if self.wall_vertical_pos_aprox_l[0] %2!=0:
                    self.x_checked_l = self.wall_vertical_pos_aprox_l[0] + (disancia_l + 0.6)
                else:
                    self.x_checked_l = self.wall_vertical_pos_aprox_l[1] + (disancia_l + 0.6)
                print(f'X L checado: {self.x_checked_l:.4f}')
                #------------------- Right
                self.wall_vertical_pos_r = self.x_estimate + ( distancia_r + 0.6)
                self.wall_vertical_pos_aprox_r = [ceil(self.wall_vertical_pos_r),floor(self.wall_vertical_pos_r)]
                print(f'Posição X paredes R {self.wall_vertical_pos_aprox_r}')
                # Rever este critério
                if self.wall_vertical_pos_aprox_r[0] %2!=0:
                    self.x_checked_r = self.wall_vertical_pos_aprox_r[0] - (distancia_r + 0.6)
                else:
                    self.x_checked_r = self.wall_vertical_pos_aprox_r[1] - (distancia_r + 0.6)
                print(f'X R checado: {self.x_checked_r:.4f}')   
                #------------------- CHECK         
                self.x_checked = (self.x_checked_l + self.x_checked_r)/2
                print(f'X= {self.x_checked:.4f}')
                if self.x_checked - floor(self.x_checked) <= 0.5:
                    self.previous_x  = floor(self.x_checked)
                else:
                    self.previous_x  = ceil(self.x_checked)
                print(f'X checado: {self.previous_x:.4f}')
            #----------------------------#


    def cheking_Y(self,pos):
        match pos:
            #----------------------------#
            case 'horizontal':
                if self.right_distance > 1.125:
                    distancia_r = self.left_distance
                else:
                    distancia_r = self.right_distance

                if self.left_distance > 1.125:
                    disancia_l = self.right_distance
                else:
                    disancia_l = self.left_distance
                #------------------- Left
                print('!! - Y - !!')
                self.wall_horiszoantal_pos_l = self.y_estimate + disancia_l + 0.6
                self.wall_horizontal_pos_aprox_l = [ceil(self.wall_horiszoantal_pos_l),floor(self.wall_horiszoantal_pos_l)]
                print(f'Posição Y paredes L {self.wall_horizontal_pos_aprox_l}')
                # Rever este critério
                if self.wall_horizontal_pos_aprox_l[0] %2!=0:
                    self.y_checked_l = self.wall_horizontal_pos_aprox_l[0] - disancia_l -0.6
                else:
                    self.y_checked_l = self.wall_horizontal_pos_aprox_l[1] - disancia_l -0.6
                print(f'Y L checado: {self.y_checked_l:.4f}')
                #------------------- Right
                self.wall_horiszoantal_pos_r = self.y_estimate -( distancia_r + 0.6)
                self.wall_horizontal_pos_aprox_r = [ceil(self.wall_horiszoantal_pos_r),floor(self.wall_horiszoantal_pos_r)]
                print(f'Posição Y paredes R {self.wall_horizontal_pos_aprox_r}')
                # Rever este critério
                if self.wall_horizontal_pos_aprox_r[0] %2!=0:
                    self.y_checked_r = self.wall_horizontal_pos_aprox_r[0] + distancia_r +0.6
                else:
                    self.y_checked_r = self.wall_horizontal_pos_aprox_r[1] + distancia_r +0.6
                print(f'Y R checado: {self.y_checked_r:.4f}')   
                #------------------- CHECK         
                self.y_checked = (self.y_checked_l + self.y_checked_r)/2
                print(f'Y= {self.y_checked:.4f}')
                if self.y_checked - floor(self.y_checked) <= 0.5:
                    self.previous_y  = floor(self.y_checked)
                else:
                    self.previous_y  = ceil(self.y_checked)
                print(f'Y checado: {self.previous_y:.4f}')
            #----------------------------#
            case 'vertical':
                print('!! - Y - !!')
                #correctinf sensor                
                if self.front_distance <=1.125:
                    distancia_robo = self.front_distance + 0.6
                else:
                    distancia_robo = -(self.back_distance + 0.6)
                self.wall_horizontal_pos = self.y_estimate + distancia_robo
                self.wall_horizontal_pos_aprox = [ceil(self.wall_horizontal_pos),floor(self.wall_horizontal_pos)]
                print(f'Posição Y paredes {self.wall_horizontal_pos_aprox}')

                # Rever este critério
                if self.wall_horizontal_pos_aprox[0] %2==0:
                    self.y_checked = self.wall_horizontal_pos_aprox[0] - distancia_robo
                else:
                    self.y_checked_checked = self.wall_horizontal_pos_aprox[1] - distancia_robo
                # if self.x_checked <=self.x_estimate:
                #     self.x_checked= self.x_estimate
                # self.previous_x  = self.x_checked
                print(f'Y = {self.y_checked:.4f}')
                # if abs(self.x_checked )- floor(abs(self.x_checked)) <= 0.5:
                #     self.previous_x  = floor(self.x_checked)
                # else:
                #     self.previous_x  = ceil(self.x_checked)
                if abs(self.y_checked) - floor(abs(self.y_checked)) <= 0.5:
                    self.previous_y = self.round_towards_zero(self.y_checked)
                else:
                    self.previous_y = self.round_towards_zero(self.y_checked) + (1 if self.y_checked > 0 else -1)
                print(f'Y checado: {self.previous_y:.4f}')
            #----------------------------#
        
    def round_towards_zero(self,value):
        if value >= 0:
            return floor(value)  # Para valores positivos, usa floor
        else:
            return ceil(value)   # Para valores negativos, usa ceil
   


       

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


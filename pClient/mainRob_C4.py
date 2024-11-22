import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from random import *

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
        # Update the state based on the motion model
        self.x += control_input * delta_t

        # Update the covariance estimate
        self.P += self.Q

    def update(self, measurement):
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
        return self.x


kf = KalmanFilter(
    initial_state = 0,  # Start at orientation 0
    initial_covariance = 0,  # Initial uncertainty
    process_noise = 1.5,  # Process noise covariance
    measurement_noise = 2 # Measurement noise covariance
)

class Point:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y
    
    def update(self, new_value, coordinate):
        if coordinate == 'x':
            self.x = new_value
        elif coordinate == 'y':
            self.y = new_value

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.pid_controller = PIDController(0.15, 0.0, 0.0)
        self.last_time = 0
        self.previous_out_left = 0
        self.previous_out_right = 0
        self.previous_orientation = 0
        self.previous_orientation_estimation = 0
        self.flag = False
        self.objective = Point()
        self.pos_estimate = Point()
        self.previous_pos = Point()
        self.is_idle = True
        self.order = ""

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

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
    
    def getSpeeds(self, goal, position, orientation, order, velocity, dt):
        dx = goal.x - position.x
        dy = goal.y - position.y
        
        if (order == "forward") | (order == "backward"):
            if int(min((0, 9), key=lambda x:abs(x-(round(abs(orientation)/10))))) == 0:
                distance_to_goal = dx
                error = -dy # Compatibility issues - dx positive means turn left, while dy positive means turn right
            else:
                distance_to_goal = dy
                error = dx

            pid_value = abs(self.pid_controller.compute(error,dt))

            if distance_to_goal > 0.3:
                if error > 0.1:
                    speed_left = velocity
                    speed_right = max(0, velocity - pid_value)
                elif error < -0.1:
                    speed_left = max(0, velocity - pid_value)
                    speed_right = velocity
                else:
                    speed_left = velocity
                    speed_right = velocity
                
                return False, speed_left, speed_right
            
            elif distance_to_goal < -0.3:

                if error > 0.1:
                    speed_left = -velocity
                    speed_right = -max(0, velocity - pid_value)
                elif error < -0.1:
                    speed_left = -max(0, velocity - pid_value)
                    speed_right = -velocity
                else:
                    speed_left = -velocity
                    speed_right = -velocity
                
                return False, speed_left, speed_right
            
            else:
                return True, 0, 0
        
        elif (order == "turn left") | (order == "turn right"):
            velocity = velocity / 2
            if order == "turn left":
                if abs(90 - orientation) > 7:
                    return False, -velocity, velocity
                else:
                    return True, 0, 0
            else:   
                if abs(orientation) > 7:
                    return False, velocity, -velocity
                else:
                    return True, 0, 0
        else:
            return True, 0, 0

    def wander(self):
        front_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        base_velocity = 0.15

        # -------------------
        # Measurments
        # -------------------

        current_time = self.measures.time
        dt = current_time - self.last_time
        self.last_time = current_time

        orientation = self.measures.compass
        kf.update(orientation)
        
        filtered_orientation = kf.get_state()


        sensor_filter.add_value('center', self.measures.irSensor[front_id])
        sensor_filter.add_value('left', self.measures.irSensor[left_id])
        sensor_filter.add_value('right', self.measures.irSensor[right_id])
        sensor_filter.add_value('back', self.measures.irSensor[back_id])

        front_proximity = abs(sensor_filter.get_filtered_value('center'))
        left_proximity = abs(sensor_filter.get_filtered_value('left'))
        right_proximity = abs(sensor_filter.get_filtered_value('right'))
        back_proximity = abs(sensor_filter.get_filtered_value('back'))

        try :
            front_distance = 1 / front_proximity
        except:
            front_distance = 20
        try :
            left_distance = 1 / left_proximity
        except:
            left_distance = 20
        try :
            right_distance = 1 / right_proximity
        except:
            right_distance = 20
        try :
            back_distance = 1 / back_proximity
        except:
            back_distance = 20

        # -------------------
        # Wander Logic
        # -------------------

        dx = self.objective.x - self.pos_estimate.x
        dy = self.objective.y - self.pos_estimate.y
        print('dx: ' + str(dx) + ' dy: ' + str(dy))
        if ((abs(dx) < 0.3) & (abs(dy) < 0.3)):
            
            speed_left = 0
            speed_right = 0
            
            if self.flag:
                min_x = -2
                max_x = 24
                lower = max(min_x, int(self.pos_estimate.x - 5))
                upper = min(max_x, int(self.pos_estimate.x + 5))
                self.objective.update(randrange(lower, upper),'x')
                self.flag = False
            else:
                min_y = -10
                max_y = 2
                lower = max(min_y, int(self.pos_estimate.y - 5))
                upper = min(max_y, int(self.pos_estimate.y + 5))
                self.objective.update(randrange(lower, upper),'y')
                self.flag = True
        
        print(self.is_idle)
        print(self.order)
        if self.is_idle:
            if ((abs(dx) > 0.3) & (abs(dy) < 0.3)):
                if abs(filtered_orientation) < 7:
                    if dx > 0:
                        self.order = "forward"
                        # speed_left = base_velocity
                        # speed_right = base_velocity
                
                    elif dx < 0:
                        self.order = "backward"
                        # speed_left = -base_velocity
                        # speed_right = -base_velocity
                else:
                    self.order = "turn right"
                    # speed_left = base_velocity
                    # speed_right = -base_velocity
            
            elif ((abs(dx) < 0.3) & (abs(dy) > 0.3)):
                if abs(filtered_orientation - 90) < 7:
                    if dy > 0:
                        self.order = "forward"
                        # speed_left = base_velocity
                        # speed_right = base_velocity
                
                    elif dy < 0:
                        self.order = "backward"
                        # speed_left = -base_velocity
                        # speed_right = -base_velocity
                else:
                    self.order = "turn left"
                    # speed_left = -base_velocity
                    # speed_right = base_velocity
            else:
                self.order = "stop"

        self.is_idle, speed_left, speed_right = self.getSpeeds(self.objective, self.pos_estimate, filtered_orientation, self.order, base_velocity,dt)
        
        # -------------------
        # Actuation
        # -------------------

        self.driveMotors(speed_left, speed_right)
        
        out_left = (speed_left + self.previous_out_left) / 2 # diameters per second
        out_right = (speed_right + self.previous_out_right) / 2 # diameters per second

        lin_speed = (out_left + out_right) / 2 # diameters per second

        rot_speed = out_right - out_left # radians per second

        # -------------------
        # Prediction and Estimation
        # -------------------

        self.pos_estimate.update(self.previous_pos.x + lin_speed * cos(radians(self.previous_orientation)),'x')
        self.pos_estimate.update(self.previous_pos.y + lin_speed * sin(radians(self.previous_orientation)),'y')

        orientation_estimation = degrees(radians(self.previous_orientation_estimation) + rot_speed) # degrees

        kf.predict(degrees(rot_speed), dt)

        # -------------------
        # Visualization
        # -------------------
        print('X: ' + str(self.measures.x - 843.1) + ' X_est: ' + str(round(self.pos_estimate.x,1)))
        print('Y: ' + str(self.measures.y - 405.4) + ' Y_est: ' + str(round(self.pos_estimate.y,1)))
        # print(self.measures.beacon)
        print('F: ' + str(round(front_distance, 1)) + ' B: ' + str(round(back_distance,1)) + ' L: ' + str(round(left_distance,1)) + ' R: ' + str(round(right_distance,1)))
        print('Orientation: ' + str(self.measures.compass) + ' Orienataion_est: ' + str(round(self.previous_orientation_estimation)) + ' Orienataion_fil: ' + str(round(filtered_orientation)))
        # print(current_time)
        # The values predicted by the filter and movement model, only take effect in the compass next cycle.
        # This means, prediction and estimation should be done after actuation

        # -------------------
        # Update
        # -------------------
        self.previous_out_left = out_left
        self.previous_out_right = out_right

        self.previous_pos.update(self.pos_estimate.x, 'x')
        self.previous_pos.update(self.pos_estimate.y, 'y')

        self.previous_orientation = filtered_orientation
        self.previous_orientation_estimation = orientation_estimation

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

import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import time

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

        # current_time = time.time()
        # dt = current_time - self.last_time
        # self.last_time = current_time

        # if dt <= 0:
        #     dt = 0.01

        front_proximity = self.measures.irSensor[front_id]
        left_proximity = self.measures.irSensor[left_id]
        right_proximity = self.measures.irSensor[right_id]
        back_proximity = self.measures.irSensor[back_id]

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
            back_right_distance = 20

        # print('FL: '+str(wide_left_distance)+' FR: '+str(wide_right_distance)+' BL: '+str(middle_left_distance)+' BR: '+str(middle_right_distance))

        # error1 = wide_right_proximity - wide_left_proximity
        # error2 = middle_right_proximity - middle_left_proximity

        # pid_output1 = self.pid_controller.compute(error1, dt)
        # pid_output2 = self.pid_controller.compute(error2, dt)

        # pid_output = (pid_output1+pid_output2)/2

        # if (wide_left_distance < 0.6) & (wide_right_distance < 0.6):# & (middle_left_distance < 0.8) & (middle_right_distance < 0.8):
        #     if (error1 > 0.75) | (error2 > 0.75):
        #         print('Smooth left')
        #         self.driveMotors(max(base_velocity - pid_output,-0),+base_velocity)
        #     elif (error1 < -0.75) | (error2 < -0.75):
        #         print ('Smooth right')
        #         self.driveMotors(+base_velocity,max(base_velocity+pid_output,-0))
        #     else:
        #         print('Go')
        #         self.driveMotors(base_velocity,base_velocity)
        # elif (wide_left_distance > 1.0) & (wide_right_distance > 1.0) & (middle_left_distance > 1.0) & (middle_right_distance > 1.0):
        #     print('Crossway')
        #     self.driveMotors(base_velocity,base_velocity)
        # else:
        #     if error1 > 0:
        #         print('Sharp left')
        #         self.driveMotors(max(base_velocity - pid_output,-base_velocity),+base_velocity)
        #     elif error1 < 0:
        #         print ('Sharp right')
        #         self.driveMotors(+base_velocity,max(base_velocity+pid_output,-base_velocity))
        orientation_rad = self.measures.compass * pi / 180
        
        if front_distance < 1:
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

        out_left = (speed_left + self.previous_out_left) / 2
        out_right = (speed_right + self.previous_out_right) / 2

        lin_speed = (out_left + out_right) / 2
        x_estimate = self.previous_x + lin_speed * cos(self.previous_orientation)
        y_estimate = self.previous_y + lin_speed * sin(self.previous_orientation)

        rot_speed = (out_right - out_left)
        orientation_estimation = self.previous_orientation + rot_speed

        self.previous_out_left = out_left
        self.previous_out_right = out_right
        self.previous_orientation = orientation_estimation
        self.previous_x = x_estimate
        self.previous_y = y_estimate

        print('X: ' + str(x_estimate) + ' Y: ' + str(y_estimate) + ' Ori: ' + str(orientation_rad) + ' Ori_est: ' + str(orientation_estimation))
        print(self.measures.beacon)

        
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

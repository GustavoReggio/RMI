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
        back_left_id = 0
        front_left_id = 1
        front_right_id = 2
        back_right_id = 3
        base_velocity = 0.15
        danger_walls = 1.83
        danger_front = 0.85

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 0.01

        front_left_proximity = self.measures.irSensor[front_left_id]
        back_left_proximity = self.measures.irSensor[back_left_id]
        front_right_proximity = self.measures.irSensor[front_right_id]
        back_right_proximity = self.measures.irSensor[back_right_id]

        try :
            front_left_distance = 1 / front_left_proximity
        except:
            front_left_distance = 20
        try :
            front_right_distance = 1 / front_right_proximity
        except:
            front_right_distance = 20
        try :
            back_left_distance = 1 / back_left_proximity
        except:
            back_left_distance = 20
        try :
            back_right_distance = 1 / back_right_proximity
        except:
            back_right_distance = 20

        print('FL: '+str(front_left_distance)+' FR: '+str(front_right_distance)+' BL: '+str(back_left_distance)+' BR: '+str(back_right_distance))

        error1 = front_right_proximity - front_left_proximity
        error2 = back_right_proximity - back_left_proximity

        pid_output1 = self.pid_controller.compute(error1, dt)
        pid_output2 = self.pid_controller.compute(error2, dt)

        pid_output = (pid_output1+pid_output2)/2

        if (front_left_distance < 0.6) & (front_right_distance < 0.6):# & (back_left_distance < 0.8) & (back_right_distance < 0.8):
            if (error1 > 0.75) | (error2 > 0.75):
                print('Smooth left')
                self.driveMotors(max(base_velocity - pid_output,-0),+base_velocity)
            elif (error1 < -0.75) | (error2 < -0.75):
                print ('Smooth right')
                self.driveMotors(+base_velocity,max(base_velocity+pid_output,-0))
            else:
                print('Go')
                self.driveMotors(base_velocity,base_velocity)
        elif (front_left_distance > 1.0) & (front_right_distance > 1.0) & (back_left_distance > 1.0) & (back_right_distance > 1.0):
            print('Crossway')
            self.driveMotors(base_velocity,base_velocity)
        else:
            if error1 > 0:
                print('Sharp left')
                self.driveMotors(max(base_velocity - pid_output,-base_velocity),+base_velocity)
            elif error1 < 0:
                print ('Sharp right')
                self.driveMotors(+base_velocity,max(base_velocity+pid_output,-base_velocity))

        
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
    rob=MyRob(rob_name,pos,[30.0,60.0,-60.0,-30.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

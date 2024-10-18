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
        self.first_loop = True
        self.map_position = True
        self.discovered_map = [[' ' for _ in range(55)] for _ in range(27)]
        self.x_zero = 28 - 1
        self.y_zero = 14 - 1
        self.discovered_map[self.y_zero][self.x_zero] = 'I'
        self.base_velocity = 0.1

        self.pid_controller = PIDController(0.25, 0.0, 0.0)
        self.last_time = time.time()


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def mapLocation(self, x_position, y_position, direction):
        center_id = 0
        left_id = 1
        back_id = 2
        right_id = 3

        base_velocity = 0.15
        danger_walls = 1.8
        danger_front = 0.95

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 0.01

        center_proximity = self.measures.irSensor[center_id]
        left_proximity = self.measures.irSensor[left_id]
        back_proximity = self.measures.irSensor[back_id]
        right_proximity = self.measures.irSensor[right_id]

        try :
            center_distance = 1 / center_proximity
        except:
            center_distance = 20
        try :
            left_distance = 1 / left_proximity
        except:
            left_distance = 20
        try :
            back_distance = 1 / back_proximity
        except:
            back_distance = 20
        try :
            right_distance = 1 / right_proximity
        except:
            right_distance = 20
        
        center_distance = ceil(center_distance)
        left_distance = ceil(left_distance)
        back_distance = ceil(back_distance)
        right_distance = ceil(right_distance)

        print('Center: '+str(center_distance)+' Left: '+str(left_distance)+' Back: '+str(back_distance)+' Right: '+str(right_distance))



        error = right_distance - left_distance
        

        pid_output = self.pid_controller.compute(error, dt)

        if (left_proximity < danger_walls) & (right_proximity < danger_walls) & (center_distance < danger_front): # Crossways logic
            #print('Crossway')
            self.situation = 'Crossway'
            self.driveMotors(base_velocity,base_velocity)
        elif center_distance > danger_front:
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

    #------------------------------#------------------------------#

   
        if direction == 0:
            if center_distance < 1:
                self.discovered_map[y_position + self.y_zero][x_position + self.x_zero + 2] = '|'
            else:
                self.discovered_map[y_position + self.y_zero][x_position + 28 + 1 - 1] = 'X'
                self.discovered_map[y_position + self.y_zero][x_position + 28 + 2 - 1] = 'X'
            if right_distance < 1:
                self.discovered_map[y_position + self.y_zero + 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + self.y_zero + 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + self.y_zero + 2][x_position + 28 - 1] = 'X'
            if back_distance < 1:
                self.discovered_map[y_position + self.y_zero][x_position + 28 - 2 - 1] = '|'
            else:
                self.discovered_map[y_position + self.y_zero][x_position + 28 - 1 - 1] = 'X'
                self.discovered_map[y_position + self.y_zero][x_position + 28 - 2 - 1] = 'X'
            if left_distance < 1:
                self.discovered_map[y_position + self.y_zero - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + self.y_zero - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + self.y_zero - 2][x_position + 28 - 1] = 'X'
        elif direction == 90:
            if right_distance < 1:
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] = '|'
            else:
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 1 - 1] = 'X'
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] = 'X'
            if back_distance < 1:
                self.discovered_map[y_position + 14 + 1 - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + 14 + 1 - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + 14 + 2 - 1][x_position + 28 - 1] = 'X'
            if left_distance < 1:
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] = '|'
            else:
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 1 - 1] = 'X'
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] = 'X'
            if center_distance < 1:
                self.discovered_map[y_position + 14 - 1 - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + 14 - 1 - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + 14 - 2 - 1][x_position + 28 - 1] = 'X'
        elif (direction == 180) | (direction == -180):
            if back_distance < 1:
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] = '|'
            else:
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 1 - 1] = 'X'
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] = 'X'
            if left_distance < 1:
                self.discovered_map[y_position + 14 + 1 - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + 14 + 1 - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + 14 + 2 - 1][x_position + 28 - 1] = 'X'
            if center_distance < 1:
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] = '|'
            else:
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 1 - 1] = 'X'
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] = 'X'
            if right_distance < 1:
                self.discovered_map[y_position + 14 - 1 - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + 14 - 1 - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + 14 - 2 - 1][x_position + 28 - 1] = 'X'
        elif direction == -90:
            if left_distance < 1:
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] = '|'
            else:
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 1 - 1] = 'X'
                self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] = 'X'
            if center_distance < 1:
                self.discovered_map[y_position + 14 + 1 - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + 14 + 1 - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + 14 + 2 - 1][x_position + 28 - 1] = 'X'
            if right_distance < 1:
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] = '|'
            else:
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 1 - 1] = 'X'
                self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] = 'X'
            if back_distance < 1:
                self.discovered_map[y_position + 14 - 1 - 1][x_position + 28 - 1] = '-'
            else:
                self.discovered_map[y_position + 14 - 1 - 1][x_position + 28 - 1] = 'X'
                self.discovered_map[y_position + 14 - 2 - 1][x_position + 28 - 1] = 'X'

        for n in range(27):
            print(self.discovered_map[n])

    def findNextLocation(self, x_position, y_position, direction):
        if direction == 0:
            if self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] == 'X':
                next_step = "FW"
            else:
                if self.discovered_map[y_position + 14 + 2 - 1][x_position + 28 - 1] == 'X':
                    next_step = "CW"
                elif self.discovered_map[y_position + 14 - 2 - 1][x_position + 28 - 1] == 'X':
                    next_step = "CCW"
                else: 
                    next_step = "BW"
        if direction == 90:
            if self.discovered_map[y_position + 14 - 2 - 1][x_position + 28 - 1] == 'X':
                next_step = "FW"
            else:
                if self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] == 'X':
                    next_step = "CW"
                elif self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] == 'X':
                    next_step = "CCW"
                else: 
                    next_step = "BW"
        if (direction == 180) | (direction == -180):
            if self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] == 'X':
                next_step = "FW"
            else:
                if self.discovered_map[y_position + 14 - 2 - 1][x_position + 28 - 1] == 'X':
                    next_step = "CW"
                elif self.discovered_map[y_position + 14 + 2 - 1][x_position + 28 - 1] == 'X':
                    next_step = "CCW"
                else: 
                    next_step = "BW"
        if direction == -90:
            if self.discovered_map[y_position + 14 + 2 - 1][x_position + 28 - 1] == 'X':
                next_step = "FW"
            else:
                if self.discovered_map[y_position + 14 - 1][x_position + 28 - 2 - 1] == 'X':
                    next_step = "CW"
                elif self.discovered_map[y_position + 14 - 1][x_position + 28 + 2 - 1] == 'X':
                    next_step = "CCW"
                else: 
                    next_step = "BW"
        return next_step

    def calibratePosition(self):
        self.x_offset = self.measures.x
        self.y_offset = self.measures.y

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
            
    def wander(self):
        if self.first_loop:
            self.calibratePosition()
            self.first_loop = False

        self.x_position = int(round(self.measures.x - self.x_offset))
        self.y_position = int(round(self.measures.y - self.y_offset))
        self.direction = int(round(self.measures.compass))

        if self.map_position:
            self.mapLocation(self.x_position,self.y_position,self.direction)
            self.map_position = False
            
            next_location = self.findNextLocation(self.x_position, self.y_position, self.direction)
            print(next_location)

        print('X: '+str(self.x_position)+' Y: '+str(self.y_position)+' Direction: '+str(self.direction))
        
        self.driveMotors(self.base_velocity, self.base_velocity)
        
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
        print(self.labMap)


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
    rob=MyRob(rob_name,pos,[0.0,-90.0,180.0,90.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
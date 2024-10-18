import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

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
        self.visit_locations = []


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def mapLocation(self):
        center_id = 0
        left_id = 1
        back_id = 2
        right_id = 3

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
        
        center_distance = int(ceil(center_distance))
        left_distance = int(ceil(left_distance))
        back_distance = int(ceil(back_distance))
        right_distance = int(ceil(right_distance))

        print('Center: '+str(center_distance)+' Left: '+str(left_distance)+' Back: '+str(back_distance)+' Right: '+str(right_distance))

        #Will we map it only looking forward, or will we map in the direction we are facing?
        if center_distance < 3:
            for n in range(center_distance):
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n + 1] = 'X'
            self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n*2 + 1] = '|'
            if n > 0:
                self.visit_locations.append((self.x_position + 2, self.y_position))
        else :
            for n in range(2):
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n + 1] = 'X'
            self.visit_locations.append((self.x_position + 2, self.y_position))

        if back_distance < 3:
            for n in range(back_distance):
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n - 1] = 'X'
            self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n*2 - 1] = '|'
            if n > 0:
                self.visit_locations.append((self.x_position - 2, self.y_position))
        else :
            for n in range(2):
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n - 1] = 'X'
            self.visit_locations.append((self.x_position - 2, self.y_position))

        if left_distance < 3:
            for n in range(left_distance):
                self.discovered_map[self.y_zero + self.y_position - n - 1][self.x_zero + self.x_position] = 'X'
            self.discovered_map[self.y_zero + self.y_position - n*2 - 1][self.x_zero + self.x_position] = '-'
            if n > 0:
                self.visit_locations.append((self.x_position, self.y_position - 2))
        else :
            for n in range(2):
                self.discovered_map[self.y_zero + self.y_position - n - 1][self.x_zero + self.x_position] = 'X'
            self.visit_locations.append((self.x_position, self.y_position - 2))

        if right_distance < 3:
            for n in range(right_distance):
                self.discovered_map[self.y_zero + self.y_position + n + 1][self.x_zero + self.x_position] = 'X'
            self.discovered_map[self.y_zero + self.y_position + n*2 + 1][self.x_zero + self.x_position] = '-'
            if n > 0:
                self.visit_locations.append((self.x_position, self.y_position + 2))

        else :
            for n in range(2):
                self.discovered_map[self.y_zero + self.y_position + n + 1][self.x_zero + self.x_position] = 'X'
            self.visit_locations.append((self.x_position, self.y_position + 2))


        for n in range(27):
            print(self.discovered_map[n])
        print(self.visit_locations)


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
            self.mapLocation()
            self.map_position = False
            
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

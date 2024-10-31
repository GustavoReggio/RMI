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

class FilterSensor:
    def __init__(self, amostragem):
        self.amostragem = amostragem
        self.sensors = {
            'left': [],
            'right': [],
            'center': [],
            'back': []
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

filter_ir_sensor = FilterSensor(amostragem=5)

class Node:
  def __init__(self, x, y, distance, parent = None):
    self.x = x
    self.y = y
    self.distance = distance  
    self.parent = parent 

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
        self.free_cells = [(0,0)]
        self.visited_locations = [(0,0)]
        self.path = []
        self.counter = 5
        self.pid_controller = PIDController(0.15, 0.0, 0.0)
        self.is_idle = True
        self.objective = None
        self.goal = 0
        self.error = 0



    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def mapLocation(self):

        center_proximity = filter_ir_sensor.get_filtered_value('center')
        left_proximity = filter_ir_sensor.get_filtered_value('left')
        right_proximity = filter_ir_sensor.get_filtered_value('right')
        back_proximity = filter_ir_sensor.get_filtered_value('back')

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
        if int(min((0, 9), key=lambda x:abs(x-(round(abs(self.direction)/10))))) == 0:
            if center_distance < 2:
                for n in range(center_distance):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n + 1] = 'X'
                    if center_distance > 1:
                        self.free_cells.append((self.x_position + n + 1, self.y_position))
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n*2 + 1] = '|'
                if n > 0:
                    self.visit_locations.append((self.x_position + 2, self.y_position))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n + 1] = 'X'
                    self.free_cells.append((self.x_position + n + 1, self.y_position))
                self.visit_locations.append((self.x_position + 2, self.y_position))

            if back_distance < 2:
                for n in range(back_distance):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n - 1] = 'X'
                    if back_distance > 1:
                        self.free_cells.append((self.x_position - n - 1, self.y_position))
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n*2 - 1] = '|'
                if n > 0:
                    self.visit_locations.append((self.x_position - 2, self.y_position))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n - 1] = 'X'
                    self.free_cells.append((self.x_position - n - 1, self.y_position))
                self.visit_locations.append((self.x_position - 2, self.y_position))

            if left_distance < 2:
                for n in range(left_distance):
                    self.discovered_map[self.y_zero + self.y_position - n - 1][self.x_zero + self.x_position] = 'X'
                    if left_distance > 1:
                        self.free_cells.append((self.x_position, self.y_position - n - 1))
                self.discovered_map[self.y_zero + self.y_position - n*2 - 1][self.x_zero + self.x_position] = '-'
                if n > 0:
                    self.visit_locations.append((self.x_position, self.y_position - 2))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position - n - 1][self.x_zero + self.x_position] = 'X'
                    self.free_cells.append((self.x_position, self.y_position - n - 1))
                self.visit_locations.append((self.x_position, self.y_position - 2))

            if right_distance < 2:
                for n in range(right_distance):
                    self.discovered_map[self.y_zero + self.y_position + n + 1][self.x_zero + self.x_position] = 'X'
                    if right_distance > 1:
                        self.free_cells.append((self.x_position, self.y_position + n + 1))
                self.discovered_map[self.y_zero + self.y_position + n*2 + 1][self.x_zero + self.x_position] = '-'
                if n > 0:
                    self.visit_locations.append((self.x_position, self.y_position + 2))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position + n + 1][self.x_zero + self.x_position] = 'X'
                    self.free_cells.append((self.x_position, self.y_position + n + 1))
                self.visit_locations.append((self.x_position, self.y_position + 2))
            self.discovered_map[self.y_zero][self.x_zero] = 'I'
        
        elif int(min((0, 9), key=lambda x:abs(x-(round(abs(self.direction)/10))))) == 9:
            if right_distance < 2:
                for n in range(right_distance):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n + 1] = 'X'
                    if right_distance > 1:
                        self.free_cells.append((self.x_position + n + 1, self.y_position))
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n*2 + 1] = '|'
                if n > 0:
                    self.visit_locations.append((self.x_position + 2, self.y_position))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position + n + 1] = 'X'
                    self.free_cells.append((self.x_position + n + 1, self.y_position))
                self.visit_locations.append((self.x_position + 2, self.y_position))

            if left_distance < 2:
                for n in range(left_distance):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n - 1] = 'X'
                    if left_distance > 1:
                        self.free_cells.append((self.x_position - n - 1, self.y_position))
                self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n*2 - 1] = '|'
                if n > 0:
                    self.visit_locations.append((self.x_position - 2, self.y_position))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position][self.x_zero + self.x_position - n - 1] = 'X'
                    self.free_cells.append((self.x_position - n - 1, self.y_position))
                self.visit_locations.append((self.x_position - 2, self.y_position))

            if center_distance < 2:
                for n in range(center_distance):
                    self.discovered_map[self.y_zero + self.y_position - n - 1][self.x_zero + self.x_position] = 'X'
                    if center_distance > 1:
                        self.free_cells.append((self.x_position, self.y_position - n - 1))
                self.discovered_map[self.y_zero + self.y_position - n*2 - 1][self.x_zero + self.x_position] = '-'
                if n > 0:
                    self.visit_locations.append((self.x_position, self.y_position - 2))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position - n - 1][self.x_zero + self.x_position] = 'X'
                    self.free_cells.append((self.x_position, self.y_position - n - 1))
                self.visit_locations.append((self.x_position, self.y_position - 2))

            if back_distance < 2:
                for n in range(back_distance):
                    self.discovered_map[self.y_zero + self.y_position + n + 1][self.x_zero + self.x_position] = 'X'
                    if back_distance > 1:
                        self.free_cells.append((self.x_position, self.y_position + n + 1))
                self.discovered_map[self.y_zero + self.y_position + n*2 + 1][self.x_zero + self.x_position] = '-'
                if n > 0:
                    self.visit_locations.append((self.x_position, self.y_position + 2))
            else :
                for n in range(2):
                    self.discovered_map[self.y_zero + self.y_position + n + 1][self.x_zero + self.x_position] = 'X'
                    self.free_cells.append((self.x_position, self.y_position + n + 1))
                self.visit_locations.append((self.x_position, self.y_position + 2))
            self.discovered_map[self.y_zero][self.x_zero] = 'I'


        set1 = set(self.visit_locations)
        set2 = set(self.visited_locations) 
        self.visit_locations = list(set1 - set2)
        print(self.visited_locations)
        for n in range(27):
            print(self.discovered_map[n])
        print("Visit Locations: " + str(self.visit_locations))
        with open("your_mapfile", 'w') as file:
                    # Write content to the file
                    for i in range(len(self.discovered_map)):
                        for ii in range(len(self.discovered_map[i])):    
                            file.write(str(self.discovered_map[i][ii]))
                        file.write("\n")

    def find_next_location(self, visit_locations, free_cells, current_position):

        visited = set()
        queue = [Node(current_position[0], current_position[1], 0)]

        while queue:
            current_node = queue.pop(0)
            x, y, distance = current_node.x, current_node.y, current_node.distance

            if (x, y) in visited:
                continue
            visited.add((x, y))

            for goal in visit_locations:
                if goal[0] == x and goal[1] == y:
                    return current_node

            for dx, dy in [(1, 0), (-1, 0), (0, -1), (0, 1)]:
                new_x = x + dx
                new_y = y + dy
                if (new_x, new_y) in free_cells:
                    new_node = Node(new_x, new_y, distance + 1, current_node)
                    queue.append(new_node)

    def moveSelf(self, objective, goal, error): 
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 0.01

        if objective:
            distance_to_goal = 0
            if goal == "dx":
                distance_to_goal = self.dx
            elif goal == "dy":
                distance_to_goal = -self.dy
            elif goal == "0":
                distance_to_goal = -self.direction
            elif goal == "90":
                distance_to_goal = 90 - self.direction
            
            error_value = 0
            if error == "dy":
                error_value = self.dy
            elif error == "dx":
                error_value = self.dx

            pid_value = abs(self.pid_controller.compute(error_value,dt))
            print(distance_to_goal)

            if (objective == "forward"):
                if distance_to_goal < 0.2:
                    self.driveMotors(0, 0)
                    return True
                elif error_value > 0.1:
                    self.driveMotors(self.base_velocity, max(0, self.base_velocity - pid_value))
                elif error_value < -0.1:
                    self.driveMotors(max(0, self.base_velocity - pid_value), self.base_velocity)
                else:
                    self.driveMotors(self.base_velocity, self.base_velocity)
                    return False
            elif (objective == "backward"):
                if distance_to_goal > -0.2:
                    self.driveMotors(0, 0)
                    return True
                elif error_value > 0.1:
                    self.driveMotors(-self.base_velocity, -max(0, self.base_velocity - pid_value))
                elif error_value < -0.1:
                    self.driveMotors(-max(0, self.base_velocity - pid_value), -self.base_velocity)
                else:
                    self.driveMotors(-self.base_velocity, -self.base_velocity)
                    return False
            elif (objective == "turn left"):
                if ((distance_to_goal < 7) & (distance_to_goal > -7)):
                    print("Stop")
                    self.driveMotors(0, 0)
                    return True
                else:
                    self.driveMotors(-self.base_velocity, self.base_velocity)
                    return False
            elif (objective == "turn right"):
                if (distance_to_goal < 7) & (distance_to_goal > -7):
                    print("Stop")
                    self.driveMotors(0, 0)
                    return True
                else:
                    self.driveMotors(self.base_velocity, -self.base_velocity)
                    return False
            elif (objective == "adjust"):
                return True
        else:
            return True

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
    
    def waitForSensor(self, default_value):
        if self.counter > 0:
            self.counter -= 1
            return False
        else:
            self.counter = default_value
            return True

            
    def wander(self):

        if self.first_loop:
            self.calibratePosition()
            self.first_loop = False

        self.x_position = int(round(self.measures.x - self.x_offset))
        self.y_position = int(round(-self.measures.y - self.y_offset))
        self.direction = int(round(self.measures.compass))
        x_float_position = self.measures.x - self.x_offset
        y_float_position = -self.measures.y - self.y_offset

        center_id = 0
        left_id = 1
        back_id = 2
        right_id = 3

        filter_ir_sensor.add_value('center', self.measures.irSensor[center_id])
        filter_ir_sensor.add_value('left', self.measures.irSensor[left_id])
        filter_ir_sensor.add_value('right', self.measures.irSensor[right_id])
        filter_ir_sensor.add_value('back', self.measures.irSensor[back_id])

        
        if self.map_position:
            if self.waitForSensor(5):
                self.path = []
                self.mapLocation()
                self.map_position = False
        
                next_location = self.find_next_location(self.visit_locations, self.free_cells, (self.x_position, self.y_position))

                if next_location:
                    while next_location:
                        self.path.append(next_location)
                        next_location = next_location.parent
                    self.path.reverse()
                else:
                    return   
        
        if len(self.visit_locations) == 0:
            if len(self.free_cells) == 1:
                return
            else:
                quit()

        if self.path:
            location = self.path[0]
            self.dx = location.x - x_float_position
            self.dy = location.y - y_float_position

            print("Next X = " + str(location.x) + " Next Y = " + str(location.y))
            print("Dx = " + str(self.dx)+ " Dy = " + str(self.dy))

            if self.is_idle:
                if ((self.dy < 0.3) & (self.dy > -0.3)) & ((self.dx > 0.3) | (self.dx < -0.3)):
                    if ((self.direction < 7) & (self.direction > -7)):
                        self.error = "dy"
                        self.goal = "dx"
                        if self.dx > 0:
                            self.objective = "forward"
                        else:
                            self.objective = "backward"
                    else:
                        self.goal = "0"
                        if self.direction > 0:
                            self.objective  = "turn right"
                        else:
                            self.objective = "turn left"
                elif ((self.dy > 0.3) | (self.dy < -0.3)) & ((self.dx < 0.3) & (self.dx > -0.3)):
                    if ((self.direction < 97) & (self.direction > 83)):
                        self.goal = "dy"
                        self.error = "dx"
                        if self.dy > 0:
                            self.objective = "backward"
                        else:
                            self.objective = "forward"
                    else:
                        self.goal = "90"
                        if self.direction > 90:
                            self.objective = "turn right"
                        else:
                            self.objective = "turn left"
                else:
                    self.driveMotors(0,0)
                    del self.path[0]


            print(str(self.objective) + " " + str(self.goal))
            self.is_idle = self.moveSelf(self.objective, self.goal, self.error)

                
        else:
            if ((self.direction < 5) & (self.direction > -5)) | ((self.direction < 95) & (self.direction > 85)):
                self.driveMotors(0,0)
                self.is_idle = True
                self.visited_locations.append((self.x_position, self.y_position))
                self.map_position = True
            else:
                self.objective = "adjust"
                self.goal = min((0, 9), key=lambda x:abs(x-(round(abs(self.direction)/10))))
                self.goal = str(int(self.goal * 10))
                print(str(self.objective) + " " + str(self.goal))
                if (self.goal == "0"):
                    if (self.direction > 0):
                        self.driveMotors(self.base_velocity, -self.base_velocity)
                    elif (self.direction < 0):
                        self.driveMotors(-self.base_velocity, self.base_velocity)
                elif (self.goal == "90"):
                    if (self.direction > 90):
                        self.driveMotors(self.base_velocity, -self.base_velocity)
                    elif (self.direction < 90):
                        self.driveMotors(-self.base_velocity, self.base_velocity)

        print('X: '+str(self.x_position)+' Y: '+str(self.y_position)+' Direction: '+str(self.direction))
                
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
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outfile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,180.0,-90.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

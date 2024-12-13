#!/usr/bin/env python3
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        try:
            derivative = (error - self.previous_error) / dt
        except:
            derivative = 0
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
            values.pop(0)
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
        self.x = initial_state
        self.P = initial_covariance

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
    
    # This functions were added for comparision operations
    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))
    
    def __ne__(self, other):
        return not self.__eq__(other)

class Node:
  def __init__(self, point, f, g, h, parent = None):
    self.point = point
    self.f = f
    self.g = g
    self.h = h
    self.parent = parent 

class OwnMap:
    def __init__(self):
        self.discovered_map = [[' ' for _ in range(55)] for _ in range(27)]
        self.offset = Point(27,13)
        self.discovered_map[self.offset.y][self.offset.x] = '0'
        self.unexplored_cells = []
        self.explored_cells = [Point()]
        self.free_cells = [Point()]
        self.vertical_walls = []
        self.horizontal_walls = []
        self.beacons = [(Point(), 0)]
        self.ignored_cells = []
    
    def updateMap(self, point, state, beacon = None, line_of_sight = None):
        if state == 'free':
            if point not in self.free_cells:
                self.free_cells.append(point)
                if ((point.x % 2 == 0) & (point.y % 2 == 0)): 
                    self.unexplored_cells.append(point)
        elif state == 'v_wall':
            if point not in self.vertical_walls:
                self.vertical_walls.append(point)
        elif state == 'h_wall':
            if point not in self.horizontal_walls:
                self.horizontal_walls.append(point)
        elif state == 'beacon':
            if (point, beacon) not in self.beacons:
                self.beacons.append((point, beacon))
                self.beacons.sort(key=lambda beacon: beacon[1])
        elif state == 'mapped':
            if point not in self.explored_cells:
                self.explored_cells.append(point)
                self.unexplored_cells = [i for i in self.unexplored_cells if i not in self.explored_cells]

            beacon_status = [status for status, _ in line_of_sight]
            beacon_status.pop(0)
            if not any(beacon_status):
                for unexplored_cell in self.unexplored_cells[:]:  # Iterate over a copy of the list
                    neighbors = [
                        Point(unexplored_cell.x + dx, unexplored_cell.y + dy)
                        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]
                    ]
                    if all(neighbor in self.vertical_walls or neighbor in self.horizontal_walls or neighbor in self.free_cells for neighbor in neighbors):
                        self.unexplored_cells.remove(unexplored_cell)
                        self.ignored_cells.append(unexplored_cell)
                        if unexplored_cell not in self.explored_cells:
                            self.explored_cells.append(unexplored_cell)
        
    def writeMap(self):
        for point in self.vertical_walls:
            self.discovered_map[self.offset.y - point.y][self.offset.x + point.x] = '|'
        for point in self.horizontal_walls:
            self.discovered_map[self.offset.y - point.y][self.offset.x + point.x] = '-'
        for point in self.free_cells:
            self.discovered_map[self.offset.y - point.y][self.offset.x + point.x] = 'X'
        for beacon in self.beacons:
            self.discovered_map[self.offset.y - beacon[0].y][self.offset.x + beacon[0].x] = str(beacon[1])

        # ----- Write map in File -----
        with open(str("map_made"+rob_name+".map"), 'w') as file:
            for row in self.discovered_map:
                file.write(''.join(map(str, row)) + '\n')

        # ----- Write path in File -----
        final_path = []
        beacon_coords = [beacon[0] for beacon in self.beacons] #Array of Points
        order = self.tsp(beacon_coords)
        for i in range(len(order) - 1):
            partial_path = []
            path = self.find_next_location([beacon_coords[order[i+1]]],beacon_coords[order[i]])
            while path:
                partial_path.append(path.point)
                path = path.parent
            final_path.append(partial_path[::-2])
        
        with open(str("path_made"+rob_name+".path"), 'w') as file:
            for i, path in enumerate(final_path):
                if i > 0:
                    path = path[1:]  # Skip the first point in subsequent paths
                file.writelines(str(p.x) + " " + str(p.y) + "\n" for p in path)

    def distance(self, p1, p2):
        distance = self.find_next_location([p2],p1)
        f_value = distance.f
        return f_value

    def tsp(self, points):
        n = len(points)
        # Distance matrix
        dist = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist[i][j] = self.distance(points[i], points[j])
        
        INF = float('inf')
        dp = [[INF] * n for _ in range(1 << n)]
        parent = [[-1] * n for _ in range(1 << n)]
        
        dp[1][0] = 0

        for mask in range(1, 1 << n):
            for i in range(n):
                if mask & (1 << i):
                    for j in range(n):
                        if not (mask & (1 << j)):
                            new_mask = mask | (1 << j)
                            new_cost = dp[mask][i] + dist[i][j]
                            if new_cost < dp[new_mask][j]:
                                dp[new_mask][j] = new_cost
                                parent[new_mask][j] = i
        min_cost = INF
        last_point = -1
        for i in range(1, n):
            cost = dp[(1 << n) - 1][i] + dist[i][0]
            if cost < min_cost:
                min_cost = cost
                last_point = i
        
        path = []
        mask = (1 << n) - 1
        while last_point != -1:
            path.append(last_point)
            next_point = parent[mask][last_point]
            mask = mask ^ (1 << last_point)
            last_point = next_point        
        path.reverse()
        path.append(path[0])
        return path
    
    def find_next_location(self, visit_locations, current_position, current_orientation = None):
        
        deltas = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        
        position_int = Point(int(round(current_position.x)), int(round(current_position.y)))
        closed_set = set()
        open_set = [Node(position_int, 0, 0, self.heuristic(position_int, visit_locations))]

        while open_set:
            current_node = open_set.pop(0)
            x, y = current_node.point.x, current_node.point.y
            
            if (x, y) in closed_set:
                continue
            closed_set.add((x, y))

            for goal in visit_locations:
                if goal.x == x and goal.y == y:
                    return current_node

            if current_orientation: # Prioritize going straight
                if int(min((0, 9), key=lambda x: abs(x - round(abs(current_orientation) / 10)))) == 9:
                    deltas = [(0, 1), (0, -1), (1, 0), (-1, 0),]

            for dx, dy in deltas:
                new_x = x + dx
                new_y = y + dy
                new_g = 0
                if Point(new_x, new_y) in self.free_cells:
                    new_g = current_node.g + 1              
                    new_h = self.heuristic(Point(new_x, new_y), visit_locations)
                    new_node = Node(Point(new_x, new_y), new_g + new_h, new_g, new_h, current_node)

                    if new_node not in open_set:
                        open_set.append(new_node)
                        open_set.sort(key=lambda node: node.f)  # Sort by f-value

        return None

    def heuristic(self, current_pos, targets):

        min_distance = float('inf')
        for target in targets:
            distance = abs(current_pos.x - target.x) + abs(current_pos.y - target.y)
            min_distance = min(min_distance, distance)
        return min_distance

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.pid_controller = PIDController(0.15, 0.0, 0.0)
        self.last_time = 0
        self.previous_out_left = 0
        self.previous_out_right = 0
        self.previous_orientation = 0
        self.previous_orientation_estimation = 0
        self.map_flag = True
        self.plan_flag = False
        self.finish_flag = False
        self.complete_flag = False
        self.objective = Point()
        self.pos_estimate = Point()
        self.previous_pos = Point()
        self.is_idle = True
        self.order = ""
        self.path = []
        self.map = OwnMap()

    def map_location(self, direction, sensor_readings, ground_readings):

        if int(min((0, 9), key=lambda x: abs(x - round(abs(direction) / 10)))) == 0:
            obj_pos_x = sensor_readings[0]
            obj_neg_x = sensor_readings[1]
            obj_pos_y = sensor_readings[2]
            obj_neg_y = sensor_readings[3]
        elif int(min((0, 9), key=lambda x: abs(x - round(abs(direction) / 10)))) == 9:
            obj_pos_x = sensor_readings[3]
            obj_neg_x = sensor_readings[2]
            obj_pos_y = sensor_readings[0]
            obj_neg_y = sensor_readings[1]

        measures = [obj_pos_x, obj_neg_x, obj_pos_y, obj_neg_y]
        deltas = [Point(1,0), Point(-1,0), Point(0,1), Point(0,-1)]

        estimated_x = int(round(self.pos_estimate.x))
        estimated_y = int(round(self.pos_estimate.y))

        n = 0
        for distance in measures:
            delta = deltas[n]
            if ceil(distance) < 2:
                if n < 2:
                    self.map.updateMap(Point(estimated_x + delta.x, estimated_y + delta.y), 'v_wall')
                else:
                    self.map.updateMap(Point(estimated_x + delta.x, estimated_y + delta.y), 'h_wall')
            else:
                self.map.updateMap(Point(estimated_x + delta.x, estimated_y + delta.y), 'free')
                self.map.updateMap(Point(estimated_x + (2*delta.x), estimated_y + (2*delta.y)), 'free')
                        
            n += 1

        if ground_readings > -1:
            self.map.updateMap(Point(estimated_x,estimated_y), 'beacon', ground_readings)
        
        self.map.updateMap(Point(estimated_x, estimated_y), 'mapped', line_of_sight= self.measures.beacon)

    def update_estimation(self, measures, direction):
        if int(min((0, 9), key=lambda x: abs(x - round(abs(direction) / 10)))) == 0:
            obj_pos_x = measures[0]
            obj_neg_x = measures[1]
            obj_pos_y = measures[2]
            obj_neg_y = measures[3]
        elif int(min((0, 9), key=lambda x: abs(x - round(abs(direction) / 10)))) == 9:
            obj_pos_x = measures[3]
            obj_neg_x = measures[2]
            obj_pos_y = measures[0]
            obj_neg_y = measures[1]

        measures = [obj_pos_x, obj_neg_x, obj_pos_y, obj_neg_y]
        directions = [Point(1,0), Point(-1,0), Point(0,1), Point(0,-1)]

        position_corrected_x = [self.pos_estimate.x, 0, 0]
        position_corrected_y = [self.pos_estimate.y, 0, 0]
        n = 0
        for measure in measures:
            direction = directions[n]
            if measure < 1:
                if n < 2:
                    distance_measured = (measure + 0.5 + 0.1) * direction.x
                    wall_pos = self.pos_estimate.x + distance_measured
                    wall_pos_aprox = [ceil(wall_pos), floor(wall_pos)]

                    if (wall_pos_aprox[1] % 2) == 0:
                        position_corrected_x[n+1] = wall_pos_aprox[0] - distance_measured
                    else:
                        position_corrected_x[n+1] = wall_pos_aprox[1] - distance_measured
                        
                else:
                    distance_measured = (measure + 0.5) * direction.y
                    wall_pos = self.pos_estimate.y + distance_measured + (0.1 * direction.x)
                    wall_pos_aprox = [ceil(wall_pos), floor(wall_pos)]

                    if (wall_pos_aprox[1] % 2) == 0:
                        position_corrected_y[n-1] = wall_pos_aprox[0] - distance_measured
                    else:
                        position_corrected_y[n-1] = wall_pos_aprox[1] - distance_measured
                        
            else:
                if n < 2:
                    position_corrected_x[n+1] = self.pos_estimate.x
                else:
                    position_corrected_y[n-1] = self.pos_estimate.y
            n += 1
        self.pos_estimate.update((sum(position_corrected_x)/3), 'x')
        self.pos_estimate.update((sum(position_corrected_y)/3), 'y')
        self.previous_pos.update(self.pos_estimate.x, 'x')
        self.previous_pos.update(self.pos_estimate.y, 'y')

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
                error = -dy # Compatibility issues -> dx positive means turn left, while dy positive means turn right
            else:
                distance_to_goal = dy
                error = dx

            pid_value = abs(self.pid_controller.compute(error,dt))

            if distance_to_goal > 0.2:
                if error > 0:
                    speed_left = velocity
                    speed_right = max(0, velocity - pid_value)
                elif error < 0:
                    speed_left = max(0, velocity - pid_value)
                    speed_right = velocity
                else:
                    speed_left = velocity
                    speed_right = velocity
                
                return False, speed_left, speed_right
            
            elif distance_to_goal < -0.2:
                if error > 0:
                    speed_left = -velocity
                    speed_right = -max(0, velocity - pid_value)
                elif error < 0:
                    speed_left = -max(0, velocity - pid_value)
                    speed_right = -velocity
                else:
                    speed_left = -velocity
                    speed_right = -velocity
                
                return False, speed_left, speed_right
            
            else:
                return True, 0, 0
        
        elif (order == "turn left") | (order == "turn right"):
            if order == "turn left":
                goal_orientation = 90
            else:
                goal_orientation = 0

            distance_to_goal = goal_orientation - orientation
            
            if distance_to_goal >= 7:
                return False, -velocity, velocity
            elif distance_to_goal <= - 7:
                return False, velocity, -velocity
            else:
                return True, 0, 0
        
        elif (order == "adjust"):
            if ((abs(orientation) < 5) | (abs(orientation-90) < 5)):
                self.order = "stop"  
                if (len(self.path) != 0):
                    del self.path[0]
                    self.update_flag = True

                return True, 0, 0
            else:
                goal = min((0, 9), key=lambda x:abs(x-(round(abs(orientation)/10))))
                if (orientation > (goal * 10)):
                    speed_left = velocity / 2
                    speed_right = -velocity / 2
                elif (orientation < (goal * 10)):
                    speed_left = -velocity / 2
                    speed_right = velocity / 2
                return False, speed_left, speed_right
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

        if ((current_time == (int(self.simTime) - 5)) & (not self.complete_flag)): # early ending without finishing
            self.map.writeMap()
            self.finish()

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

        sensor_readings = [front_distance, back_distance, left_distance, right_distance]

        ground_reading = self.measures.ground
        
        # -------------------
        # Mapping
        # -------------------
        
        if self.map_flag:
            self.map_location(filtered_orientation, sensor_readings, ground_reading)
            self.map_flag = False
            self.plan_flag = True

        # -------------------
        # Path Planning
        # -------------------
        if self.plan_flag:
            if self.map.unexplored_cells:
                next_location = self.map.find_next_location(self.map.unexplored_cells , self.pos_estimate, filtered_orientation)
            elif ((len(self.map.unexplored_cells) == 0) & (self.finish_flag != True)):
                next_location = self.map.find_next_location([Point()], self.pos_estimate, filtered_orientation)
                self.finish_flag = True

            if next_location:
                while next_location:
                    self.path.append(next_location)
                    next_location = next_location.parent
                self.path.reverse()
                self.plan_flag = False

        if self.path:
            self.objective.update(self.path[0].point.x, 'x')
            self.objective.update(self.path[0].point.y, 'y')

            dx = self.path[0].point.x - self.pos_estimate.x
            dy = self.path[0].point.y - self.pos_estimate.y

        elif not self.finish_flag:
            self.update_estimation(sensor_readings, filtered_orientation)
            self.map_flag = True        

        else:
            self.map.unexplored_cells = []
            self.map.writeMap()
            self.finish()
            print("Robot: " + rob_name + " - Time taken: " + str(current_time) + " - Collisions: " + str(self.measures.collisions))
            self.complete_flag = True        

        # -------------------
        # Wander Logic
        # -------------------
        
        if self.is_idle & (len(self.path) != 0):
            if ((max(abs(dx), abs(dy)) ==  abs(dx)) & (abs(dx) > 0.2)):
                if abs(filtered_orientation) < 7:
                    if dx > 0:
                        self.order = "forward"                
                    elif dx < 0:
                        self.order = "backward"
                else:
                    self.order = "turn right"            
            elif ((max(abs(dx), abs(dy)) ==  abs(dy)) & (abs(dy) > 0.2)):
                if abs(filtered_orientation - 90) < 7:
                    if dy > 0:
                        self.order = "forward"                
                    elif dy < 0:
                        self.order = "backward"
                else:
                    self.order = "turn left"
            else:
                self.order = "adjust"
        
        self.is_idle, speed_left, speed_right = self.getSpeeds(self.objective, self.pos_estimate, filtered_orientation, self.order, base_velocity, dt)

        # -------------------
        # Actuation
        # -------------------

        self.driveMotors(speed_left, speed_right)
        
        out_left = (speed_left + self.previous_out_left) / 2
        out_right = (speed_right + self.previous_out_right) / 2 

        lin_speed = (out_left + out_right) / 2

        rot_speed = out_right - out_left

        # -------------------
        # Prediction and Estimation
        # -------------------

        self.pos_estimate.update(self.previous_pos.x + lin_speed * cos(radians(self.previous_orientation)),'x')
        self.pos_estimate.update(self.previous_pos.y + lin_speed * sin(radians(self.previous_orientation)),'y')

        orientation_estimation = degrees(radians(self.previous_orientation_estimation) + rot_speed)

        kf.predict(degrees(rot_speed), dt)

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
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outfile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

from dataclasses import dataclass
import numpy as np
from typing import Dict, Tuple
import math
from mappings import LOCATION_TO_INT, INT_TO_LOCATION

@dataclass
class PackageState:
    package_idx: int = -1
    shipping_type: str = "standard"
    current_location: str = "A"
    next_location: str = "B"
    lifecycle_state: str = "SPAWNED"
    availability: bool = False

STATION_COORDS: Dict[str, Tuple[float, float]] = {
    "A": (5.0, 15.0),
    "B": (9.0, 15.0),
    "C": (13.0, 15.0),
    "D": (17.0, 15.0),
    "E": (13.0, 1.0),
    "F": (9.0, 1.0),
    "G": (5.0, 1.0),
}
START_COORDS: Tuple[float, float] = (1.0, 13.0)

STATIONS = ["A", "B", "C", "D", "E", "F", "G"]

class Simulator:
    def __init__(self):

        # global states
        self._observers = [self.callback_spawner, self.callback_battery_decrease, self.callback_stations] # callback functoin list

        # package states
        self.num_packages = 20
        self.spawn_freq = 20 # evry 20 seconds

        self.seed = 0

        self.reset()

    def register_callback(self, func):
        self._observers.append(func)

    @property
    def time(self):
        return self._time
    
    @time.setter
    def time(self, value):
        self._time = value
        if hasattr(self, '_observers'):
            for callback in self._observers:
                callback(value)

    def callback_spawner(self, time):
        for i in range(self.num_packages):
            if (self.spawn_tracker[i] == False) and (time >= self.spawn_events[i]):
                self.packages[i].lifecycle_state= "READY"
                self.packages[i].availability = True
                self.spawn_tracker[i] = True
                self.output_queues[0].append(i)



    def callback_battery_decrease(self, time):
        dt = time - self.last_time_battery
        if self.charge_flag == False:
            self.robot_battery -=dt
        self.last_time_battery = time

    def callback_stations(self, time):
        for s_idx in range(7):
            # check processing start
            while self.process_time_start[s_idx] and time >= self.process_time_start[s_idx][0]:
                # pick packet from input for processing
                p_idx = self.input_queues[s_idx].pop(0)
                self.processing_units[s_idx].append(p_idx)
                
                # remove time stamp
                self.process_time_start[s_idx].pop(0)
                
                # status update
                self.packages[p_idx].lifecycle_state = "PROCESSING"
                self.packages[p_idx].availability = False
                print(f"[{time:.1f}s] Station {STATIONS[s_idx]}: Packet {p_idx} starts processing.")

            # check if processing finished
            while self.process_time_end[s_idx] and time >= self.process_time_end[s_idx][0]:
                # put packat to output
                p_idx = self.processing_units[s_idx].pop(0)
                
                # remove time stamp
                self.process_time_end[s_idx].pop(0)
                

                # select next station
                if s_idx == 0: # A
                    self.packages[p_idx].next_location = "B"
                    self.output_queues[s_idx].append(p_idx)
                    # status update
                    self.packages[p_idx].lifecycle_state = "READY"
                    self.packages[p_idx].availability = True

                elif s_idx == 1: # B
                    self.output_queues[s_idx].append(p_idx)
                    self.packages[p_idx].lifecycle_state = "READY"
                    self.packages[p_idx].availability = True
                    success = (self.rng.random() < 0.80)
                    if success:
                        self.packages[p_idx].next_location = "C"
                    else:
                        self.packages[p_idx].next_location = "G"

                elif s_idx == 2: # C
                    success = (self.rng.random() < 0.95)
                    if success:
                        self.output_queues[s_idx].append(p_idx)
                        self.packages[p_idx].lifecycle_state = "READY"
                        self.packages[p_idx].availability = True
                        if self.packages[p_idx].shipping_type == "standard":
                            self.packages[p_idx].next_location = "D"
                        else:
                            self.packages[p_idx].next_location = "E"
                    else:
                        # put back to input queue
                        self.packages[p_idx].lifecycle_state = "WAITING"
                        self.packages[p_idx].availability = False
                        dt = 1.0 * self.rng.random() + 1.5 # U(1.5,2.5)
                            # check if input_queue is empty
                        if not self.input_queues[s_idx]:
                            start_time = self.time # start immediately

                        # determine start
                        else:
                            start_time = self.process_time_end[s_idx][-1]
                        self.input_queues[s_idx].append(p_idx)
                        self.process_time_start[s_idx].append(start_time)
                        self.process_time_end[s_idx].append(start_time + dt)

                elif s_idx == 3: # D
                    success = (self.rng.random() < 0.85)
                    if success:
                        self.packages[p_idx].next_location = "FINISH"
                        self.packages[p_idx].lifecycle_state = "FINISHED"
                        self.packages[p_idx].availability = False
                    else:
                        self.packages[p_idx].next_location = "E"
                        self.output_queues[s_idx].append(p_idx)
                        self.packages[p_idx].lifecycle_state = "FAILED"
                        self.packages[p_idx].availability = True

                elif s_idx == 4: # E
                    if self.packages[p_idx].lifecycle_state == "FAILED":
                        self.packages[p_idx].next_location = "FINISH"
                        self.packages[p_idx].availability = False
                    else:
                        success = (self.rng.random() < 0.85)
                        if success:
                            self.packages[p_idx].next_location = "FINISH"
                            self.packages[p_idx].lifecycle_state = "FINISHED"
                            self.packages[p_idx].availability = False
                        else:
                            self.output_queues[s_idx].append(p_idx)
                            self.packages[p_idx].lifecycle_state = "FAILED"
                            self.packages[p_idx].availability = True
                            self.packages[p_idx].next_location = "FINISH"

                elif s_idx == 6: # G
                    self.packages[p_idx].next_location = "C"
                    self.output_queues[s_idx].append(p_idx)
                    # status update
                    self.packages[p_idx].lifecycle_state = "READY"
                    self.packages[p_idx].availability = True                    
                    
                print(f"[{time:.1f}s] Station {STATIONS[s_idx]}: Packet {p_idx} is finished.")

    def cmd(self, cmd, param):
        if cmd == "WAIT":
            self.time += self.wait_time
            result = True
        elif cmd == "CHARGE":
            self.charge_flag = True
            # calculate time until battery ist full
            dt = (self.robot_battery_max - self.robot_battery) / self.charge_rate
            self.robot_battery = 100
            self.time+=dt
            self.charge_flag = False
            result = True

        elif cmd == "MOVE_TO":
            # param: 0..6 -> A..G
            location_idx = LOCATION_TO_INT[self.robot_location]
            idx = int(param)
            idx = max(0, min(idx, 6))
            self.robot_location = STATIONS[idx]
            self.time += self.move_to_times[location_idx][param]
            result = True

        elif cmd == "PICK":
            # check if robot is idle and package is available
            if self.robot_carrying != -1 or self.packages[param].availability != True:
                print("pick is not vaild")
                result = False
            else:
                # check if package is available at the current station
                if (self.robot_location == self.packages[param].current_location):
                    result = True
                    station_idx = LOCATION_TO_INT[self.robot_location] -1
                    self.robot_carrying = param
                    self.packages[param].current_location = "ROBOT"
                    self.packages[param].availability = False
                    self.output_queues[station_idx].remove(param)
                    self.time += 0.1
                else:
                    print("pick is not vaild")
                    result = False
            
        elif cmd == "DROP":
            # check robot is at a staion an is carrying the package to drop
            if (self.robot_location in STATIONS) and self.robot_carrying == param:
                result = True
                station_idx = LOCATION_TO_INT[self.robot_location] -1
                self.robot_carrying = -1
                self.packages[param].current_location = self.robot_location
                self.packages[param].availability = False
                if self.robot_location != "E": # do not overwrite FAILED
                    self.packages[param].lifecycle_state = "WAITING"
                

                # calculate time for processes
                if self.robot_location == "B":
                    dt = 1.0 * self.rng.random() + 1.5 # U(1.5,2.5)
                elif self.robot_location == "C":
                    dt = 1.0 * self.rng.random() + 1.5 # U(1.5,2.5)
                elif self.robot_location == "D":
                    dt = 1.0 * self.rng.random() + 1.0 # U(1.0,2.0)
                elif self.robot_location == "E":
                    if self.packages[param].lifecycle_state == "FAILED":
                        dt = 0
                    else:
                        dt = 1.0 * self.rng.random() + 1.0 # U(1.0,2.0)
                elif self.robot_location == "G":
                    dt = 2.0 * self.rng.random() + 2.0 # U(2.0,4.0)
                else:
                    print("drop is not vaild")

                # check if input_queue is empty
                if not self.input_queues[station_idx]:
                    start_time = self.time # start immediately

                # determine start
                else:
                    start_time = self.process_time_end[station_idx][-1]
                self.input_queues[station_idx].append(param)
                self.process_time_start[station_idx].append(start_time)
                self.process_time_end[station_idx].append(start_time + dt)
                self.time += 0.1

            else:
                print("drop is not vaild")
                result = False

        elif cmd == "PICK_A":
            if (self.robot_location != "A") and (self.packages[param].current_location != "A") and (self.packages[param].availability == False):
                result = False
                print("pick at A not valid")
            else:
                attempt = 0
                dt = 0
                station_idx = LOCATION_TO_INT[self.robot_location] -1
                while True:
                    attempt += 1
                    pick_time = 1.0 * self.rng.random() + 1.0 # U(1.0,2.0)
                    dt += pick_time
                    success = (self.rng.random() < 0.95)
                    if success:
                        break
                self.output_queues[station_idx].remove(param)
                self.packages[param].current_location = "ROBOT"
                self.packages[param].next_location = "B"
                self.packages[param].lifecycle_state = "READY"
                self.packages[param].availability = False
                self.robot_carrying = param
                self.time += dt
                result = True

        return result

    def get_dist_times(self, p1, p2):
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) / self.robot_max_speed

    def reset(self):

        # robot states
        self.robot_battery = 100.0
        self.robot_battery_max = 100.0
        self.charge_rate = 10 # 10 per seconds
        self.robot_location = "ON_TRANSIT"
        self.robot_carrying = -1
        self.charge_flag= False
        self.last_time_battery = 0.0
        self.robot_max_speed = 2.0 # 2 m/s

        # package states
        self.packages = []
        self.spawn_events = np.arange(self.num_packages) * self.spawn_freq
        self.spawn_tracker =  np.full((20), False)

        # init package dictonariy
        for i in range(self.num_packages):
            s_type = "express" if i % 2 == 1 else "standard"
            p = PackageState(package_idx=i, shipping_type=s_type)
            self.packages.append(p)

        # cmd params
        self.wait_time = 1.0 # 1 second wait time 

        self.move_to_times = [] # adapt if A* is finished
        starts = [START_COORDS] + [STATION_COORDS[s] for s in STATIONS]
        for start_pos in starts:
            row = []
            for target in STATIONS:
                target_pos = STATION_COORDS[target]
                dist = self.get_dist_times(start_pos, target_pos)
                row.append(round(dist, 2))
            self.move_to_times.append(row)
        
        # station params
        self.seed += 1  # for random numbers
        self.rng = np.random.default_rng(seed=self.seed)
        self.input_queues = [[] for _ in range(7)]
        self.output_queues = [[] for _ in range(7)]
        self.process_time_start = [[] for _ in range(7)]
        self.process_time_end = [[] for _ in range(7)]
        self.processing_units = [[] for _ in range(7)]



        
        self.time = 0.0


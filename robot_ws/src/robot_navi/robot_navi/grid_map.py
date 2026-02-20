import numpy as np
from enum import Enum

# Map Dimensions
resolution = 2.0       # Scaling Factor, in m
map_width = 18.0       # 18m
map_height = 16.0      # 16m

class CellType(Enum):
    FREE = 0
    STATION_A = 1
    STATION_B = 2
    STATION_C = 3
    STATION_D = 4
    STATION_E = 5
    STATION_F = 6
    STATION_G = 7
    OBSTACLE = 8
    START = 9

# Represents a Station
# entry_direction requires top, left, bottom, right as input!
class Station:
    def __init__(self, name, x, y, width, height, entry_direction):
        self.name = name
        self.x = int(x // resolution)
        self.y = int(y // resolution)
        self.width = int(width // resolution)
        self.height = int(height // resolution)
        self.entry_direction = entry_direction
        self.center_x = self.x + self.width // 2
        self.center_y = self.y + self.height // 2

        self.entry_point = self._calculate_entry_point()
    
    # Helper to get the coords of the entry point, uses floor division
    def _calculate_entry_point(self):
        

        if self.entry_direction == 'top':
            return (self.center_x, self.y -1) 
        elif self.entry_direction == 'left':
            return (self.x - 1, self.center_y)
        elif self.entry_direction == 'bottom':
            return (self.center_x, self.y + self.height)
        elif self.entry_direction == 'right':
            return (self.x + self.width, self.center_y)
        else:
            raise ValueError(f"Invalid entry direction: {self.entry_direction}")
    
    def __repr__(self):
        return(f"Station (name = {self.name}, pos = ({self.x, self.y}), size = ({self.width, self.height}), entry = {self.entry_direction}), entry_pos = {self.entry_point}")

# Represents a Obstacle
class Obstacle:
    def __init__(self, name, x, y, width, height):
        self.name = name
        self.x = int(x // resolution)
        self.y = int(y // resolution)
        self.width = int(width // resolution)
        self.height = int(height // resolution)
    
    def __repr__(self):
        return(f"Obstacle (name = {self.name}, pos = ({self.x, self.y}), size = ({self.width, self.height}))")

# Represents the Start
class Start:
    def __init__(self, name, x, y):
        self.name = name
        self.x = int(x // resolution)
        self.y = int(y // resolution)
    
    def __repr__(self):
        return(f"Start (name = {self.name}, pos = ({self.x, self.y}))")

class MapGrid:
    def __init__(self):

        self.grid_width = int(map_width / resolution)
        self.grid_height = int(map_height / resolution)

        # create empty grid
        # 0 = empty space, 1-7 = occupied space
        self.grid = np.zeros((self.grid_width, self.grid_height), dtype=[('type', int), ('name', 'U20')])
        self.grid['type'] = CellType.FREE.value
        self.grid['name'] = ''

        self.stations = {}
        self.obstacles = {}
        self.starts = {}

        obstacle_1 = Obstacle(name='OBSTACLE_1', x=2, y=4, width=2, height=8)
        self.add_obstacle(obstacle_1)
        obstacle_2 = Obstacle(name='OBSTACLE_2', x=6, y=4, width=2, height=8)
        self.add_obstacle(obstacle_2)
        obstacle_3 = Obstacle(name='OBSTACLE_3', x=10, y=4, width=2, height=8)
        self.add_obstacle(obstacle_3)
        obstacle_4 = Obstacle(name='OBSTACLE_4', x=14, y=4, width=2, height=8)
        self.add_obstacle(obstacle_4)

        station_a = Station(name='STATION_A', x=4, y=0, width=2, height=2, entry_direction='bottom')
        self.add_station(station_a)
        station_b = Station(name='STATION_B', x=8, y=0, width=2, height=2, entry_direction='bottom')
        self.add_station(station_b)
        station_c = Station(name='STATION_C', x=12, y=0, width=2, height=2, entry_direction='bottom')
        self.add_station(station_c)
        station_d = Station(name='STATION_D', x=16, y=0, width=2, height=2, entry_direction='bottom')
        self.add_station(station_d)
        station_e = Station(name='STATION_E', x=12, y=14, width=2, height=2, entry_direction='top')
        self.add_station(station_e)
        station_f = Station(name='STATION_F', x=8, y=14, width=2, height=2, entry_direction='top')
        self.add_station(station_f)
        station_g = Station(name='STATION_G', x=4, y=14, width=2, height=2, entry_direction='top')
        self.add_station(station_g)

        start_pos = Start(name='START', x=1, y=3)
        self.add_start(start_pos)

        self.console_visualize()

    def add_obstacle(self, obstacle):
        self.obstacles[obstacle.name] = obstacle

        gx1, gy1 = (obstacle.x, obstacle.y)
        gx2, gy2 = (obstacle.x + obstacle.width, obstacle.y + obstacle.height)

        self.grid[gx1:gx2, gy1:gy2]['type'] = CellType.OBSTACLE.value
        self.grid[gx1:gx2, gy1:gy2]['name'] = obstacle.name
    
    def add_station(self, station):
        self.stations[station.name] = station

        gx1, gy1 = (station.x, station.y)
        gx2, gy2 = (station.x + station.width, station.y + station.height)

        self.grid[gx1:gx2, gy1:gy2]['type'] = CellType[f'{station.name}'].value
        self.grid[gx1:gx2, gy1:gy2]['name'] = station.name

    def add_start(self, start):
        self.starts[start.name] = start

        gx1, gy1 = (start.x, start.y)

        self.grid[gx1, gy1]['type'] = CellType.START.value
        self.grid[gx1, gy1]['name'] = start.name

    # Returns grid cell infomation
    def get_cell(self, x, y):
        if not ((0 <= x < self.grid_width) and {0 <= y < self.grid_height}):
            raise ValueError(f"(x, y) out of bounds: {x}, {y}")

        return {
            'type': CellType(self.grid[x, y]['type']),
            'name': self.grid[x, y]['name']
        }
    
    def console_visualize(self):
        print("=== Generated Grid ===")
        print(f"Grid Size: {self.grid_width}, {self.grid_height} Grid Resolution: {resolution}, Map Size: {map_width}, {map_height}")

        symbol_map = {
            CellType.FREE: '.',
            CellType.OBSTACLE: '#',
            CellType.STATION_A: 'A',
            CellType.STATION_B: 'B',
            CellType.STATION_C: 'C',
            CellType.STATION_D: 'D',
            CellType.STATION_E: 'E',
            CellType.STATION_F: 'F',
            CellType.STATION_G: 'G',
            CellType.START: '*'
        }

        for row in range(self.grid_height):
            row_str = "|"
            for cell in range(self.grid_width):
                cell_type = CellType(self.grid[cell, row]['type'])
                row_str += f'{symbol_map.get(cell_type, '?')}|'
            print(row_str)
        
        print('Objects:')
        for start in self.starts.values():
            print(start)
        for station in self.stations.values():
            print(station)
        for obstacle in self.obstacles.values():
            print(obstacle)
            
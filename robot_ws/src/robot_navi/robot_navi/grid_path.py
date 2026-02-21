import numpy as np
import json
import os
from queue import PriorityQueue
try:
    from .grid_map import MapGrid, CellType, resolution, map_width, map_height
except ImportError:
    from grid_map import MapGrid, CellType, resolution, map_width, map_height

robot_width=1.8 # m
robot_height=1.8 # m
robot_speed=2.0 # m/s

# Represents a A* Node
class ANode:
    def __init__(self, x, y, g_cost=float('inf'), h_cost=0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # Cost from start to this node
        self.h_cost = h_cost  # Heuristic cost to goal
        self.f_cost = g_cost + h_cost  # Total cost
        self.parent = parent  # Parent node for path reconstruction
    
    # For priority queue comparison
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


class AGridPath:
    def __init__(self, grid_map):
        self.grid = grid_map
        
        self.robot_width_grid = int(robot_width / resolution)
        self.robot_height_grid = int(robot_height / resolution)
        self.robot_speed_grid = resolution / robot_speed
        
        print(f'Real Robot Size: {robot_width, robot_height}m, Speed {robot_speed}m/s')
        print(f'Scaled Size: {self.robot_width_grid, self.robot_height_grid}, Speed {self.robot_speed_grid}')
        
        self.path_lookup = {}
        self._precompute_all_paths()
    
    # Manhattan distance
    def _heuristic(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)
    
    def _is_valid_position(self, x, y):
        # Bounds check
        if x < 0 or x >= self.grid.grid_width or y < 0 or y >= self.grid.grid_height:
            return False
        
        # Robot sizing check
        half_w = self.robot_width_grid // 2
        half_h = self.robot_height_grid // 2
        
        for dx in range(-half_w, half_w + 1):
            for dy in range(-half_h, half_h + 1):
                rx = x + dx
                ry = y + dy
                
                # Out of bounds
                if rx < 0 or rx >= self.grid.grid_width or ry < 0 or ry >= self.grid.grid_height:
                    return False
                
                # Check if cell is obstacle
                cell_type = self.grid.grid[rx, ry]['type']
                if cell_type == CellType.OBSTACLE.value:
                    return False
        
        return True
    
    # path directions: up, down, left, right
    def _get_neighbors(self, x, y):
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self._is_valid_position(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    # Fing path from start(x,y) to goal(x,y)
    def find_path(self, start_x, start_y, goal_x, goal_y):
        if not self._is_valid_position(start_x, start_y):
            print(f"  Start position ({start_x}, {start_y}) is invalid")
            return None
        
        if not self._is_valid_position(goal_x, goal_y):
            print(f"  Goal position ({goal_x}, {goal_y}) is invalid")
            return None
        
        start_node = ANode(start_x, start_y, g_cost=0, h_cost=self._heuristic(start_x, start_y, goal_x, goal_y))
        
        # Open set (to be explored)
        open_set = PriorityQueue()
        open_set.put(start_node)
        
        # Track nodes in open set for quick lookup
        open_set_hash = {(start_x, start_y)}
        
        # Closed set (already explored)
        closed_set = set()
        
        # Cost tracking
        g_costs = {(start_x, start_y): 0}
        
        while not open_set.empty():
            # Get node with lowest f_cost
            current = open_set.get()
            current_pos = (current.x, current.y)
            
            # Remove from open set hash
            if current_pos in open_set_hash:
                open_set_hash.remove(current_pos)
            
            # Check if we reached the goal
            if current.x == goal_x and current.y == goal_y:
                return self._reconstruct_path(current)
            
            # Add to closed set
            closed_set.add(current_pos)
            
            # Explore neighbors
            for nx, ny in self._get_neighbors(current.x, current.y):
                neighbor_pos = (nx, ny)
                
                # Skip if already explored
                if neighbor_pos in closed_set:
                    continue
                
                # Calculate tentative g_cost
                tentative_g = current.g_cost + 1  # Cost of moving to neighbor is 1
                
                # Check if this path is better
                if neighbor_pos not in g_costs or tentative_g < g_costs[neighbor_pos]:
                    g_costs[neighbor_pos] = tentative_g
                    h_cost = self._heuristic(nx, ny, goal_x, goal_y)
                    neighbor_node = ANode(nx, ny, g_cost=tentative_g, h_cost=h_cost, parent=current)
                    
                    if neighbor_pos not in open_set_hash:
                        open_set.put(neighbor_node)
                        open_set_hash.add(neighbor_pos)
        
        return None
    
    # reconstructs path from node
    def _reconstruct_path(self, node):
        path = []
        current = node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        path.reverse()
        return path
    
    def _precompute_all_paths(self):
        print("=== Pre-computing A* paths ===")
        
        # Get all location names
        all_locations = list(self.grid.starts.keys()) + list(self.grid.stations.keys())
        
        total_paths = 0
        successful_paths = 0
        
        for from_name in all_locations:
            for to_name in all_locations: # permutate all locations
                if from_name == to_name:
                    continue
                
                # Determine start position
                if from_name in self.grid.starts:
                    start_obj = self.grid.starts[from_name]
                    from_x, from_y = start_obj.x, start_obj.y
                else:
                    station_obj = self.grid.stations[from_name]
                    from_x, from_y = station_obj.entry_point
                
                # Determine goal position
                if to_name in self.grid.starts:
                    # goal_obj = self.grid.starts[to_name]
                    # to_x, to_y = goal_obj.x, goal_obj.y
                    continue # to START is not needed
                else:
                    station_obj = self.grid.stations[to_name]
                    to_x, to_y = station_obj.entry_point
                
                print(f"Computing: {from_name} ({from_x},{from_y}) -> {to_name} ({to_x},{to_y})", end=" ")
                path = self.find_path(from_x, from_y, to_x, to_y)
                
                total_paths += 1
                if path:
                    # add step into/out of station
                    if to_name in self.grid.stations:
                        center_x = self.grid.stations[to_name].center_x
                        center_y = self.grid.stations[to_name].center_y
                        path.append((center_x, center_y))
                    
                    if from_name in self.grid.stations:
                        station_obj = self.grid.stations[from_name]
                        center_x, center_y = station_obj.center_x, station_obj.center_y
                        entry_x, entry_y = station_obj.entry_point
                        
                        # Add entry point first, then center
                        path.insert(0, (entry_x, entry_y))  # Entry point
                        path.insert(0, (center_x, center_y))  # Center
                    
                    self.path_lookup[(from_name, to_name)] = path
                    successful_paths += 1
                    print(f'= {len(path)} steps')
                else:
                    self.path_lookup[(from_name, to_name)] = None
                    print(f'= NO PATH')
        
        print(f'Computed {successful_paths}/{total_paths} paths')
    
    # Save computed paths and metadata into json
    def save_paths(self, filename='grid_path.json'):
        serializable_paths = {}
        
        for (from_name, to_name), path in self.path_lookup.items():
            key = f'{from_name, to_name}'
            if path is not None:
                path_length = len(path)
                path_time = (path_length - 1) * self.robot_speed_grid # -1 since start doesnt count
                # Convert tuple into list
                serializable_paths[key] = {
                    'from': from_name,
                    'to': to_name,
                    'path': [[int(x), int(y)] for x, y in path],
                    'length': path_length,
                    'time': path_time
                }
            # else: # ignore not used paths
            #     serializable_paths[key] = {
            #         'from': from_name,
            #         'to': to_name,
            #         'path': None,
            #         'length': 0,
            #         'time': 0
            #     }
        
        # Add metadata
        output_data = {
            'metadata': {
                'map_width': map_width,
                'map_height': map_height,
                'resolution': resolution,
                'robot_width': robot_width,
                'robot_height': robot_height,
                'robot_speed': robot_speed,
                'grid_width': self.grid.grid_width,
                'grid_height': self.grid.grid_height,
                'robot_width_grid': self.robot_width_grid,
                'robot_height_grid': self.robot_height_grid,
                'robot_speed_grid': self.robot_speed_grid
            },
            'paths': serializable_paths
        }
        
        # Write to file
        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"Paths saved to {filename}")
        print(f"  Total paths: {len(serializable_paths)}")
    
    def get_path(self, from_location, to_location):
        return self.path_lookup.get((from_location, to_location))
    
    # used for debug
    def visualize_path(self, path):
        if path is None:
            print("No path to visualize")
            return
        
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
        
        # Create path set for quick lookup
        path_set = set(path)
        
        print("=== Path Visualization ===")
        for y in range(self.grid.grid_height):
            row_str = "|"
            for x in range(self.grid.grid_width):
                if (x, y) in path_set:
                    row_str += "+|"  # Path marker
                else:
                    cell_type = CellType(self.grid.grid[x, y]['type'])
                    row_str += f"{symbol_map.get(cell_type, '?')}|"
            print(row_str)

# Helper to load from file
class PathLoader:
    @staticmethod
    def load_paths(filename):
        with open(filename, 'r') as f:
            data = json.load(f)
        
        print(f"Loaded paths from {filename}")
        print(f"  Grid: {data['metadata']['grid_width']}x{data['metadata']['grid_height']}")
        print(f"  Resolution: {data['metadata']['resolution']}m")
        print(f"  Total paths: {len(data['paths'])}")
        
        return data
    
    @staticmethod
    def get_path(data, from_location, to_location):
        key = f"{from_location}->{to_location}"
        path_data = data['paths'].get(key)
        
        if path_data and path_data['path'] is not None:
            return path_data['path']
        else:
            return None


# Example usage
if __name__ == '__main__':
    # Create grid
    grid = MapGrid()
    
    # Create planner with 2x2m robot (automatically scaled by resolution)
    planner = AGridPath(grid)
    
    # Save paths to file
    planner.save_paths('grid_path.json')
    
    # # Test loading paths
    # print("\n=== Testing Path Loading ===")
    # loaded_data = PathLoader.load_paths('grid_path.json')
    
    # # Test retrieving a specific path
    # test_path = PathLoader.get_path(loaded_data, 'START', 'STATION_A')
    # print(f"\nLoaded path START -> STATION_A: {test_path}")
    # print(f"Length: {len(test_path) if test_path else 0}")
    
    # # Print all available paths
    # print("\n=== All Available Paths ===")
    # for key, path_info in sorted(loaded_data['paths'].items()):
    #     if path_info['path']:
    #         print(f"{path_info['from']:12} -> {path_info['to']:12}: {path_info['length']:3} steps")
    #     else:
    #         print(f"{path_info['from']:12} -> {path_info['to']:12}: NO PATH")
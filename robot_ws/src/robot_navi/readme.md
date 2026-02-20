# Robot Navigation Package
## Generate Grid
This package needs a pre-computed grid, which can be created using:
`python3 ./robot_navi/grid_path.py`

This creates a json file that contains all possible paths, as well as the travel times.

## Modify Grid & Robot
The Grid can be modified in `grid_map.py` with width, height and resolution. Resolution is in this case a scaling factor and determines how large each cell is for the given map dimensions. Based on the resolution the size of all objects, as well as the robot speed is determined.

The Robot can be modified in `grid_path.py` with width, height and speed. These values are scaled by the resolution.

If a value has been modified then the grid needs to be regenerated!
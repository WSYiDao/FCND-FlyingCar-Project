# FCND - 3D Motion Planning
Project base on https://github.com/udacity/FCND-Motion-Planning

### Solution and Writeup
#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### Explain the functionality of what's provided in code
##### `motion_planning.py` 
1. 3 callback function for judging which state transition function to use by the current position,velocity and state.
2. There are 8 drone States includes Manual, Arming, Takeoff, Waypoint, Landing, Disarming and Planning. Compare to the `backyard_flyer_solution.py`, PLANNING state is added in the code after Arming state.
3. `plan_path`function find a path from the start position to the goal position. In the code we converted to current local position from global position based on north and eat offset. Then it used A* to find the minimum cost path.
 
##### `planning_utils.py`
1. `create_grid` function to init a grid representation of a 2D configuration space and minimum north and east coordinates based on given obstacle data.
2. `Action` Enum include 8 directions to search the goal position, and the cost of move action.
3. `valid_actions` function return a list of valid actions given a grid and current node. Actions is based on the Action Enum.
4. `a_star` used A* with PriorityQueue and `heuristic` function to find the minimum cost  path from start to goal. 



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

The first line of the colliders.csv file is `lat0 37.792480, lon0 -122.397450`

```python

with open('colliders.csv', 'r') as fd:
    data = fd.readline().split(",")
    print(data[0].split(' '))
lat0 = float(data[0].strip().split(' ')[1])
lon0 = float(data[1].strip().split(' ')[1])

self.set_home_position(lon0,lat0,0)

```

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

`self.global_home` is set by `sel.set_home_position()` in the part 1. So it's a good way to use  `global_to_local()`  function convert to current local position from current global poistion .

```python
#  retrieve current global position
c_global_position = [self._longitude, self._latitude, self._altitude]

#  convert to current local position using global_to_local()
c_local_position = global_to_local(c_global_position, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

Adding start location base on the east and north offset.

```python
# convert start position to current position rather than map center
grid_start = (int(c_local_position[0] - north_offset), int(c_local_position[1] - east_offset))
```


#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

It's hard to choose (lat, lon) direct. So in the code  I used `local_to_global()`  and `global_to_local` function, and the `vector` to init the goal I want to choose.

```python
# TODO: adapt to set goal as latitude / longitude position and convert
vector = (100,-100)
(latitude , longitude , _ ) = local_to_global((c_local_position[0] + vector[0], c_local_position[1] + vector[1], 0), self.global_home)
goal_glo_pos = global_to_local([latitude, longitude, self._altitude], self.global_home)
grid_goal = (int(goal_glo_pos[0] - north_offset), int(goal_glo_pos[1] - east_offset))
```  


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

1. Add 4 direction in `Action` Enum
2. Modify the `valid_actions`function

```python
    up_right =  (-1,  1, np.sqrt(2))
    up_left =   (-1, -1, np.sqrt(2))
    down_right= ( 1,  1, np.sqrt(2))
    down_left = ( 1, -1, np.sqrt(2))
```

```python
def valid_actions(grid, current_node):
    actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    valid_actions = []
    for action in actions:
        da = action.delta
        new_x = current_node[0] + da[0]
        new_y = current_node[1] + da[1]
        if( new_x < 0 or new_x > n or new_y < 0 or new_y > m):
            continue
        if(grid(new_x,new_y) == 1):
            continue
        valid_actions.append(action)
    return valid_actions
```


#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

I used a collinearity test to prune my path of unnecessary waypoints. The function `prune_path` remove middle point of path when 3 point are collinearity.

```python
def collinearity_check(p1, p2, p3, epsilon):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path,epsilon=1e-6):
    if path is not None:
        pruned_path = [p for p in path]

        i = 0
        while i < len(pruned_path) - 2:
            p1 = point(pruned_path[i])
            p2 = point(pruned_path[i+1])
            p3 = point(pruned_path[i+2])
            if collinearity_check(p1, p2, p3,epsilon):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
    else:
        pruned_path = path

    return pruned_path
```

### Execute the flight
#### 1. Does it work?
It works!


# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


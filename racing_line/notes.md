# Next Steps: 
1. Draw racing line, ie. if right turn then move left before turning 
2. Implement time into speed 
   - go through different versions of the map and calculate which one will finish faster
3. Smooth racing line - optimise and direct it to a certain direction

# Active Files: 
- run.py --> central running platform for all files
- map.py
- speed.py (in prog) --> finds the ideal speed with track waypoints 
   - doesn't take time into account
- draw_map.py (modified as files are added) --> draws the map inputed 

---------------------------------------------------------------------  

# Path Finding Strategies: 
## Minimum Curvature Path 
1. Find track boundaries (walls)
2. Create a graph of possible paths
3. Minimize total curvature along the path
4. Path with least curvature = highest average speed

## Time-Optimal Path 
1. Discretize track into segments
2. For each segment, calculate:
   - Maximum cornering speed: v = sqrt(μ * g * r)
   - Braking/acceleration limits
3. Use dynamic programming or optimization to find:
   - Path that minimizes total lap time
   - Subject to: physics constraints, track boundaries

## Model Predictive Control
1. Define vehicle dynamics model (bicycle model)
2. Set up optimization problem:
   - Minimize: lap time
   - Variables: steering angle, throttle/brake, position
   - Constraints: track boundaries, tire friction circle
3. Solve using nonlinear optimization (like IPOPT)

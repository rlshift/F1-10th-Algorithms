import numpy as np
import matplotlib.pyplot as plt
import math

# ==============================================================================
# PURE PURSUIT ALGORITHM - Path Tracking Controller
# ==============================================================================
# Pure Pursuit is a geometric path tracking algorithm that calculates the 
# steering angle needed to follow a path by looking at a target point ahead
# of the vehicle on the path.
#
# Key Concept: The algorithm finds a point on the path ahead of the vehicle
# (the "lookahead point") and steers toward it using geometric relationships.
# ==============================================================================

k = 0.1        # Look-ahead gain: Controls how lookahead distance changes with speed
               
Lfc = 1.0      # Base look-ahead distance [m]: Minimum distance to look ahead on the path
               
Kp = 1.0       # Speed proportional gain: Controls how aggressively to reach target speed

dt = 0.1       # Time step [s]: Simulation update interval

WB = 2.9       # Wheelbase [m]: Distance between front and rear axles of the vehicle
               
class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x      # X-position of vehicle center [m]
        self.y = y      # Y-position of vehicle center [m]
        self.yaw = yaw  # Heading angle [rad]: 0 = pointing right, π/2 = pointing up
        self.v = v      # Velocity [m/s]
        
        # Calculate rear axle position 
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

        """
        Update vehicle state using bicycle model kinematics.
        
        Args:
            a: Acceleration [m/s²]
            delta: Steering angle [rad] (positive = left turn)
        
        Bicycle Model Equations:
        - x' = v * cos(yaw)       : Velocity component in x-direction
        - y' = v * sin(yaw)       : Velocity component in y-direction
        - yaw' = (v/L) * tan(δ)   : Angular velocity (L = wheelbase, δ = steering)
        - v' = a                  : Change in velocity from acceleration
        """

    def update(self, a, delta):
        # Update position based on current velocity and heading
        self.x += self.v * math.cos(self.yaw) * dt # velocity * time in x direction
        self.y += self.v * math.sin(self.yaw) * dt # velocity * time in y direction
        
        # Update heading based on steering angle (bicycle model)
        self.yaw += self.v / WB * math.tan(delta) * dt # angular velocity * time
        
        # Update velocity based on acceleration
        self.v += a * dt
        
        # Recalculate rear axle position after state update
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))


# the further you are from the target speed, the stronger the acceleration command.
# Kp is speed proportional gain
def proportional_control(target, current):
    a = Kp * (target - current)
    return a


"""
    Pure Pursuit algorithm.
    
    Args:
        state: Current vehicle state
        cx, cy: Arrays of x and y coordinates defining the path
        pind: Previous target index (to prevent looking backwards)
    
    Returns:
        delta: Steering angle [rad]
        ind: Current target point index on the path
    
    Pure Pursuit Logic:
    1. Find the target point ahead on the path (lookahead point)
    2. Calculate the angle (alpha) between vehicle heading and target
    3. Use geometry to compute steering angle needed to reach that point
"""

# PURE PURSUIT STEERING CONTROL - Core Algorithm
def pure_pursuit_steer_control(state, cx, cy, pind):
    
    # Find the lookahead target point on the path
    ind = calc_target_index(state, cx, cy)

    # Prevent going backwards - ensure we're always moving forward along path
    if pind >= ind:
        ind = pind

    # Get the coordinates of the target lookahead point
    if ind < len(cx):
        tx = cx[ind]  # Target x-coordinate
        ty = cy[ind]  # Target y-coordinate
    else:  # If we've reached the end, aim for the final point
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    # Calculate alpha: the angle between the vehicle's heading and the target point
    # atan2(dy, dx) gives angle from rear axle to target point
    # Subtracting state.yaw gives the angle relative to vehicle's current heading
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # ==== THE PURE PURSUIT FORMULA ====
    # Calculate lookahead distance: increases with speed for smoother high-speed tracking
    # At low speeds: Lf ≈ Lfc (minimum lookahead), high speeds: Lf increases proportionally with v
    Lf = k * state.v + Lfc
    
    # This formula ensures the vehicle follows a circular arc that passes through lookahead point.
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


"""
    Target Index Calculation
    Find the index of the target lookahead point on the path.
    
    Args:
        state: Current vehicle state
        cx, cy: Path coordinates
    
    Returns:
        ind: Index of the target point to aim for
    
    Algorithm:
    1. Find the nearest point on the path to the vehicle
    2. Move forward along the path until we've traveled the lookahead distance
"""

# TARGET INDEX CALCULATION - Finding the Lookahead Point
def calc_target_index(state, cx, cy):

    # Calculate distance from rear axle to each point on the path
    dx = [state.rear_x - icx for icx in cx]  # X-distance to each path point
    dy = [state.rear_y - icy for icy in cy]  # Y-distance to each path point
    
    # Compute Euclidean distance to each point
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d)) # index of closest point on path
    
    # Move forward along the path until we reach the lookahead distance
    L = 0.0  # Accumulated distance along the path
    Lf = k * state.v + Lfc  # Target lookahead distance (speed-dependent)

    # Walk forward along the path, accumulating distance
    while Lf > L and (ind + 1) < len(cx):
        # Calculate distance between consecutive path points
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)  # Add segment length to total
        ind += 1  # Move to next point

    return ind

def main():
    
    cx = np.arange(0, 50, 0.5)  # X-coordinates: 0 to 50 meters in 0.5m steps
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]  # Y-coordinates: sine wave

    target_speed = 10.0 / 3.6  # [m/s] ≈ 2.78 m/s or 10 km/h

    state = State(x=0.0, y=-3.0, yaw=0.0, v=0.0)
    lastIndex = len(cx) - 1
    time = 0.0
    
    # Get initial target index
    target_ind, _ = pure_pursuit_steer_control(state, cx, cy, 0)

    # Setup visualization
    plt.ion()  # Turn on interactive mode for real-time plotting
    fig, ax = plt.subplots(figsize=(10, 5))

    while lastIndex > target_ind:
        # Speed control: calculate acceleration to reach target speed
        ai = proportional_control(target_speed, state.v)
        
        # Steering control: calculate steering angle using Pure Pursuit
        di, target_ind = pure_pursuit_steer_control(state, cx, cy, target_ind)
        
        # Update vehicle state with the computed controls
        state.update(ai, di)
        time += dt

        plt.cla()  # Clear the current plot
        plt.plot(cx, cy, ".r", label="Course")
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="Target")
        plt.plot(state.x, state.y, "ob", label="Car")
        
        # Draw an arrow showing vehicle heading direction
        plt.arrow(state.x, state.y, math.cos(state.yaw), math.sin(state.yaw), 
                  head_width=0.5, head_length=0.5, fc='blue', ec='blue')
        
        # Configure plot appearance
        plt.axis("equal")  # Equal aspect ratio (so circles look circular)
        plt.grid(True)
        plt.title("Pure Pursuit Simulation (Speed: " + str(round(state.v * 3.6, 2)) + " km/h)")
        plt.pause(0.001)  # Brief pause to update the plot
    plt.show()

if __name__ == '__main__':
    main()
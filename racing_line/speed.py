import math

class Controls:
    ## 1 is reserved for racing line at the moment
    ACCEL = 2
    BRAKE = 3
    HOLD = 4

def calculate_curvature(p1, p2, p3):
    """
    Calculate curvature at p2 using three consecutive points.
    Returns 0 for straight, higher values for tighter corners.
    """
    # Vector from p1 to p2
    v1 = [p2[0] - p1[0], p2[1] - p1[1]]
    # Vector from p2 to p3
    v2 = [p3[0] - p2[0], p3[1] - p2[1]]
    
    # Calculate lengths
    len1 = math.sqrt(v1[0]**2 + v1[1]**2)
    len2 = math.sqrt(v2[0]**2 + v2[1]**2)
    
    if len1 == 0 or len2 == 0:
        return 0
    
    # Normalize vectors
    v1_norm = [v1[0]/len1, v1[1]/len1]
    v2_norm = [v2[0]/len2, v2[1]/len2]
    
    # Calculate angle between vectors using dot product
    dot = v1_norm[0]*v2_norm[0] + v1_norm[1]*v2_norm[1]
    dot = max(-1, min(1, dot))  # Clamp to avoid numerical errors
    angle = math.acos(dot)
    
    # Curvature is proportional to angle change
    return angle

def calculate_max_speeds(waypoints, corner_data, base_speed, min_corner_speed):
    max_speeds = []
    
    for i in range(len(waypoints)):
        if i == 0 or i == len(waypoints) - 1:
            max_speeds.append(base_speed * 0.5)
        else:
            # cp = corner_data[i]['cp']
            if i < len(corner_data):
                cp = corner_data[i]['cp']
            else:
                cp = 0
            
            if cp == 0:
                max_speeds.append(base_speed)
            else:
                # Use inverse relationship: tighter turn (bigger |cp|) = slower speed
                # Cap at reasonable maximum curvature
                max_cp = 50.0  # Adjust based on your track
                corner_factor = min(1.0, abs(cp) / max_cp)
                max_speeds.append(base_speed * (1.0 - corner_factor * 0.8) + min_corner_speed * 0.2)
    
    return max_speeds

def apply_braking_zones(max_speeds, max_decel):
    # Work backwards to ensure we can brake in time for corners.
    
    speeds = max_speeds.copy()
    
    # Work backwards from end
    for i in range(len(speeds) - 2, -1, -1):
        # Maximum speed we can have at point i and still brake to speeds[i+1]
        max_entry_speed = speeds[i+1] + max_decel
        
        # Limit current speed
        speeds[i] = min(speeds[i], max_entry_speed)
    
    return speeds

def apply_acceleration_zones(speeds, max_accel):
    """
    Work forwards to model acceleration.
    """
    final_speeds = speeds.copy()
    
    # Work forwards from start
    for i in range(1, len(final_speeds)):
        # Maximum speed we can reach from previous point
        max_reachable = final_speeds[i-1] + max_accel
        
        # Take minimum of what we can reach and what's safe
        final_speeds[i] = min(final_speeds[i], max_reachable)
    
    return final_speeds

def translate_to_controls(speeds):
    """
    Convert speed profile to control inputs.
    Returns: 2=Accelerate, 3=Brake, 4=Maintain
    """
    controls = []
    
    for i in range(len(speeds) - 1):
        diff = speeds[i+1] - speeds[i]
        
        if diff > 0.2:
            controls.append(Controls.ACCEL)  # Accelerate
        elif diff < -0.2:
            controls.append(Controls.BRAKE)  # Brake
        else:
            controls.append(Controls.HOLD)  # Hold
    
    controls.append(4)  # Final point
    return controls

def get_control_name(value):
    if value == Controls.ACCEL:
        return "ACCEL"
    elif value == Controls.BRAKE:
        return "BRAKE"
    elif value == Controls.HOLD:
        return "HOLD"
    return "UNKNOWN"


def optimize_racing_line(waypoints, corner_data, base_speed, min_corner_speed, 
                         max_accel, max_decel):
    """
    Main optimization function.
    """
    print("Waypoints:", len(waypoints))
    
    # Step 1: Calculate maximum safe speeds based on track geometry
    max_speeds = calculate_max_speeds(waypoints, corner_data, base_speed, min_corner_speed)
    print("\nMax speeds (based on curvature):", [round(s, 1) for s in max_speeds])
    
    # Step 2: Apply braking zones (work backwards)
    speeds_with_braking = apply_braking_zones(max_speeds, max_decel)
    print("After braking zones:", [round(s, 1) for s in speeds_with_braking])
    
    # Step 3: Apply acceleration limits (work forwards)
    final_speeds = apply_acceleration_zones(speeds_with_braking, max_accel)
    print("Final speed profile:", [round(s, 1) for s in final_speeds])
    
    # Step 4: Translate to control inputs
    controls = translate_to_controls(final_speeds)
    print("\nControls:", [get_control_name(c) for c in controls])
    print(controls)
    diffs = [round(final_speeds[i+1] - final_speeds[i], 2)
         for i in range(len(final_speeds)-1)]
    print("Speed diffs:", diffs)

    return final_speeds, controls


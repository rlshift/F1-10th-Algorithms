import time
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(-1.0, 1.0)):
        self.kp = kp  # Proportional Gain
        self.ki = ki  # Integral Gain
        self.kd = kd  # Derivative Gain
        self.dt = dt  # Time step (seconds)
        
        self.min_out, self.max_out = output_limits
        
        # Memory variables (State)
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, target, current):
        # 1. Calculate Error
        error = target - current

        # 2. Proportional Term (P) == sluggish / aggressive acceleration
        # "How far off am I right now?"
        p_term = self.kp * error

        # 3. Integral Term (I) == adds force to overcome steady-state error
        # "How long have I been wrong?"
        # We add current error to the accumulated history
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # 4. Derivative Term (D) == damper / shock absorber
        # "How fast is the error shrinking?"
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        # 5. Total Output
        output = p_term + i_term + d_term

        # 6. Anti-Windup / Clamping
        # Real motors can't go 500% speed. We clip the output to +/- 1.0
        # We also stop the integral from growing if we are maxed out
        if output > self.max_out:
            output = self.max_out
            self.integral -= error * self.dt 
        elif output < self.min_out:
            output = self.min_out
            self.integral -= error * self.dt

        # 7. Save state for next loop
        self.prev_error = error

        return output

# ==========================================
# PART 2: The Fake Car (The Plant)
# ==========================================
class FakeCar:
    def __init__(self, mass=100.0, drag=2.0, dt=0.1):
        self.velocity = 0.0
        self.mass = mass
        self.drag = drag # Air resistance/Friction
        self.dt = dt

    def step(self, throttle_command):
        """
        Input: throttle_command (-1.0 to 1.0)
        Output: New velocity
        Physics: F = ma -> a = F/m
        """
        # Engine force (arbitrary 500 Newtons max force)
        force = throttle_command * 500.0 
        
        # Air resistance (Drag increases as speed increases)
        drag_force = self.drag * self.velocity
        
        # Net acceleration
        acceleration = (force - drag_force) / self.mass
        
        # Update velocity (Euler integration)
        self.velocity += acceleration * self.dt
        
        return self.velocity

def run_experiment(kp, ki, kd):
    dt = 0.1 # 10Hz
    total_time = 20.0 # seconds
    steps = int(total_time / dt)

    # Initialize Objects
    pid = PIDController(kp=kp, ki=ki, kd=kd, dt=dt)
    car = FakeCar(dt=dt)

    # Data logs for plotting
    times = []
    targets = []
    actuals = []
    throttles = []

    current_target = 0.0

    for i in range(steps):
        t = i * dt
        
        # Change target speed at different times to test response
        if t < 2.0: current_target = 0.0
        elif t < 10.0: current_target = 10.0 # Jump to 10 m/s
        else: current_target = 5.0 # Drop to 5 m/s

        # 1. Get Control Command from PID
        throttle = pid.update(current_target, car.velocity)

        # 2. Apply to Car
        actual_speed = car.step(throttle)

        # 3. Log Data
        times.append(t)
        targets.append(current_target)
        actuals.append(actual_speed)
        throttles.append(throttle)

    return times, targets, actuals

# ==========================================
# PART 4: Run and Visualize
# ==========================================
if __name__ == "__main__":
    # --- EXPERIMENT ZONE: CHANGE THESE NUMBERS ---
    # Try 1: Pure Proportional (kp=0.5, ki=0, kd=0) -> Should fail to reach target
    # Try 2: Perfect Tuning (kp=2.0, ki=1.5, kd=0.1) -> Should work well
    # Try 3: Oscillation (kp=15.0, ki=0, kd=0) -> Should wobble
    
    kp_val = 1.5
    ki_val = 4.0
    kd_val = 0.1
    
    times, targets, actuals = run_experiment(kp_val, ki_val, kd_val)

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(times, targets, 'r--', label='Target Speed')
    plt.plot(times, actuals, 'b-', linewidth=2, label=f'Actual Speed (Kp={kp_val}, Ki={ki_val})')
    plt.title('PID Control Learning Lab')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True)
    plt.legend()
    plt.show()
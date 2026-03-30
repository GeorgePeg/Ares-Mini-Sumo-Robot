#Author: Georgios Pegiazis
#Date: 30/03/2026
#Version: 1.1
#License: Copyright © 2026 Georgios Pegiazis. All rights reserved.
#Description: PID Simulator for Mini Sumo Robot using Python. The app.py is the main script that connects the 
#pid_logic.py with a simple GUI using the Flask framework.
from flask import Flask, render_template, request, jsonify
from pid_logic import PIDController
import numpy as np

#Initializing the Flask app
app = Flask(__name__)
#Creating an instance based on some characteristics of Ares, the Mini Sumo Robot
mass = 0.5 #Mass of the robot in kg
pwm_limit = 255 #Maximum PWM value for the motors
friction = 0.2 #Friction coefficient
maximum_force = 10 #Maximum motor force in Newtons

def simulate_pid(current_pos, current_speed, pwm_input, dt):
    #Translating the PWM input to force
    force = (pwm_input / pwm_limit) * maximum_force
    friction_force = friction * current_speed
    net_force = force - friction_force
    a = net_force / mass
    new_speed = current_speed + a*dt
    new_pos = current_pos + new_speed*dt
    return new_pos,new_speed
# --- Routes ---
@app.route('/')
def index():
    return render_template('pid_simulator.html')

@app.route('/simulate', methods=['POST'])
def simulate():
    data = request.json
    
    # Extracting PID parameters and setpoint from the request
    kp = float(data.get('kp', 20.0))
    ki = float(data.get('ki', 5.0))
    kd = float(data.get('kd', 1.0))
    setpoint = float(data.get('setpoint', 100.0)) # Setpoint in cm
    
    # Initializing the PID Controller with the given parameters and output limits
    pid = PIDController(kp, ki, kd, output_limits=(-pwm_limit, pwm_limit))
    
    #Simulation parameters
    position = 0.0      # cm
    velocity = 0.0      # cm/s
    
    # Simulation parameters
    duration = 10.0      # time in seconds
    dt = 0.02           # 20ms per update
    time_steps = int(duration / dt)
    
    # Saving the results for visualization
    history = {
        'time': [],
        'position': [],
        'pwm': []
    }
    
    # Loop simulation
    for i in range(time_steps):
        t = i * dt
        
        # PID Calculation
        pwm_out = pid.calculation(setpoint, position, dt)
        
        #  Calculating the new position and speed based on the PID output
        position, velocity = simulate_pid(position, velocity, pwm_out, dt)
        
        # Save
        history['time'].append(round(t, 2))
        history['position'].append(position)
        history['pwm'].append(pwm_out)
        
    return jsonify(history)

if __name__ == '__main__':
    app.run(debug=True, port=5000)

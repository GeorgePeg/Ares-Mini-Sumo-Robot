#Author: Georgios Pegiazis
#Date: 30/03/2026
#Version: 1.1
#License: Copyright © 2026 Georgios Pegiazis. All rights reserved.
#Description: PID Simulator for Mini Sumo Robot using Python. This script is responsible
#for the PID Control logic. The pid_logic.py then is imported in the app.py where the backend
#connects with a simple GUI that visualizes the PID control charts.

class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits = (None, None)):
        #Initializing the PID Controller parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.minimum, self.maximum =  output_limits #Referring to PWM limits for the motors
        self.pidError = 0
        self.pidLastError = 0
        self.pidIntegral = 0

    def calculation(self,setpoint, value, dt):
    #Calculating the PID output based on the setpoint and the current value. 
    #The dt parameter is the time interval between calulations.
        if dt <= 0 : return 0 #We do that to avoid division by zero in the derivative term
        error = setpoint - value
       #Proportional Term
        P = self.Kp * error
        #Integral Term
        I = self.Ki * (self.pidIntegral + error * dt)
        if self.maximum is not None:
            I = max(self.minimum, min(I,self.maximum))
        #Derivative Term
        derivative = (error - self.pidLastError) / dt
        D = self.Kd  * derivative
        #Total PID Output
        output = P + I + D
        if self.maximum is not None:
            output = max(self.minimum, min(output,self.maximum))
        self.pidLastError = error
        return output
    def reset(self):
        self.pidError = 0
        self.pidIntegral = 0
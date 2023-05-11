import time




def clamp(val: float, lower_limit: float, upper_limit: float):
    return lower_limit if val <= lower_limit else upper_limit if val > upper_limit else val

class PID:

    def __init__(self,
                 setpoint: float = 0.0,
                 kp: float = 0.0,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 min_output: float = None,
                 max_output: float = None,
                 dt_min: float = 0.01):
        
        """
        setpoint: PID controller setpoint
        kp: proportional gain constant
        ki: integral gain constant
        kd: derivative gain constant
        dt_min: minimum time between error corrections
        """
        self.setpoint = setpoint
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt_min = dt_min
        self.min_output = min_output
        self.max_output = max_output
        
        self.p_out: float = 0.0
        self.i_out: float = 0.0
        self.d_out: float = 0.0
        self.prev_time = time.time()
        self.prev_input: float = 0.0
        self.prev_output: float = 0.0


    def __call__(self, input):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        d_input = input - self.prev_input

        if dt < self.dt_min:
            return self.prev_output
        
        error = self.setpoint - input

        # proportional control signal
        self.p_out = self.kp * error

        # integral control signal
        self.i_out += self.ki * error * dt
        self.i_out = clamp(self.i_out, self.min_output, self.max_output)

        # derivative control signal
        self.d_out = self.kd * d_input / dt

        output = self.p_out + self. i_out + self.d_out
        output = clamp(output, self.min_output, self.max_output)

        self.prev_input = input
        self.prev_output = output

        return output
    
    def tune(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_kp(self):
        return self.kp
    
    def get_ki(self):
        return self.ki
    
    def get_kd(self):
        return self.kd
    



    

from .basic_pid import PID




class ObjectFollower:

    def __init__(self):
        self.yaw_pid = PID(160, 1.0, 0.0, 0.5, -1000, 1000)
        self.thrust_pid = PID(160, 1.0, 0.5, 0.2, 0, 1000)
        self.pitch_pid = PID(70, 0.2, 1.0, 0.5, -1000, 1000)
        self.yaw_out = 0
        self.thrust_out = 0
        self.pitch_out = 0

    def __call__(self, x, y, distance):
        self.yaw_out = int(self.yaw_pid(x))
        self.thrust_out = int(self.thrust_pid(y))
        self.pitch_out = int(self.pitch_pid(distance))

    def tune_yaw(self, kp, ki, kd):
        self.yaw_pid.tune(kp, ki, kd)

    def tune_thrust(self, kp, ki, kd):
        self.thrust_pid.tune(kp, ki, kd)

    def tune_pitch(self, kp, ki, kd):
        self.pitch_pid.tune(kp, ki, kd)




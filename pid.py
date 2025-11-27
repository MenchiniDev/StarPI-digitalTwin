class PID:
    def __init__(self, Kp, Ki, Kd, dt, u_min=0.0, u_max=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.u_min = u_min
        self.u_max = u_max

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        # termine integrale
        self.integral += error * self.dt

        # termine derivativo
        derivative = (error - self.prev_error) / self.dt

        # PID
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error

        # saturazione output [u_min, u_max]
        if u < self.u_min:
            u = self.u_min
        if u > self.u_max:
            u = self.u_max

        return u


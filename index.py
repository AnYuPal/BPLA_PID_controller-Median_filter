import math
import statistics

class GPS:
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    def get_coord(self):
        return {"x": self.x, "y": self.y, "z": self.z}

    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
def median_filter(data):
# Простая реализация медианного фильтра
# для сглаживания управляющих сигналов PID
    return statistics.median(data)

class Dron:
    def __init__(self, start_gps, name):
        self.gps = start_gps
        self.name = name
        self.speed = 40
        self.angular_speed = 5
        self.pid_controller_x = PIDController(kp=0.1, ki=0.01, kd=0.01)
        self.pid_controller_y = PIDController(kp=0.1, ki=0.01, kd=0.01)

    def move(self, coord_dest, dt):
        error_x = coord_dest.x - self.gps.x
        error_y = coord_dest.y - self.gps.y

        u_x = self.pid_controller_x.calculate(error_x, dt)
        u_y = self.pid_controller_y.calculate(error_y, dt)

        # Сглаживание управляющих сигналов с использованием медианного фильтра
        filtered_u_x = median_filter([u_x])
        filtered_u_y = median_filter([u_y])

        self.gps.x += filtered_u_x * self.speed * dt
        self.gps.y += filtered_u_y * self.speed * dt

# Пример использования
start_position = GPS(x=0, y=0, z=0)
destination = GPS(x=100, y=100, z=0)
drone = Dron(start_position, "MyDrone")

# Двигаться к координатам destination с временным шагом 0.1
for _ in range(100):
    drone.move(destination, 0.1)
    current_position = drone.gps.get_coord()
    print(f"Current Position: {current_position}")

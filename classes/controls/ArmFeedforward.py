import math


class ArmFeedforward:
    def __init__(self, s: float, g: float, v: float, a: float) -> None:
        self.s = s
        self.g = g
        self.v = v
        self.a = a
        self.previous_position = None
        self.max_output = None
        self.min_output = None

    def set_output_range(self, min: float, max: float):
        self.max_output = max
        self.min_output = min

    def signum(number: float):
        if number == 0:
            return 0
        elif number > 0:
            return 1
        else:
            return -1

    def calculate(self, position_radians: float, velocity_rad_per_sec: float, acceleration_rad_per_sec: float = None):
        if acceleration_rad_per_sec == None:
            return self.calculate(position_radians, velocity_rad_per_sec, 0)

        output = self.s * ArmFeedforward.signum(velocity_rad_per_sec) + self.g * math.cos(
            position_radians) + self.v * velocity_rad_per_sec + self.a * acceleration_rad_per_sec
        if self.max_output is not None or self.min_output is not None:
            if output > self.max_output:
                return self.max_output
            elif output < self.min_output:
                return self.min_output

        return output

    def max_achievable_velocity(self, max_voltage: float, angle: float, acceleration: float):
        return (max_voltage - self.s - math.cos(angle) * self.g - acceleration * self.a) / self.v

    def min_achievable_velocity(self, max_voltage: float, angle: float, acceleration: float):
        return (-max_voltage + self.s - math.cos(angle) * self.g - acceleration * self.a) / self.v

    def max_achievable_acceleration(self, max_voltage: float, angle: float, velocity: float):
        return (max_voltage - self.s * ArmFeedforward.signum(velocity) - math.cos(angle) * self.g - velocity * self.v) / self.a

    def min_achievable_acceleration(self, max_voltage: float, angle: float, velocity: float):
        return self.max_achievable_velocity(-max_voltage, angle, velocity)

    def auto_tune_g(self, current_position: float):
        if self.previous_position != None:
            delta = current_position - self.previous_position
            delta = round(delta, 4)
            if delta == 0:
                self.g += 10
            if current_position > 0:
                self.g -= 1

        self.previous_position = current_position

        return self.calculate(0, 0, 0)

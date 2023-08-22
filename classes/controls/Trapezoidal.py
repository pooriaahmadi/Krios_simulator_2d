from dataclasses import dataclass
import math


@dataclass
class State:
    position: float
    velocity: float

    def change_direction(self, direction: int):
        self.position *= direction
        self.velocity *= direction

        return self


@dataclass
class Constraints:
    max_velocity: float
    max_acceleration: float


class TrapezoidProfile:
    def __init__(self, constraints: Constraints, goal: State, initial: State = None) -> None:

        if initial is None:
            return TrapezoidProfile(constraints, goal, State(0, 0))

        self.initial = initial
        self.goal = goal

        self.constraints = constraints
        self.initial = initial.change_direction(self.__direction)
        self.goal = goal.change_direction(self.__direction)

        if self.initial.velocity > self.constraints.max_velocity:
            self.initial.velocity = self.constraints.max_velocity

        cutoff_begin = self.initial.velocity / self.constraints.max_acceleration
        cutoff_dist_begin = cutoff_begin * cutoff_begin * \
            self.constraints.max_acceleration / 2

        cutoff_end = self.goal.velocity / self.constraints.max_acceleration
        cutoff_dist_end = cutoff_end * cutoff_end * self.constraints.max_acceleration

        full_trapezoid_dist = cutoff_dist_begin + \
            (self.goal.position - self.initial.position) + cutoff_dist_end
        acceleration_time = self.constraints.max_velocity / \
            self.constraints.max_acceleration

        fullspeed_dist = full_trapezoid_dist - acceleration_time * \
            acceleration_time * self.constraints.max_acceleration

        if fullspeed_dist < 0:
            acceleration_time = math.sqrt(
                full_trapezoid_dist / self.constraints.max_acceleration)
            fullspeed_dist = 0

        self.__end_acceleration = acceleration_time - cutoff_begin
        self.__end_fullspeed = self.__end_acceleration + \
            fullspeed_dist / self.constraints.max_velocity
        self.__end_deccel = self.__end_fullspeed + acceleration_time - cutoff_end

    def calculate(self, t: float) -> State:
        result = State(self.initial.position, self.initial.velocity)\

        if t < self.__end_acceleration:
            result.velocity += t * self.constraints.max_acceleration
            result.position += (self.initial.velocity +
                                t * self.constraints.max_acceleration / 2) * t
        elif t < self.__end_fullspeed:
            result.velocity = self.constraints.max_velocity
            result.position += (self.initial.velocity + self.__end_acceleration * self.constraints.max_acceleration / 2) * \
                self.__end_acceleration + self.constraints.max_velocity * \
                (t - self.__end_acceleration)
        elif t <= self.__end_deccel:
            result.velocity = self.goal.velocity + \
                (self.__end_deccel - t) * self.constraints.max_acceleration
            time_left = self.__end_deccel - t
            result.position = self.goal.position - \
                (self.goal.velocity + time_left *
                 self.constraints.max_acceleration / 2.0) * time_left
        else:
            result = self.goal

        return result.change_direction(self.__direction)

    def time_left_until(self, target: float):
        position = self.initial.position * self.__direction
        velocity = self.initial.velocity * self.__direction

        end_acceleration = self.__end_acceleration * self.__direction
        end_fullspeed = self.__end_fullspeed * self.__direction - end_acceleration

        if target < position:
            end_acceleration = -end_acceleration
            end_fullspeed = -end_fullspeed
            velocity = -velocity

        end_acceleration = max([end_acceleration, 0])
        end_fullspeed = max([end_fullspeed, 0])

        acceleration = self.constraints.max_acceleration
        decceleration = -self.constraints.max_acceleration

        dist_to_target = abs(target - position)
        if dist_to_target < 1e-6:
            return 0

        acceleration_distance = velocity * end_acceleration + \
            0.5 * acceleration * math.pow(end_acceleration, 2)

        decceleration_velocity: float = None
        if end_acceleration > 0:
            decceleration_velocity = math.sqrt(
                abs(math.pow(velocity, 2) + 2 * acceleration * acceleration_distance))
        else:
            decceleration_velocity = velocity

        fullspeed_distance = self.constraints.max_velocity * end_fullspeed
        decceleration_distance: float = None

        if acceleration_distance > dist_to_target:
            acceleration_distance = dist_to_target
            fullspeed_distance = 0
            decceleration_distance = 0
        elif acceleration_distance + fullspeed_distance > dist_to_target:
            fullspeed_distance = dist_to_target - acceleration_distance
            decceleration_distance = 0
        else:
            decceleration_distance = dist_to_target - \
                fullspeed_distance - acceleration_distance

        acceleration_time = (-velocity + math.sqrt(abs(math.pow(velocity,
                             2) + 2 * acceleration * acceleration_distance))) / acceleration
        decceleration_time = (-decceleration_velocity + math.sqrt(math.pow(
            decceleration_velocity, 2) + 2 * decceleration * decceleration_distance)) / decceleration

        fullspeed_time = fullspeed_distance / self.constraints.max_velocity

        return acceleration_time + fullspeed_time + decceleration_time

    def is_finished(self, t: float):
        return t >= self.total_time

    @property
    def total_time(self):
        return self.__end_deccel

    @property
    def __direction(self):
        return -1 if self.initial.position > self.goal.position else 1

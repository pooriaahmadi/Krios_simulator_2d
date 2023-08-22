import pymunk
import math


class DoubleJointed:
    def __init__(self, arm_length: int, wrist_length: int) -> None:
        self.arm_length = arm_length
        self.wrist_length = wrist_length
        self.position = pymunk.Vec2d(0, 0)

    def calculate_angles(self):
        d = math.sqrt(math.pow(self.position.x, 2) +
                      math.pow(self.position.y, 2))

        # non intersecting or one circle within other
        if d > self.arm_length + self.wrist_length or d < abs(self.arm_length - self.wrist_length):
            return

        ## POSITIVE ##
        # Wrist angle
        q2 = (self.position.x**2 + self.position.y**2 - self.arm_length **
              2 - self.wrist_length**2) / (2 * self.arm_length * self.wrist_length)
        q2 = math.acos(q2)

        # Elbow angle
        q1 = math.atan(self.position.y / self.position.x) - math.atan(
            (self.wrist_length * math.sin(q2)) / (self.wrist_length * math.cos(q2) + self.arm_length))

        ## NEGATIVE ##
        # Elbow angle
        q1_different = math.atan(self.position.y / self.position.x) + math.atan(
            (self.wrist_length * math.sin(q2)) / (self.wrist_length * math.cos(q2) + self.arm_length))

        return (q1, q2), (q1_different, -q2)

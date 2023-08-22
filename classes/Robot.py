import pymunk
import arcade
import math
from .kinematics.DoubleJointed import DoubleJointed
from .controls import ArmFeedforward
from ._Graph import Graph
from .controls.Trapezoidal import TrapezoidProfile
from typing import Optional
from PID_Py.PID import PID
from .Timer import Timer


class Robot:
    def __init__(self, physics_engine: arcade.PymunkPhysicsEngine, chassis_width=150, wheel_radius=10, offset=pymunk.Vec2d(0, 0)) -> None:
        self.physics_engine = physics_engine
        self.robot_segments = arcade.SpriteList()
        self.chassis_width = chassis_width
        self.offset = offset
        self.wheel_radius = wheel_radius
        self.arm_length = 150
        self.wrist_length = 40
        self.ik = DoubleJointed(self.arm_length, self.wrist_length)

        self.arm_pid = PID(50000, 1000, 30000, indirectAction=False)
        self.arm_ff = ArmFeedforward(0, 65000, 0, 0)
        self.arm_target_angle = 0

        self.wrist_pid = PID(70000, 10000, 2000)
        self.wrist_ff = ArmFeedforward(0, 0, 0, 0)
        self.wrist_target_angle = 0

        self.graph = Graph(300, 500, 600, 200, "arm PID")
        self.graph.set_max(2)
        self.graph.set_min(-0.5)
        self.graph.add_parameter("Target", arcade.color.BLUE)
        self.graph.add_parameter("Actual", arcade.color.ORANGE)
        self.graph.add_parameter("Speed", arcade.color.PINK)

        self.motion: TrapezoidProfile = None
        self.timer = Timer()

        self.arm_body: Optional[arcade.PymunkPhysicsObject] = None
        self.wrist_body: Optional[arcade.PymunkPhysicsObject] = None
        self.arm_motor: Optional[pymunk.SimpleMotor] = None

        self.setup()

    def on_update(self):
        vel = 0
        if self.motion is not None:
            result = self.motion.calculate(
                self.timer.get_delta_sec())
            self.arm_target_angle = result.position
            vel = result.velocity

        pid_output = self.arm_pid.compute(
            self.arm_target_angle, self.arm_body.body.angle)
        ff_output = self.arm_ff.calculate(self.arm_body.body.angle, 0)

        self.arm_body.body.apply_force_at_local_point(
            (0, ff_output + pid_output), (self.arm_length / 2, 0))

        self.graph.append_data("Target", self.arm_target_angle)
        self.graph.append_data("Actual", self.arm_body.body.angle)
        self.graph.append_data("Speed", vel)

        # ff_output = self.wrist_ff.calculate()
        pid_output = self.wrist_pid.compute(
            self.wrist_target_angle, self.wrist_body.body.angle)

        self.wrist_body.body.apply_force_at_local_point(
            (0, pid_output), (self.wrist_length / 2, 0))

        # print(pid_output)

    def setup(self):
        self.create_chassis()
        self.move_endpoint(pymunk.Vec2d(190, 0))

    def move_endpoint(self, position: pymunk.Vec2d):
        self.ik.position = position
        results = self.ik.calculate_angles()
        if results is None:
            return

        self.arm_target_angle = results[1][0]
        self.wrist_target_angle = results[1][1] + results[1][0]

    def create_point(self, x, y):
        body = pymunk.Body()
        body.position = (x, y)
        body.body_type = pymunk.Body.STATIC
        self.physics_engine.space.add(body)

        return body

    def create_robot_segment(self, position: tuple, width: int, height: int, color: arcade.Color, mass: int):
        sprite = arcade.SpriteSolidColor(width, height, color)
        sprite.position = position

        self.robot_segments.append(sprite)
        self.physics_engine.add_sprite(sprite, mass, 1, 0)

        return sprite, self.physics_engine.get_physics_object(sprite)

    def create_wheel(self, position: tuple, radius: int, color: arcade.Color, mass: int):
        sprite = arcade.SpriteCircle(radius, color)
        sprite.position = position

        self.robot_segments.append(sprite)
        self.physics_engine.add_sprite(sprite, mass, 0.9)

        return sprite, self.physics_engine.get_physics_object(sprite)

    def create_chassis(self):

        left_wheel, left_wheel_object = self.create_wheel(
            (self.offset.x + self.wheel_radius, self.offset.y), self.wheel_radius, arcade.color.ALMOND, 1)

        right_wheel, right_wheel_object = self.create_wheel(
            (self.offset.x + self.chassis_width - self.wheel_radius, self.offset.y), self.wheel_radius, arcade.color.ALMOND, 1)

        chassis, chassis_object = self.create_robot_segment(
            (self.offset.x + self.chassis_width / 2, self.offset.y + 10), self.chassis_width, 20, arcade.color.ASH_GREY, 50)
        structure, structure_object = self.create_robot_segment(
            (self.offset.x + self.chassis_width / 2, self.offset.y + 20 + 70), 20, 140, arcade.color.ASH_GREY, 20)

        left_wheel_joint = pymunk.PivotJoint(
            chassis_object.body, left_wheel_object.body, (-self.chassis_width / 2 + self.wheel_radius, -20), (0, 0))
        left_wheel_joint.collide_bodies = False

        right_wheel_joint = pymunk.PivotJoint(
            chassis_object.body, right_wheel_object.body, (self.chassis_width / 2 - self.wheel_radius, -20), (0, 0))
        right_wheel_joint.collide_bodies = False

        structure_joint = pymunk.PivotJoint(
            chassis_object.body, structure_object.body, (0, 10), (0, -70))
        structure_gear_joint = pymunk.GearJoint(
            chassis_object.body, structure_object.body, 0, 1)
        structure_joint.collide_bodies = False
        structure_gear_joint.collide_bodies = False

        left_motor = pymunk.SimpleMotor(
            chassis_object.body, left_wheel_object.body, 0)
        right_motor = pymunk.SimpleMotor(
            chassis_object.body, right_wheel_object.body, 0)

        self.physics_engine.space.add(
            structure_joint, left_wheel_joint, left_motor, right_wheel_joint, right_motor, structure_gear_joint)

        self.create_arm(structure, structure_object)

    def create_arm(self, structure: arcade.SpriteSolidColor, structure_object: arcade.PymunkPhysicsObject):

        shoulder, shoulder_body = self.create_robot_segment(
            (structure.position[0] + self.arm_length / 2, structure.position[1] + structure.height / 2), self.arm_length, 20, arcade.color.CRIMSON, 10)

        wrist, wrist_body = self.create_robot_segment(
            (shoulder.position[0] + self.arm_length / 2 + self.wrist_length / 2, shoulder.position[1]), self.wrist_length, 10, arcade.color.AMAZON, 3)

        shoulder_pivot_joint = pymunk.PinJoint(
            structure_object.body, shoulder_body.body, (0, structure.height / 2), (-self.arm_length / 2, 0))
        shoulder_pivot_joint.collide_bodies = False
        shoulder_motor = pymunk.SimpleMotor(
            structure_object.body, shoulder_body.body, 0)
        shoulder_motor.max_force = 3000000

        wrist_pivot_joint = pymunk.PinJoint(
            shoulder_body.body, wrist_body.body, (self.arm_length / 2, 0), (-self.wrist_length / 2, 0))

        wrist_pivot_joint.collide_bodies = False

        wrist_motor = pymunk.SimpleMotor(
            shoulder_body.body, wrist_body.body, 0)

        self.physics_engine.space.add(
            shoulder_pivot_joint, wrist_pivot_joint)

        self.arm_body = shoulder_body
        self.arm_motor = shoulder_motor
        self.wrist_body = wrist_body
        self.wrist_motor = wrist_motor

    def draw(self):
        self.robot_segments.draw()
        self.graph.draw()

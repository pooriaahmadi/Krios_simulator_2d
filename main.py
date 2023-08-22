import math
import arcade
import pymunk
from typing import Optional, List
from classes import Robot
from classes.controls.Trapezoidal import State, Constraints, TrapezoidProfile


SCREEN_TITLE = "pymunk simlation"
WIDTH = 800
HEIGHT = 608


class GameWindow(arcade.Window):
    """Main Window"""

    def __init__(self, width, height, title):
        super().__init__(width, height, title)
        arcade.set_background_color(arcade.color.WHITE)
        self.ball_list: Optional[arcade.SpriteList] = None
        self.robot_segments: Optional[arcade.SpriteList] = None
        self.wall_list: Optional[arcade.SpriteList] = None
        self.joints: Optional[List[pymunk.Constraint]] = None
        self.physics_engine: Optional[arcade.PymunkPhysicsEngine] = None

        self.robot: Optional[Robot] = None

        # Mouse stuff
        self.mouse_position: Optional[tuple] = None
        self.mouse_left_click: Optional[bool] = None
        self.mouse_ball: Optional[arcade.SpriteCircle] = None

    def setup(self):
        """ Set up everything with the game """
        self.physics_engine = arcade.PymunkPhysicsEngine((0, -980))
        self.ball_list = arcade.SpriteList()
        self.robot_segments = arcade.SpriteList()
        self.wall_list = arcade.SpriteList()
        self.robot = Robot(self.physics_engine, offset=pymunk.Vec2d(200, 100))
        self.joints = []
        self.mouse_position = (0, 0)
        self.mouse_left_click = False

        # self.create_arm()
        self.create_boundaries(10)

    def create_robot_segment(self, position: tuple, width: int, height: int, color: arcade.Color, mass: int):
        sprite = arcade.sprite.SpriteSolidColor(width, height, color)
        sprite.position = position

        self.robot_segments.append(sprite)
        self.physics_engine.add_sprite(sprite, mass, 1, 0)
        return sprite, self.physics_engine.get_physics_object(sprite)

    def on_mouse_press(self, x: int, y: int, button: int, modifiers: int):
        if button == 1:
            self.mouse_ball = self.create_ball(x, y, 20, 10, True)
            self.mouse_left_click = True

    def on_mouse_motion(self, x: int, y: int, dx: int, dy: int):
        self.mouse_position = (x, y)

    def on_mouse_release(self, x: int, y: int, button: int, modifiers: int):
        if button == 1:
            self.mouse_left_click = False

            angle = self.calculate_angle([self.mouse_ball.center_x,
                                          self.mouse_ball.center_y], self.mouse_position)
            force = self.calculate_distance([self.mouse_ball.center_x,
                                             self.mouse_ball.center_y], self.mouse_position) * 20

            fx = math.cos(angle) * force
            fy = math.sin(angle) * force
            self.make_ball_dynamic(self.mouse_ball, 10)
            self.physics_engine.apply_impulse(self.mouse_ball, (fx, fy))
            self.mouse_ball = None

    def on_key_press(self, key, modifiers):
        """Called whenever a key is pressed. """
        pass

    def on_key_release(self, key, modifiers):
        """Called when the user releases a key. """
        pass

    def on_update(self, delta_time):
        """ Movement and game logic """
        for _ in range(10):
            self.physics_engine.step(delta_time / 10)

        # self.robot.move_endpoint(pymunk.Vec2d(
        #     self.robot.ik.position.x-0.5, self.robot.ik.position.y+0.5))

        self.robot.on_update()

        # self.physics_engine.resync_sprites()

    def on_draw(self):
        """ Draw everything """
        self.clear()
        self.ball_list.draw()
        self.wall_list.draw()
        self.robot_segments.draw()
        self.draw_line()
        self.robot.draw()

        for joint in self.joints:
            color = arcade.color.BLUE
            arcade.draw_line(joint.a.position.x, joint.a.position.y,
                             joint.b.position.x, joint.b.position.y, color, 3)

    def create_ball(self, x, y, radius, mass, static=False, color=arcade.color.CRIMSON):
        ball = arcade.SpriteCircle(radius, color, False)
        ball.set_position(x, y)
        self.ball_list.append(ball)
        if not static:
            self.make_ball_dynamic(ball, mass)
        return ball

    def make_ball_dynamic(self, ball: arcade.SpriteCircle, mass):
        self.physics_engine.add_sprite(
            sprite=ball,
            mass=mass,
            elasticity=0.9,
            friction=0.4,
            body_type=arcade.PymunkPhysicsEngine.DYNAMIC
        )

    def create_boundaries(self, offset: int):
        positions = [
            [(WIDTH / 2, offset / 2), (WIDTH, offset)],
            [(offset / 2, HEIGHT / 2), (offset, HEIGHT)],
            [(WIDTH - offset / 2, HEIGHT / 2), (offset, HEIGHT)],
            [(WIDTH / 2, HEIGHT - offset / 2), (WIDTH, offset)]
        ]
        for pos, size in positions:
            wall = arcade.SpriteSolidColor(
                size[0], size[1], arcade.color.BLACK)
            wall.position = pos
            self.wall_list.append(wall)

        self.physics_engine.add_sprite_list(
            self.wall_list,
            body_type=arcade.PymunkPhysicsEngine.STATIC,
            elasticity=0.4,
            friction=0.5
        )

    def draw_line(self):
        if self.mouse_left_click:
            arcade.draw_line(self.mouse_ball.center_x, self.mouse_ball.center_y,
                             self.mouse_position[0], self.mouse_position[1], arcade.color.BLACK, 2)
            self.robot.motion = TrapezoidProfile(
                Constraints(2, 1), State(math.pi / 2, 0), State(self.robot.arm_body.body.angle, 0))
            self.robot.timer.start()

    def calculate_distance(self, p1, p2):
        return math.sqrt((p2[1] - p1[1]) ** 2 + (p2[0] - p1[0]) ** 2)

    def calculate_angle(self, p1, p2):
        return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def main():
    window = GameWindow(WIDTH, HEIGHT, SCREEN_TITLE)
    window.setup()
    arcade.run()


if __name__ == "__main__":
    main()

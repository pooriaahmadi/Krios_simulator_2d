import arcade
from typing import List, Dict


class Parameter:
    def __init__(self, label: str, color: arcade.Color) -> None:
        self.data: List[float] = []
        self.label = label
        self.color = color


class Graph:
    def __init__(self, center_x: int, center_y: int, width: int, height: int, label: str) -> None:
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.label = label
        self.min: float = 0
        self.max: float = 0
        self.duration = 8
        self.FPS = 60
        self.auto_zoom = True

        self.parameters: Dict[str, Parameter] = {}

    def set_min(self, min: float):
        self.auto_zoom = False
        self.min = min

    def set_max(self, max: float):
        self.auto_zoom = False
        self.max = max

    def set_duration(self, duration: int):
        self.duration = duration

    def add_parameter(self, parameter: str, color: arcade.Color):
        self.parameters[parameter] = Parameter(parameter, color)

    def append_data(self, parameter: str, data: float):
        self.parameters[parameter].data.append(data)
        if self.auto_zoom:
            if data > self.max:
                self.max = data
            elif data < self.min:
                self.min = data

        if len(self.parameters[parameter].data) > self.FPS * self.duration:
            self.parameters[parameter].data.pop(0)

    def update(self):
        pass

    def draw(self):

        # Background
        arcade.draw_rectangle_filled(
            self.center_x, self.center_y, self.width, self.height, arcade.color.BLACK)

        # Label
        arcade.draw_text(self.label, self.center_x,
                         self.center_y - self.height / 2 + 5, arcade.color.WHITE, 12, anchor_x="center", anchor_y="baseline")

        # Graph numbers
        arcade.draw_text(round(self.min, 1), self.center_x - self.width / 2 + 5, self.center_y - self.height / 2 + 20, arcade.color.WHITE, 12,
                         anchor_x="left", anchor_y="baseline")
        arcade.draw_text(round(self.max, 1), self.center_x - self.width / 2 + 5, self.center_y + self.height / 2 - 5, arcade.color.WHITE, 12,
                         anchor_x="left", anchor_y="top")

        # Border
        offset = 50
        arcade.draw_rectangle_outline(
            self.center_x + 20, self.center_y + 10, self.width - offset, self.height - 40, arcade.color.GOLD, 1)

        # Graph data
        spacing_x = (self.width - offset) / (self.FPS * self.duration)

        delta = self.max - self.min
        for parameter in self.parameters.values():
            point_list = []
            for index, value in enumerate(parameter.data):
                y = ((value - self.min) / delta)
                y = (self.height - 40) * y
                point_list.append(
                    [index * spacing_x + ((self.center_x + 20) - ((self.width - offset) / 2)), y + self.center_y + 10 - (self.height - 40) / 2])
            arcade.draw_line_strip(point_list, parameter.color, 1)

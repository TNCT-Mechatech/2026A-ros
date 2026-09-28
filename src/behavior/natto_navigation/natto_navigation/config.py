"""Configuration and button edge handling, independent of ROS."""
import math

import yaml


def read_yaml(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping')
    return data


def load_destinations(path):
    data = read_yaml(path)
    frame = data.get('frame_id', 'map')
    if frame != 'map':
        raise ValueError('destinations must use the map frame')
    destinations = data.get('destinations', {})
    if not isinstance(destinations, dict) or not destinations:
        raise ValueError('at least one destination is required')
    for name, pose in destinations.items():
        if not isinstance(name, str) or not name or not isinstance(pose, dict):
            raise ValueError('invalid destination entry')
        for key in ('x', 'y', 'yaw_deg'):
            value = pose.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name}.{key} must be a finite number')
    return frame, destinations


def load_bindings(path, enable_button):
    bindings = {}
    for key, destination in read_yaml(path).get('bindings', {}).items():
        if not isinstance(key, str) or not key.startswith('button_'):
            raise ValueError(f'invalid button: {key}')
        index = int(key.removeprefix('button_'))
        if index < 0 or index == enable_button or index in bindings:
            raise ValueError(f'invalid or reserved button: {key}')
        if not isinstance(destination, str) or not destination:
            raise ValueError(f'invalid destination for {key}')
        bindings[index] = destination
    if not bindings:
        raise ValueError('at least one binding is required')
    return bindings


class ButtonEdges:
    def __init__(self, bindings, enable_button=16):
        self.bindings = bindings
        self.enable_button = enable_button
        self.previous = None

    def reset(self):
        self.previous = None

    def sample(self, buttons):
        required = max(self.enable_button, *self.bindings)
        if len(buttons) <= required:
            self.reset()
            return False, False, ''
        enabled = buttons[self.enable_button] == 1
        held = [index for index in self.bindings if buttons[index] == 1]
        destination = ''
        if self.previous is not None and enabled and len(held) == 1:
            index = held[0]
            if self.previous[index] != 1:
                destination = self.bindings[index]
        self.previous = list(buttons)
        return True, enabled, destination

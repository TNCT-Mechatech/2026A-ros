"""Lifecycle regressions: cancellation, stale results, input edges and timeouts."""
import copy
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from natto_msgs.msg import NavigationInput, NavigationFeedback, NavigationPath
from rclpy.parameter import Parameter

from natto_navigation.config import ButtonEdges, load_destinations, load_bindings
from natto_navigation.navigation_manager import NavigationManager


class RecordedPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


def buttons(*pressed):
    result = [0] * 17
    for i in pressed:
        result[i] = 1
    return result


def test_edge_hold_release_and_simultaneous_buttons():
    edges = ButtonEdges({0: 'flag', 1: 'home'})
    assert edges.sample(buttons(16, 0))[2] == ''  # held at startup
    edges.sample(buttons(16))
    assert edges.sample(buttons(16, 0))[2] == 'flag'
    assert edges.sample(buttons(16, 0))[2] == ''
    edges.sample(buttons())
    assert edges.sample(buttons(0))[2] == ''  # permission off: no queue
    assert edges.sample(buttons(16, 0))[2] == ''
    edges.sample(buttons(16))
    assert edges.sample(buttons(16, 0, 1))[2] == ''
    assert edges.sample(buttons(16, 1))[2] == ''  # releasing one is not an edge
    edges.sample(buttons(16))
    assert edges.sample(buttons(16, 1))[2] == 'home'


def test_reconnection_and_short_joy_require_new_edge():
    edges = ButtonEdges({0: 'flag', 1: 'home'})
    edges.sample(buttons())
    assert edges.sample(buttons(16, 0))[2] == 'flag'
    edges.reset()
    assert edges.sample(buttons(16, 0))[2] == ''
    assert edges.sample([1, 0]) == (False, False, '')
    assert edges.sample(buttons(16, 0))[2] == ''


def test_configuration_validation(tmp_path):
    path = tmp_path / 'invalid.yaml'
    path.write_text('destinations: {flag: {x: .nan, y: 0, yaw_deg: 0}}')
    with pytest.raises(ValueError):
        load_destinations(path)
    path.write_text('bindings: {button_16: flag}')
    with pytest.raises(ValueError):
        load_bindings(path, 16)


@pytest.fixture
def manager(tmp_path):
    path = tmp_path / 'destinations.yaml'
    path.write_text('frame_id: map\ndestinations:\n  flag: {x: 2.0, y: 1.0, yaw_deg: 90.0}\n  home: {x: 1.0, y: 1.0, yaw_deg: 0.0}\n')
    rclpy.init()
    node = NavigationManager(parameter_overrides=[Parameter('destinations_path', value=str(path))])
    for name in ('goal_pub', 'command_pub', 'allow_pub', 'status_pub'):
        setattr(node, name, RecordedPublisher())
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    node.on_pose(pose)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def select(node, destination='flag', enabled=True, connected=True):
    node.on_input(NavigationInput(connected=connected, enabled=enabled, destination=destination))
    return node.request_id


def feedback(request, speed=0.2, reached=False):
    command = TwistStamped()
    command.twist.linear.x = speed
    return NavigationFeedback(request_id=request, command=command, reached=reached)


def assert_stopped(node):
    node.tick()
    command = node.command_pub.messages[-1].twist
    assert (command.linear.x, command.linear.y, command.angular.z) == (0.0, 0.0, 0.0)


def test_switch_ignores_old_velocity_arrival_and_path_failure(manager):
    first = select(manager)
    manager.on_feedback(feedback(first))
    manager.tick()
    assert manager.command_pub.messages[-1].twist.linear.x == 0.2
    second = select(manager, 'home')
    assert second != first
    assert manager.goal_pub.messages[-2].cancel
    assert manager.goal_pub.messages[-2].request_id == first
    manager.on_feedback(feedback(first, reached=True))
    manager.on_path(NavigationPath(request_id=first, valid=False, reason='old failure'))
    assert manager.state == 'planning'
    assert manager.request_id == second
    assert_stopped(manager)
    manager.on_feedback(feedback(second))
    assert manager.state == 'moving'


def test_release_does_not_resume_until_new_destination(manager):
    first = select(manager)
    select(manager, '', enabled=False)
    assert manager.state == 'canceled'
    assert not manager.request_id
    select(manager, '')
    manager.on_feedback(feedback(first))
    assert_stopped(manager)
    assert not manager.request_id
    assert select(manager) != first


def test_same_destination_no_restart_and_arrival_cancels_planner(manager):
    request = select(manager)
    assert select(manager) == request
    assert len(manager.goal_pub.messages) == 1
    manager.on_feedback(feedback(request, reached=True))
    assert manager.state == 'arrived'
    assert manager.goal_pub.messages[-1].cancel
    assert not manager.request_id
    assert_stopped(manager)
    assert select(manager) != request


def test_missing_destination_keeps_current_request(manager):
    request = select(manager)
    select(manager, 'not_configured')
    assert manager.request_id == request


def test_failed_plan_stops_and_cancels(manager):
    request = select(manager)
    manager.on_path(NavigationPath(request_id=request, valid=False, reason='blocked'))
    assert manager.state == 'failed'
    assert manager.status_pub.messages[-1].reason == 'blocked'
    assert manager.goal_pub.messages[-1].cancel
    manager.on_feedback(feedback(request))
    assert_stopped(manager)


@pytest.mark.parametrize('failure', ['input', 'pose', 'plan', 'feedback', 'travel'])
def test_timeouts_cancel_without_automatic_restart(manager, failure):
    request = select(manager)
    now = time.monotonic()
    if failure == 'input':
        manager.last_input = now - manager.input_timeout - 1
    elif failure == 'pose':
        manager.last_pose = now - manager.pose_timeout - 1
    elif failure == 'plan':
        manager.started = now - manager.plan_timeout - 1
    elif failure == 'travel':
        manager.started = now - manager.travel_timeout - 1
    else:
        manager.on_feedback(feedback(request))
        manager.last_feedback = now - manager.feedback_timeout - 1
    assert_stopped(manager)
    assert not manager.request_id
    assert manager.goal_pub.messages[-1].cancel
    select(manager, '')
    manager.on_feedback(feedback(request))
    assert not manager.request_id


def test_disconnection_revokes_permission(manager):
    select(manager)
    select(manager, '', connected=False)
    assert not manager.enabled
    assert not manager.allow_pub.messages[-1].data
    assert_stopped(manager)

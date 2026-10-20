"""ROS integration regression tests for empty JointState publishers.

Build/source natto_joy, natto_state_machine and natto_chassis_calculator first.
Run without robot nodes, in a separate local ROS domain:
  ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1 python3 test_joint_state_publish.py
"""

import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, TwistStamped
from natto_msgs.msg import StateAction, StateResult
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster


class JointStatePublishTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '10'):
            raise RuntimeError('Use a separate test ROS_DOMAIN_ID, e.g. 211')
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Set ROS_LOCALHOST_ONLY=1 for these local tests')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.namespace = f'/joint_state_test_{os.getpid()}/{self._testMethodName}'
        self.node = rclpy.create_node('probe', namespace=self.namespace)
        self.processes = []
        self.subscriptions = []
        self.logs = tempfile.TemporaryDirectory(prefix='joint-state-test-')
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for process, log in self.processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            log.close()
        self.node.destroy_node()
        self.logs.cleanup()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.01, end - time.monotonic()))

    def wait_for(self, predicate, timeout=10):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            for process, log in self.processes:
                if process.poll() is not None:
                    log.flush()
                    self.fail(Path(log.name).read_text())
            self.spin(0.01)
        self.assertTrue(predicate(), 'Timed out waiting for ROS communication')

    def start(self, package, executable, parameters=None):
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        args = [str(binary), '--ros-args', '-r', f'__ns:={self.namespace}']
        for name, value in (parameters or {}).items():
            args.extend(['-p', f'{name}:={value}'])
        log = open(Path(self.logs.name) / f'{executable}.log', 'w')
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))

    def receive(self, message_type, topic, qos=10):
        messages = []
        subscription = self.node.create_subscription(
            message_type, topic, messages.append, qos)
        self.subscriptions.append(subscription)
        return messages

    def publisher(self, message_type, topic):
        publisher = self.node.create_publisher(message_type, topic, 10)
        self.wait_for(lambda: publisher.get_subscription_count() > 0)
        self.spin(0.1)
        return publisher

    def wait_for_outputs(self):
        self.wait_for(lambda: all(sub.get_publisher_count() > 0
                                  for sub in self.subscriptions))
        self.spin(0.2)

    def button_manager(self, parameters):
        self.joints = self.receive(JointState, 'joint_states', qos_profile_sensor_data)
        self.allow = self.receive(Bool, 'allow_auto_drive')
        self.power = self.receive(Bool, 'power')
        self.origins = self.receive(String, 'get_origin_joint_name')
        self.start('natto_joy', 'button_manager', parameters)
        self.joy = self.publisher(Joy, 'joy')
        self.wait_for_outputs()

    def send_joy(self, buttons, axes=None):
        previous = len(self.allow)
        self.joy.publish(Joy(buttons=buttons, axes=axes or [0.0] * 6))
        self.wait_for(lambda: len(self.allow) == previous + 1)
        self.spin(0.05)

    def test_zero_joints_preserves_other_functions_and_high_frequency(self):
        self.button_manager({
            'num_button': 3,
            'button_0.function': 'allow_auto_drive', 'button_0.mode': 'hold',
            'button_1.function': 'power', 'button_1.mode': 'toggle',
            'button_2.function': 'get_origin', 'button_2.mode': 'positive_edge',
            'button_2.joint_name': 'origin_target',
        })
        self.send_joy([0, 1, 1])
        self.assertFalse(self.allow[-1].data)
        self.assertTrue(self.power[-1].data)
        self.assertEqual([msg.data for msg in self.origins], ['origin_target'])
        self.send_joy([1, 1, 1])
        self.assertTrue(self.allow[-1].data)
        self.assertTrue(self.power[-1].data)
        self.assertEqual(len(self.origins), 1)
        self.send_joy([0, 0, 0])
        self.send_joy([0, 1, 1])
        self.assertFalse(self.power[-1].data)
        self.assertEqual(len(self.origins), 2)
        previous = len(self.allow)
        for i in range(100):
            self.joy.publish(Joy(buttons=[i % 2, 0, 0], axes=[0.0] * 6))
            self.spin(0.005)  # approximately 200 Hz
        self.wait_for(lambda: len(self.allow) == previous + 100)
        self.spin(0.2)
        self.assertEqual(self.joints, [])

    def test_manual_joint_values_and_one_message_per_joy(self):
        self.button_manager({
            'num_button': 2,
            'button_0.function': 'joint_position', 'button_0.mode': 'hold',
            'button_0.joint_name': 'arm', 'button_0.position_on': 2.5,
            'button_0.position_off': -0.5,
            'button_1.function': 'joint_speed', 'button_1.mode': 'hold',
            'button_1.joint_name': 'roller', 'button_1.speed_on': 3.0,
            'button_1.speed_off': -1.0,
        })
        for i in range(20):
            pressed = i % 2
            self.send_joy([pressed, pressed])
            self.assertEqual(len(self.joints), i + 1)
            msg = self.joints[-1]
            self.assertEqual(msg.name, ['arm', 'roller'])
            self.assertEqual(list(msg.position), [2.5 if pressed else -0.5, 0.0])
            self.assertEqual(list(msg.velocity), [0.0, 3.0 if pressed else -1.0])
            self.assertGreater(msg.header.stamp.sec, 0)
        self.assertEqual(self.power, [])  # no assigned power button

    def test_auto_without_always_and_toggle_edges(self):
        self.button_manager({
            'num_button': 2,
            'button_0.function': 'allow_auto_drive', 'button_0.mode': 'toggle',
            'button_1.function': 'joint_position', 'button_1.mode': 'hold',
            'button_1.joint_name': 'arm', 'button_1.position_on': 2.5,
        })
        self.send_joy([0, 0])
        self.assertEqual(len(self.joints), 1)
        for buttons in ([1, 1], [1, 0], [0, 1]):
            self.send_joy(buttons)
            self.assertTrue(self.allow[-1].data)
            self.assertEqual(len(self.joints), 1)
        self.send_joy([1, 1])
        self.assertFalse(self.allow[-1].data)
        self.assertEqual(len(self.joints), 2)
        self.assertEqual(list(self.joints[-1].position), [2.5])

    def test_auto_always_subset_and_trigger_updates(self):
        self.button_manager({
            'num_button': 2,
            'button_0.function': 'allow_auto_drive', 'button_0.mode': 'hold',
            'button_1.function': 'joint_position', 'button_1.mode': 'hold',
            'button_1.joint_name': 'arm', 'button_1.position_on': 2.5,
            'zr.function': 'joint_speed', 'zr.mode': 'hold',
            'zr.joint_name': 'roller', 'zr.speed_on': 4.0,
            'zr.publish_always': True,
        })
        self.send_joy([0, 1])
        self.assertEqual(self.joints[-1].name, ['arm', 'roller'])
        for i, axis in enumerate([0.25, -0.5, 0.0]):
            self.send_joy([1, 1], [0.0] * 5 + [axis])
            self.assertEqual(len(self.joints), i + 2)
            self.assertEqual(self.joints[-1].name, ['roller'])
            self.assertEqual(list(self.joints[-1].velocity), [-axis * 4.0])
            self.assertEqual(self.joints[-1].header.stamp.sec, 0)
        self.send_joy([0, 0])
        self.assertEqual(self.joints[-1].name, ['arm', 'roller'])
        self.assertEqual(list(self.joints[-1].position), [0.0, 0.0])

    def test_auto_always_button_shared_slot_and_high_frequency(self):
        self.button_manager({
            'num_button': 3,
            'button_0.function': 'allow_auto_drive', 'button_0.mode': 'hold',
            'button_1.functions': '[joint_position, joint_speed]',
            'button_1.joint_names': '[arm, arm]', 'button_1.mode': 'hold',
            'button_1.position_on': 2.5, 'button_1.speed_on': 3.0,
            'button_1.publish_always_list': '[false, true]',
            'button_2.function': 'joint_speed', 'button_2.mode': 'hold',
            'button_2.joint_name': 'manual_roller',
        })
        self.send_joy([0, 1, 1])
        self.assertEqual(self.joints[-1].name, ['arm', 'manual_roller'])
        self.send_joy([1, 1, 1])
        self.assertEqual(self.joints[-1].name, ['arm'])
        self.assertEqual(list(self.joints[-1].position), [2.5])
        self.assertEqual(list(self.joints[-1].velocity), [3.0])
        previous_joints, previous_allow = len(self.joints), len(self.allow)
        for i in range(100):
            self.joy.publish(Joy(buttons=[1, i % 2, 1], axes=[0.0] * 6))
            self.spin(0.005)
        self.wait_for(lambda: len(self.allow) == previous_allow + 100)
        self.wait_for(lambda: len(self.joints) == previous_joints + 100)
        self.assertTrue(all(msg.name == ['arm'] for msg in self.joints[previous_joints:]))
        self.assertEqual(list(self.joints[-1].position), [2.5])
        self.assertEqual(list(self.joints[-1].velocity), [3.0])

    def test_behavior_empty_normal_commands_switching_and_wait(self):
        joints = self.receive(JointState, 'command_joint_states', qos_profile_sensor_data)
        results = self.receive(StateResult, 'state_result')
        self.start('natto_state_machine', 'default_action',
                   {'initial_allow_auto_drive': True})
        actions = self.publisher(StateAction, 'state_action')
        allow = self.publisher(Bool, 'allow_auto_drive')
        self.wait_for_outputs()
        self.spin(0.35)
        self.assertEqual(joints, [])
        actions.publish(StateAction(state_id=1, action_name='set_joint_position',
                                    arguments_names=['arm'], arguments_values=['1.25']))
        self.wait_for(lambda: len(joints) >= 3)
        for msg in joints:
            self.assertEqual(msg.name, ['arm'])
            self.assertEqual(list(msg.position), [1.25])
            self.assertGreater(msg.header.stamp.sec, 0)
        allow.publish(Bool(data=False))
        self.spin(0.2)
        previous = len(joints)
        self.spin(0.3)
        self.assertEqual(len(joints), previous)
        allow.publish(Bool(data=True))
        self.wait_for(lambda: len(joints) > previous)
        actions.publish(StateAction(state_id=2, action_name='set_joint_velocity',
                                    arguments_names=['roller'], arguments_values=['3.5']))
        self.wait_for(lambda: joints[-1].name == ['roller'])
        self.assertEqual(list(joints[-1].velocity), [3.5])
        self.wait_for(lambda: any(msg.state_id == 2 and msg.success for msg in results))
        actions.publish(StateAction(state_id=3, action_name='set_joint_velocity'))
        self.wait_for(lambda: any(msg.state_id == 3 for msg in results))
        self.spin(0.15)
        previous = len(joints)
        actions.publish(StateAction(state_id=4, action_name='wait',
                                    arguments_names=['duration_sec'], arguments_values=['0.1']))
        self.wait_for(lambda: any(msg.state_id == 4 and msg.success for msg in results))
        self.spin(0.3)
        self.assertEqual(len(joints), previous)
        self.assertTrue(all(msg.name for msg in joints))

    def test_mecanum_wheel_commands_with_feedback(self):
        names = ['fl', 'fr', 'rl', 'rr']
        bases = [name + '_base' for name in names]
        joints = self.receive(JointState, 'command_joint_states', qos_profile_sensor_data)
        self.start('natto_chassis_calculator', 'chassis_calculator', {
            'chassis_type': 'mecanum', 'wheel_radius': 0.076,
            'wheel_names': str(names), 'wheel_base_names': str(bases),
        })
        broadcaster = StaticTransformBroadcaster(self.node)
        transforms = []
        for base, (x, y) in zip(bases, [(0.2, 0.15), (0.2, -0.15),
                                      (-0.2, 0.15), (-0.2, -0.15)]):
            transform = TransformStamped()
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id = 'command/base_link'
            transform.child_frame_id = f'command/{base}_link'
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        broadcaster.sendTransform(transforms)
        velocity = self.publisher(TwistStamped, 'command_velocity')
        feedback = self.publisher(JointState, 'joint_states')
        self.wait_for_outputs()
        self.spin(0.3)
        for x, y, z in [(0.5, 0.0, 0.0), (0.0, 0.5, 0.0),
                        (0.0, 0.0, 0.5), (0.0, 0.0, 0.0)]:
            feedback.publish(JointState(name=names, position=[1.0] * 4, velocity=[2.0] * 4))
            command = TwistStamped()
            command.twist.linear.x = x
            command.twist.linear.y = y
            command.twist.angular.z = z
            previous = len(joints)
            velocity.publish(command)
            self.wait_for(lambda: len(joints) == previous + 1)
            self.assertEqual(joints[-1].name, names)
            expected = [x - y - 0.35 * z, x + y + 0.35 * z,
                        x + y - 0.35 * z, x - y + 0.35 * z]
            for actual, linear in zip(joints[-1].velocity, expected):
                self.assertAlmostEqual(actual, linear / 0.076)


if __name__ == '__main__':
    unittest.main(verbosity=2)

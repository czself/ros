#!/usr/bin/env python3
"""Drive two fixed Gazebo traffic lights without deleting them on a change."""

import math
import pathlib
import re
import time

import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SetLinkState, SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32, String


LIGHTS = (
    ('traffic_light_1', -1.109649420, 0.066644743, 0.035, 1.526553365),
    ('traffic_light_2', 1.639769554, 0.329708397, 0.035, 0.049170261),
)
COLOURS = ('RED', 'GREEN', 'YELLOW')
FRONT_Y = -0.018
HIDDEN_Y = 0.0
PANEL_Z = 0.410
LOCAL_X = {'RED': -0.210, 'YELLOW': 0.0, 'GREEN': 0.210}
LEGACY_NAMES = tuple(
    f'{base_name}_{colour.lower()}'
    for base_name, *_ in LIGHTS
    for colour in COLOURS
)


def pose_at(x, y, z, yaw):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class TrafficLightController:
    def __init__(self):
        self.cycle = (
            ('RED', float(rospy.get_param('~red_duration', 10.0))),
            ('GREEN', float(rospy.get_param('~green_duration', 15.0))),
            ('YELLOW', float(rospy.get_param('~yellow_duration', 3.0))),
        )
        if any(duration <= 0.0 for _, duration in self.cycle):
            raise ValueError('all traffic-light durations must be greater than zero')
        template_path = pathlib.Path(
            rospy.get_param('~template_path', '/root/traffic_light/model.sdf')
        )
        self.template = re.sub(
            r'<collision name="[^"]+">.*?</collision>', '',
            template_path.read_text(encoding='utf-8'), flags=re.S)
        if '__MODEL_NAME__' not in self.template:
            raise ValueError('traffic-light template missing __MODEL_NAME__')

        self.state_pub = rospy.Publisher('/traffic_light/state', String, queue_size=1, latch=True)
        self.remaining_pub = rospy.Publisher(
            '/traffic_light/time_remaining', Float32, queue_size=1, latch=True
        )
        self.ready_pub = rospy.Publisher('/traffic_light/ready', Bool, queue_size=1, latch=True)
        self.ready_pub.publish(False)
        self.spawn_model = None
        self.delete_model = None
        self.get_world = None
        self.set_link_state = None
        self.displayed_state = None

    def connect(self):
        self.ready_pub.publish(False)
        while not rospy.is_shutdown():
            try:
                for service in (
                    '/gazebo/spawn_sdf_model', '/gazebo/delete_model',
                    '/gazebo/get_world_properties', '/gazebo/set_link_state'
                ):
                    rospy.wait_for_service(service, timeout=2.0)
                self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                self.get_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
                self.set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
                self.get_world()
                rospy.loginfo('Gazebo traffic-light services are ready')
                return True
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn_throttle(5.0, 'waiting for Gazebo services: %s', exc)
                time.sleep(1.0)
        return False

    def delete_if_present(self, model_name, present):
        if model_name not in present:
            return True
        try:
            response = self.delete_model(model_name)
        except rospy.ServiceException as exc:
            rospy.logerr('delete service disconnected for %s: %s', model_name, exc)
            return False
        if not response.success:
            rospy.logerr('delete failed for %s: %s', model_name, response.status_message)
        return response.success

    def create_fixed_models(self):
        """Recreate only at controller startup, never at a colour transition."""
        try:
            present = set(self.get_world().model_names)
        except rospy.ServiceException as exc:
            rospy.logerr('world service disconnected: %s', exc)
            return False
        names = [name for name, *_ in LIGHTS] + list(LEGACY_NAMES)
        if not all(self.delete_if_present(name, present) for name in names):
            return False
        for model_name, x, y, z, yaw in LIGHTS:
            try:
                response = self.spawn_model(
                    model_name,
                    self.template.replace('__MODEL_NAME__', model_name),
                    '', pose_at(x, y, z, yaw), 'world'
                )
            except rospy.ServiceException as exc:
                rospy.logerr('spawn service disconnected for %s: %s', model_name, exc)
                return False
            if not response.success:
                rospy.logerr('spawn failed for %s: %s', model_name, response.status_message)
                return False
        self.displayed_state = None
        return True

    def template_for_state(self, active_state, model_name):
        """Build a static lamp with only its active coloured face in front.

        Static SDF links cannot reliably be moved by SetLinkState.  Instead
        each state is a complete fixed model: its active `*_on` plate sits at
        the front y plane and all other `*_off` plates do.  No loose link can
        separate from the lamp frame.
        """
        rendered = self.template.replace('__MODEL_NAME__', model_name)
        for colour in COLOURS:
            name = colour.lower()
            on_y = '-0.018' if colour == active_state else '0'
            off_y = '0' if colour == active_state else '-0.018'
            # Replace only the authored pose prefix of each named link.
            if colour == 'RED':
                rendered = rendered.replace(
                    '<link name="red_on"><gravity>false</gravity><pose>-0.210 0 ',
                    '<link name="red_on"><gravity>false</gravity><pose>-0.210 %s ' % on_y)
                rendered = rendered.replace(
                    '<link name="red_off"><gravity>false</gravity><pose>-0.210 -0.018 ',
                    '<link name="red_off"><gravity>false</gravity><pose>-0.210 %s ' % off_y)
            elif colour == 'YELLOW':
                rendered = rendered.replace(
                    '<link name="yellow_on"><gravity>false</gravity><pose>0 0 ',
                    '<link name="yellow_on"><gravity>false</gravity><pose>0 %s ' % on_y)
                rendered = rendered.replace(
                    '<link name="yellow_off"><gravity>false</gravity><pose>0 -0.018 ',
                    '<link name="yellow_off"><gravity>false</gravity><pose>0 %s ' % off_y)
            else:
                rendered = rendered.replace(
                    '<link name="green_on"><gravity>false</gravity><pose>0.210 0 ',
                    '<link name="green_on"><gravity>false</gravity><pose>0.210 %s ' % on_y)
                rendered = rendered.replace(
                    '<link name="green_off"><gravity>false</gravity><pose>0.210 -0.018 ',
                    '<link name="green_off"><gravity>false</gravity><pose>0.210 %s ' % off_y)
        return rendered

    def panel_state(self, model, colour, illuminated, visible):
        model_name, model_x, model_y, model_z, yaw = model
        local_x = LOCAL_X[colour]
        local_y = FRONT_Y if visible else HIDDEN_Y
        link = LinkState()
        link.link_name = '%s::%s_%s' % (
            model_name, colour.lower(), 'on' if illuminated else 'off'
        )
        link.reference_frame = 'world'
        link.pose = pose_at(
            model_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            model_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
            model_z + PANEL_Z, yaw,
        )
        return link

    def move_panel(self, model, colour, illuminated, visible):
        try:
            response = self.set_link_state(self.panel_state(model, colour, illuminated, visible))
        except rospy.ServiceException as exc:
            rospy.logerr('link-state service disconnected: %s', exc)
            return False
        if not response.success:
            rospy.logerr('panel update failed for %s: %s', colour, response.status_message)
        return response.success

    def apply_state(self, active_state):
        """Expose exactly one illuminated face while keeping the fixed frame."""
        self.ready_pub.publish(False)
        for model in LIGHTS:
            for colour in COLOURS:
                is_active = colour == active_state
                if not self.move_panel(model, colour, True, is_active):
                    return False
                if not self.move_panel(model, colour, False, not is_active):
                    return False
        self.displayed_state = active_state
        return True

    def hold_state(self, duration):
        deadline = time.monotonic() + duration
        while not rospy.is_shutdown():
            remaining = max(0.0, deadline - time.monotonic())
            self.remaining_pub.publish(remaining)
            if remaining <= 0.0:
                return
            time.sleep(min(0.1, remaining))

    def run(self):
        while not rospy.is_shutdown():
            if not self.connect() or not self.create_fixed_models():
                time.sleep(1.0)
                continue
            disconnected = False
            while not rospy.is_shutdown() and not disconnected:
                for active_state, duration in self.cycle:
                    if not self.apply_state(active_state):
                        disconnected = True
                        break
                    self.state_pub.publish(active_state)
                    self.ready_pub.publish(True)
                    rospy.loginfo('traffic lights: %s for %.1f seconds', active_state, duration)
                    self.hold_state(duration)


def main():
    rospy.init_node('traffic_light_controller')
    try:
        TrafficLightController().run()
    except (OSError, ValueError) as exc:
        rospy.logfatal('%s', exc)


if __name__ == '__main__':
    main()

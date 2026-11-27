"""Load a recorded run as a list of events.

An event is {'t': seconds, 'topic': '/name', 'data': {...}}. A run is either a
folder written by record_trial (with a bag/ inside, needs ROS to read) or an
events.jsonl file/folder exported from one (works on any PC).
"""
import json
import math
import os

import yaml

EVENTS_FILE = 'events.jsonl'


def message_to_data(topic, msg):
    """Reduce a ROS message to the fields the analyzer and replay use."""
    if topic == '/cmd_vel':
        return {'linear': msg.linear.x, 'angular': msg.angular.z}
    if topic == '/odom':
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {'x': pos.x, 'y': pos.y, 'yaw': yaw,
                'v': msg.twist.twist.linear.x,
                'w': msg.twist.twist.angular.z}
    if topic.startswith('/ultrasonic/'):
        return {'range': msg.range}
    if topic in ('/vision/target', '/vision/direction', '/vision/fps',
                 '/behavior/state', '/behavior/safety'):
        return {'data': msg.data}
    return {}


def read_bag(bag_dir):
    """Read a rosbag2 MCAP folder into events (needs a sourced ROS)."""
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type)
             for t in reader.get_all_topics_and_types()}
    events = []
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        events.append({'t': stamp_ns / 1e9, 'topic': topic,
                       'data': message_to_data(topic, msg)})
    return events


def write_events(path, events):
    """Write events as JSON Lines."""
    with open(path, 'w', encoding='utf-8') as fh:
        for event in events:
            fh.write(json.dumps(event) + '\n')


def read_jsonl(path):
    """Read JSON objects from a file, skipping damaged lines; [] if absent."""
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if isinstance(item, dict):
                out.append(item)
    return out


def load_events(path):
    """Events sorted by time from an events file or a run folder."""
    if os.path.isfile(path):
        events = read_jsonl(path)
    elif os.path.isfile(os.path.join(path, EVENTS_FILE)):
        events = read_jsonl(os.path.join(path, EVENTS_FILE))
    elif os.path.isdir(os.path.join(path, 'bag')):
        events = read_bag(os.path.join(path, 'bag'))
    else:
        raise FileNotFoundError('no events.jsonl or bag/ in %s' % path)
    return sorted(events, key=lambda e: e['t'])


def load_params(path, node):
    """ros__parameters of node from a `ros2 param dump` or robot.yaml file."""
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    for key in ('/' + node, node):
        if isinstance(data.get(key), dict):
            return dict(data[key].get('ros__parameters') or {})
    return {}


def run_folder(path):
    """Return the folder holding marks and parameters for a run path."""
    return path if os.path.isdir(path) else os.path.dirname(path) or '.'

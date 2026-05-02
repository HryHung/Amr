#!/usr/bin/env python3
import json
import os
import threading
import time
from typing import Dict, List, Optional

from flask import Flask, jsonify, render_template, request
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor

from pathlib import Path
from flask import Flask

BASE_DIR = Path(__file__).resolve().parent
app = Flask(
    __name__,
    template_folder=str(BASE_DIR / 'templates'),
    static_folder=str(BASE_DIR / 'static')
)


class MissionManager:
    def __init__(self, waypoint_file: str):
        self.waypoint_file = os.path.expanduser(waypoint_file)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.ack_event = threading.Event()
        self.executor: Optional[MultiThreadedExecutor] = None
        self.executor_thread: Optional[threading.Thread] = None
        self.worker_thread: Optional[threading.Thread] = None

        self.status = 'BOOTING'
        self.message = 'Starting web mission manager...'
        self.current_target: Optional[str] = None
        self.awaiting_ack_target: Optional[str] = None
        self.mission_sequence: List[str] = []
        self.cancel_requested = False
        self.home_name: Optional[str] = None
        self.locations: Dict[str, PoseStamped] = {}
        self.raw_points: Dict[str, dict] = {}

        self.navigator = BasicNavigator(node_name='web_mission_navigator')
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.navigator)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.navigator.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.navigator.get_logger().info('Nav2 is active.')

        self.reload_waypoints()
        self.status = 'IDLE'
        self.message = 'Ready for mission input.'

    def shutdown(self):
        self.stop_event.set()
        with self.lock:
            try:
                self.navigator.cancelTask()
            except Exception:
                pass

        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)

        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1.0)
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)

        try:
            self.navigator.lifecycleShutdown()
        except Exception:
            pass
        try:
            self.navigator.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    def create_pose(self, x: float, y: float, qz: float, qw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def reload_waypoints(self):
        if not os.path.exists(self.waypoint_file):
            raise FileNotFoundError(f'Waypoint file not found: {self.waypoint_file}')

        with open(self.waypoint_file, 'r', encoding='utf-8') as handle:
            data = json.load(handle)

        if not data:
            raise RuntimeError('waypoints.json exists but contains no waypoint.')

        locations: Dict[str, PoseStamped] = {}
        for name, item in data.items():
            for key in ['x', 'y', 'qz', 'qw']:
                if key not in item:
                    raise KeyError(f'Waypoint {name} missing key: {key}')
            locations[name] = self.create_pose(item['x'], item['y'], item['qz'], item['qw'])

        home_name = None
        for name in locations.keys():
            low = name.lower()
            if 'home' in low or low == 'o':
                home_name = name
                break
        if home_name is None:
            home_name = list(locations.keys())[0]

        with self.lock:
            self.raw_points = data
            self.locations = locations
            self.home_name = home_name

    def get_waypoint_names(self) -> List[str]:
        with self.lock:
            return list(self.locations.keys())

    def get_status_payload(self):
        with self.lock:
            return {
                'status': self.status,
                'message': self.message,
                'current_target': self.current_target,
                'awaiting_ack_target': self.awaiting_ack_target,
                'mission_sequence': self.mission_sequence,
                'home_name': self.home_name,
                'waypoints': self.raw_points,
                'busy': self.worker_thread is not None and self.worker_thread.is_alive(),
            }

    def start_mission(self, mission_sequence: List[str]):
        with self.lock:
            if self.worker_thread is not None and self.worker_thread.is_alive():
                raise RuntimeError('Another mission is running.')
            if not mission_sequence:
                raise RuntimeError('Mission sequence is empty.')

            unknown = [name for name in mission_sequence if name not in self.locations]
            if unknown:
                raise RuntimeError(f'Unknown waypoint(s): {unknown}')

            self.cancel_requested = False
            self.ack_event.clear()
            self.awaiting_ack_target = None
            self.current_target = None
            self.mission_sequence = mission_sequence[:]
            self.status = 'RUNNING'
            self.message = 'Mission accepted. Robot is preparing to move.'
            self.worker_thread = threading.Thread(
                target=self._run_mission,
                args=(mission_sequence[:],),
                daemon=True,
            )
            self.worker_thread.start()

    def continue_after_arrival(self):
        with self.lock:
            if self.awaiting_ack_target is None:
                raise RuntimeError('Robot is not waiting for station confirmation.')
            self.message = f'Continue command accepted at [{self.awaiting_ack_target}].'
            self.ack_event.set()

    def cancel_mission(self):
        with self.lock:
            if self.worker_thread is None or not self.worker_thread.is_alive():
                raise RuntimeError('No active mission to cancel.')
            self.cancel_requested = True
            self.message = 'Cancel requested. Robot will return home.'
            self.ack_event.set()
            try:
                self.navigator.cancelTask()
            except Exception:
                pass

    def _go_to_named_target(self, target_name: str) -> TaskResult:
        with self.lock:
            self.current_target = target_name
            self.message = f'Moving to [{target_name}]...'
            pose = self.locations[target_name]

        self.navigator.goToPose(pose)

        while not self.stop_event.is_set():
            with self.lock:
                if self.cancel_requested:
                    try:
                        self.navigator.cancelTask()
                    except Exception:
                        pass
                    break
            if self.navigator.isTaskComplete():
                break
            time.sleep(0.2)

        return self.navigator.getResult()

    def _return_home(self):
        with self.lock:
            home_name = self.home_name
            if home_name is None:
                self.status = 'FAILED'
                self.message = 'No home waypoint available.'
                return
            self.current_target = home_name
            self.awaiting_ack_target = None
            self.ack_event.clear()
            self.message = f'Returning home to [{home_name}]...'

        result = self._go_to_named_target(home_name)
        with self.lock:
            if result == TaskResult.SUCCEEDED:
                self.status = 'IDLE'
                self.message = f'Robot reached home [{home_name}] safely.'
            elif result == TaskResult.CANCELED:
                self.status = 'IDLE'
                self.message = f'Return-home task canceled. Robot last target was [{home_name}].'
            else:
                self.status = 'FAILED'
                self.message = f'Robot failed to return home [{home_name}].'
            self.current_target = None
            self.awaiting_ack_target = None
            self.mission_sequence = []
            self.cancel_requested = False
            self.ack_event.clear()

    def _run_mission(self, sequence: List[str]):
        try:
            for target_name in sequence:
                result = self._go_to_named_target(target_name)
                with self.lock:
                    canceled = self.cancel_requested

                if canceled:
                    break

                if result == TaskResult.SUCCEEDED:
                    with self.lock:
                        self.awaiting_ack_target = target_name
                        self.status = 'WAITING_CONFIRM'
                        self.message = f'Arrived at [{target_name}]. Waiting for station confirmation.'
                        self.ack_event.clear()

                    while not self.stop_event.is_set():
                        with self.lock:
                            if self.cancel_requested:
                                break
                        if self.ack_event.wait(timeout=0.2):
                            break

                    with self.lock:
                        self.awaiting_ack_target = None
                        if not self.cancel_requested:
                            self.status = 'RUNNING'
                            self.message = f'Station [{target_name}] confirmed. Continuing mission.'
                            self.ack_event.clear()
                elif result == TaskResult.CANCELED:
                    with self.lock:
                        self.cancel_requested = True
                        self.message = f'Navigation to [{target_name}] canceled. Returning home.'
                    break
                else:
                    with self.lock:
                        self.cancel_requested = True
                        self.status = 'FAILED'
                        self.message = f'Navigation failed at [{target_name}]. Returning home.'
                    break
        finally:
            self._return_home()


def create_app(manager: MissionManager) -> Flask:
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    static_dir = os.path.join(os.path.dirname(__file__), 'static')
    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)

    @app.get('/')
    def index():
        return render_template('index.html')

    @app.get('/api/status')
    def api_status():
        return jsonify(manager.get_status_payload())

    @app.get('/api/waypoints')
    def api_waypoints():
        return jsonify({
            'home_name': manager.home_name,
            'waypoints': manager.raw_points,
        })

    @app.post('/api/reload_waypoints')
    def api_reload_waypoints():
        try:
            manager.reload_waypoints()
            return jsonify({'ok': True, 'message': 'Waypoint file reloaded.'})
        except Exception as exc:
            return jsonify({'ok': False, 'message': str(exc)}), 400

    @app.post('/api/start_mission')
    def api_start_mission():
        payload = request.get_json(silent=True) or {}
        mission_sequence = payload.get('mission_sequence', [])
        try:
            manager.start_mission(mission_sequence)
            return jsonify({'ok': True, 'message': 'Mission started.'})
        except Exception as exc:
            return jsonify({'ok': False, 'message': str(exc)}), 400

    @app.post('/api/continue')
    def api_continue():
        try:
            manager.continue_after_arrival()
            return jsonify({'ok': True, 'message': 'Continue accepted.'})
        except Exception as exc:
            return jsonify({'ok': False, 'message': str(exc)}), 400

    @app.post('/api/cancel')
    def api_cancel():
        try:
            manager.cancel_mission()
            return jsonify({'ok': True, 'message': 'Cancel accepted.'})
        except Exception as exc:
            return jsonify({'ok': False, 'message': str(exc)}), 400

    return app


def main():
    rclpy.init(args=None)
    bootstrap_node = rclpy.create_node('web_mission_server_bootstrap')
    bootstrap_node.declare_parameter('host', '127.0.0.1')
    bootstrap_node.declare_parameter('port', 8080)
    bootstrap_node.declare_parameter(
        'waypoint_file',
        '/home/hungubuntu/Documents/amr_ws/waypoints.json'
    )

    host = bootstrap_node.get_parameter('host').get_parameter_value().string_value
    port = bootstrap_node.get_parameter('port').get_parameter_value().integer_value
    waypoint_file = bootstrap_node.get_parameter('waypoint_file').get_parameter_value().string_value
    bootstrap_node.destroy_node()

    manager = None
    try:
        manager = MissionManager(waypoint_file=waypoint_file)
        app = create_app(manager)
        manager.navigator.get_logger().info(
            f'Web mission server running at http://{host}:{port}'
        )
        app.run(host=host, port=int(port), debug=False, use_reloader=False)
    finally:
        if manager is not None:
            manager.shutdown()


if __name__ == '__main__':
    main()

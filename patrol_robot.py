#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from threading import Lock, Thread
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# Battery threshold constants (percentage as float 0.0 ~ 1.0)
BATTERY_HIGH = 0.95      # Resume patrol after charging
BATTERY_LOW = 0.30       # Trigger auto-docking to recharge
BATTERY_CRITICAL = 0.10  # Emergency shutdown

def reverse_direction(direction):
    """Return the opposite direction for reverse patrol path."""
    reverse_map = {
        TurtleBot4Directions.NORTH: TurtleBot4Directions.SOUTH,
        TurtleBot4Directions.SOUTH: TurtleBot4Directions.NORTH,
        TurtleBot4Directions.EAST:  TurtleBot4Directions.WEST,
        TurtleBot4Directions.WEST:  TurtleBot4Directions.EAST,
    }
    return reverse_map.get(direction, direction)


class BatteryMonitor(Node):
    """Subscribes to battery_state topic and stores current battery percentage."""
    def __init__(self, lock):
        super().__init__('battery_monitor')
        self.lock = lock
        self.battery_percent = None
        self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            10
        )

    def battery_state_callback(self, msg):
        with self.lock:
            self.battery_percent = msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


class TargetGoalSubscriber(Node):
    """
    Subscribes to /mqtt_message topic to receive sick cat location
    from the tracking robot via MQTT broker.
    Message format: 'x,y' (comma-separated float coordinates)
    """
    def __init__(self, lock):
        super().__init__('target_goal_subscriber')
        self.lock = lock
        self.received_goal = None

        self.create_subscription(
            String,
            '/mqtt_message',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg: String):
        try:
            parts = msg.data.strip().split(',')
            x = float(parts[0])
            y = float(parts[1])
            self.get_logger().info(f"Received external goal from tracking robot: x={x}, y={y}")

            # Default direction set to EAST when arriving at sick cat location
            goal_pose = TurtleBot4Navigator().getPoseStamped([x, y], TurtleBot4Directions.EAST)
            with self.lock:
                self.received_goal = goal_pose

        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse MQTT message '{msg.data}': {e}")

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


def main(args=None):
    rclpy.init(args=args)
    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    goal_subscriber = TargetGoalSubscriber(lock)
    navigator = TurtleBot4Navigator()
    position_index = 0

    # Run battery monitor and goal subscriber in separate threads
    Thread(target=battery_monitor.thread_function, daemon=True).start()
    Thread(target=goal_subscriber.thread_function, daemon=True).start()

    # Dock first to set initial pose reliably
    if not navigator.getDockedStatus():
        navigator.info("Docking before initializing pose.")
        navigator.dock()

    # Fixed initial pose based on self-generated SLAM map coordinate
    # Future improvement: replace with AMCL auto-localization
    initial_pose = navigator.getPoseStamped([0.51, 4.98], TurtleBot4Directions.WEST)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.undock()

    # Patrol waypoints are hardcoded based on a custom map generated via SLAM:
    # 1. Manually operated TurtleBot4 through the environment (teleoperation)
    # 2. LiDAR scanned surroundings during movement
    # 3. SLAM Toolbox (Nav2) built the map from scan data
    # 4. Waypoint coordinates extracted from the generated map
    # Future improvement: load waypoints dynamically via ROS2 parameters
    forward_path = [
        ([0.81, 4.02], TurtleBot4Directions.NORTH),
        ([1.4885, 4.02], TurtleBot4Directions.NORTH),
        ([2.08756, 4.02], TurtleBot4Directions.NORTH),
        ([2.08756, 4.02], TurtleBot4Directions.SOUTH),
        ([0.7873, 4.02], TurtleBot4Directions.EAST),
        ([0.7873, 3.07162], TurtleBot4Directions.EAST),
        ([0.7873, 2.43439], TurtleBot4Directions.EAST),
        ([0.7873, 2.43439], TurtleBot4Directions.WEST),
        ([0.7873, 3.07162], TurtleBot4Directions.WEST),
        ([0.7873, 3.7094], TurtleBot4Directions.WEST),
        ([0.7873, 3.7094], TurtleBot4Directions.SOUTH),
        ([-0.1651, 3.7094], TurtleBot4Directions.SOUTH),
        ([-0.323, 3.7094], TurtleBot4Directions.SOUTH),
        ([-0.5944, 3.7094], TurtleBot4Directions.SOUTH),
        ([-1.10, 3.7094], TurtleBot4Directions.SOUTH),
        ([-1.10, 3.7094], TurtleBot4Directions.WEST),
        ([-1.10, 4.2], TurtleBot4Directions.WEST),
        ([-1.10, 5.00], TurtleBot4Directions.WEST),
    ]

    # Build full patrol path: forward + reverse
    goal_pose = []
    for pos, direction in forward_path:
        goal_pose.append(navigator.getPoseStamped(pos, direction))
    for pos, direction in reversed(forward_path):
        goal_pose.append(navigator.getPoseStamped(pos, reverse_direction(direction)))

    while rclpy.ok():
        with lock:
            battery_percent = battery_monitor.battery_percent
            external_goal = goal_subscriber.received_goal

        if battery_percent is not None:
            navigator.info(f'Battery: {battery_percent * 100:.2f}%')

            if battery_percent < BATTERY_CRITICAL:
                # Emergency shutdown — battery too low to operate safely
                navigator.error("Battery critically low! Shutting down.")
                break

            elif battery_percent < BATTERY_LOW:
                # Auto-dock and recharge until battery reaches BATTERY_HIGH
                navigator.info("Low battery. Docking to recharge.")
                # Docking station position hardcoded based on SLAM map coordinate
                navigator.startToPose(navigator.getPoseStamped([0.51, 4.90], TurtleBot4Directions.EAST))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error("Docking failed.")
                    break

                while battery_percent < BATTERY_HIGH:
                    sleep(15)
                    with lock:
                        battery_percent = battery_monitor.battery_percent
                    navigator.info(f"Charging... {battery_percent * 100:.2f}%")

                navigator.undock()
                position_index = 0  # Restart patrol from beginning

            elif external_goal:
                # Sick cat detected — interrupt patrol and navigate to location
                # Uses intermediate waypoints to avoid obstacles on the map
                navigator.info("External goal received from tracking robot. Interrupting patrol.")

                intermediate1 = navigator.getPoseStamped([0.70638, 3.07162], TurtleBot4Directions.EAST)
                navigator.info("Moving to intermediate waypoint 1")
                navigator.startToPose(intermediate1)
                sleep(2)

                intermediate2 = navigator.getPoseStamped([-0.757764, 3.83305], TurtleBot4Directions.EAST)
                navigator.info("Moving to intermediate waypoint 2")
                navigator.startToPose(intermediate2)
                sleep(2)

                navigator.info("Moving to sick cat location")
                navigator.startToPose(external_goal)
                sleep(2)

                # Clear received goal and resume patrol
                with lock:
                    goal_subscriber.received_goal = None

                navigator.info("Resuming patrol.")
                position_index = 0

            else:
                # Normal patrol — move to next waypoint in sequence
                navigator.startToPose(goal_pose[position_index])
                position_index = (position_index + 1) % len(goal_pose)

    battery_monitor.destroy_node()
    goal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

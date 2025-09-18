#!/usr/bin/env python3
# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Run takeoff and lawnmower pattern flight on Copter.

Warning - This is NOT production code; it's a simple demo of capability.
"""

import math
import rclpy
import time

from rclpy.node import Node
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPoseStamped
from geopy import distance
from geopy import point
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff


COPTER_MODE_GUIDED = 4
FRAME_GLOBAL_INT = 5
TAKEOFF_ALT = 595.0  # meters


class CopterLawnmower(Node):
    """Copter takeoff and lawnmower pattern using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_lawnmower")

        # Setup arm service
        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._client_arm = self.create_client(ArmMotors, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        # Setup mode switch service
        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        # Setup takeoff service
        self.declare_parameter("takeoff_service", "/ap/experimental/takeoff")
        self._takeoff_topic = self.get_parameter("takeoff_service").get_parameter_value().string_value
        self._client_takeoff = self.create_client(Takeoff, self._takeoff_topic)
        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        # Setup global position publisher for waypoints
        self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")
        self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
        self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)

        # Setup geopose subscription
        self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
        self._geopose_topic = self.get_parameter("geopose_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, 
            depth=1
        )

        self._subscription_geopose = self.create_subscription(
            GeoPoseStamped, self._geopose_topic, self.geopose_cb, qos
        )
        self._cur_geopose = GeoPoseStamped()

    def geopose_cb(self, msg: GeoPoseStamped):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:
            self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))
            self._cur_geopose = msg

    def arm(self):
        """Arm the motors."""
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        """Switch flight mode."""
        req = ModeSwitch.Request()
        assert mode in [COPTER_MODE_GUIDED]
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode and self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)
        return is_in_desired_mode

    def takeoff(self, alt):
        """Takeoff to specified altitude."""
        req = Takeoff.Request()
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff_with_timeout(self, alt: float, timeout: rclpy.duration.Duration):
        """Try to takeoff. Returns true on success, or false if takeoff fails or times out."""
        takeoff_success = False
        start = self.get_clock().now()
        while not takeoff_success and self.get_clock().now() - start < timeout:
            result = self.takeoff(alt)
            takeoff_success = result.status
            time.sleep(1)
        return takeoff_success

    def get_cur_geopose(self):
        """Return latest geopose."""
        return self._cur_geopose

    def send_goal_position(self, goal_global_pos):
        """Send goal position. Must be in guided for this to work."""
        self._global_pos_pub.publish(goal_global_pos)

    def create_waypoint(self, lat, lon, alt):
        """Create a GlobalPosition waypoint."""
        waypoint = GlobalPosition()
        waypoint.latitude = lat
        waypoint.longitude = lon
        waypoint.altitude = alt
        waypoint.coordinate_frame = FRAME_GLOBAL_INT
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.get_clock().now().to_msg()
        return waypoint

    def achieved_goal(self, goal_global_pos, cur_geopose, tolerance=5.0):
        """Return true if the current position is close enough to the goal."""
        if not cur_geopose.header.stamp.sec:
            return False
            
        p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
        cur_pos = cur_geopose.pose.position
        p2 = (cur_pos.latitude, cur_pos.longitude, cur_pos.altitude)

        flat_distance = distance.distance(p1[:2], p2[:2]).m
        euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
        self.get_logger().info(f"Goal is {euclidian_distance:.2f} meters away")
        return euclidian_distance < tolerance

    def wait_for_waypoint(self, waypoint, timeout_sec=60):
        """Wait for the copter to reach a waypoint."""
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)
        
        while self.get_clock().now() - start < timeout:
            rclpy.spin_once(self)
            if self.achieved_goal(waypoint, self.get_cur_geopose()):
                return True
            time.sleep(1.0)
        
        return False

    def generate_lawnmower_pattern(self, start_lat, start_lon, width_m, height_m, spacing_m, altitude):
        """Generate lawnmower pattern waypoints."""
        waypoints = []
        
        # Calculate approximate degree per meter at this latitude
        lat_deg_per_m = 1.0 / 111320.0  # approximately constant
        lon_deg_per_m = 1.0 / (111320.0 * math.cos(math.radians(start_lat)))
        
        # Number of passes
        num_passes = int(height_m / spacing_m) + 1
        
        for i in range(num_passes):
            # Vertical offset (north/south)
            y_offset = -i * spacing_m * lat_deg_per_m
            
            # Horizontal offsets (east/west)
            x_start = 0                  # start at left (relative)
            x_end = width_m * lon_deg_per_m  # end at right (relative)
            
            # Alternate direction for each pass
            if i % 2 == 0:  # Even pass: left to right
                start_x, end_x = x_start, x_end
            else:           # Odd pass: right to left
                start_x, end_x = x_end, x_start
            
            # Compute waypoints
            lat = start_lat + y_offset
            waypoints.append(self.create_waypoint(lat, start_lon + start_x, altitude))
            waypoints.append(self.create_waypoint(lat, start_lon + end_x, altitude))

        
        return waypoints

    def execute_lawnmower_pattern(self, start_lat, start_lon, width_m=50, height_m=50, spacing_m=10):
        """Execute a lawnmower pattern starting from the given coordinates."""
        self.get_logger().info("Generating lawnmower pattern...")
        
        # Use current altitude for the pattern
        current_alt = self.get_cur_geopose().pose.position.altitude
        if current_alt < 5:  # Safety check
            current_alt = TAKEOFF_ALT
            
        waypoints = self.generate_lawnmower_pattern(
            start_lat, start_lon, width_m, height_m, spacing_m, current_alt
        )
        
        self.get_logger().info(f"Generated {len(waypoints)} waypoints for lawnmower pattern")
        
        # Execute waypoints
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f"Flying to waypoint {i+1}/{len(waypoints)}")
            self.send_goal_position(waypoint)
            
            if not self.wait_for_waypoint(waypoint):
                self.get_logger().error(f"Failed to reach waypoint {i+1}")
                return False
                
            self.get_logger().info(f"Reached waypoint {i+1}")
            
        self.get_logger().info("Lawnmower pattern completed!")
        return True


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = CopterLawnmower()
    
    try:
        # Switch to guided mode
        node.get_logger().info("Switching to guided mode...")
        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")
        
        # Arm the copter
        node.get_logger().info("Arming motors...")
        if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Unable to arm")

        # Takeoff
        node.get_logger().info(f"Taking off to {TAKEOFF_ALT}m...")
        if not node.takeoff_with_timeout(TAKEOFF_ALT, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to takeoff")

        # Wait for takeoff to complete
        node.get_logger().info("Waiting for takeoff to complete...")
        is_ascending = True
        while is_ascending:
            rclpy.spin_once(node)
            time.sleep(1.0)
            current_alt = node.get_cur_geopose().pose.position.altitude
            is_ascending = current_alt < (TAKEOFF_ALT - 1.0)  # Allow some tolerance

        node.get_logger().info(f"Takeoff completed at {current_alt:.2f}m")

        # Get starting position for lawnmower pattern
        current_pos = node.get_cur_geopose().pose.position
        start_lat = current_pos.latitude
        start_lon = current_pos.longitude
        
        node.get_logger().info(f"Starting lawnmower pattern at lat: {start_lat:.6f}, lon: {start_lon:.6f}")
        
        # Execute lawnmower pattern
        # You can adjust these parameters:
        # width_m: width of the area to cover in meters
        # height_m: height of the area to cover in meters  
        # spacing_m: distance between parallel lines in meters
        success = node.execute_lawnmower_pattern(
            start_lat, start_lon, 
            width_m=80,     # 40 meter wide area
            height_m=80,    # 40 meter long area
            spacing_m=8     # 8 meter spacing between lines
        )
        
        if success:
            node.get_logger().info("Mission completed successfully!")
        else:
            node.get_logger().error("Mission failed!")

    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Mission failed with error: {str(e)}")

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

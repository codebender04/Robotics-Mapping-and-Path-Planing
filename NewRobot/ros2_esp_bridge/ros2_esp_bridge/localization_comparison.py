#!/usr/bin/env python3
"""
Localization Comparison Script

Extends the rectangle navigation to compare:
1. Dead Reckoning (wheel odometry) 
2. Landmark-Based (LiDAR + cylinders/walls)
3. Ground Truth (manual measurements)

Generates visual comparison graphs and accuracy analysis.

# Coordinate Frame Diagram
#
#  Rectangle (20cm x 20cm)
#
#   ^ Y (up)
#   |
#   |
#   C3---------C2
#   |           |
#   |           |
#   C4---------C1 ---> X (right)
#
# Robot always starts at C4 (bottom left), facing towards C1 (bottom right)
# - X axis points right
# - Y axis points up
# - All landmark positions and robot movements are measured in this frame
#
# This ensures auto-discovered landmark positions match the navigation frame.
"""

import serial
import time
import math
import json
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from rectangle_path import RectanglePathController

# ROS2 imports for LiDAR data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LIDAR_YAW_OFFSET = 1.5708

class LandmarkTracker(Node):
    def __init__(self):
        super().__init__('landmark_tracker')
        
        # Landmark positions - will be auto-discovered during calibration
        self.landmarks = {}  # Auto-populated during initial scan
        self.landmark_auto_discovery = True  # Enable automatic landmark mapping
        
        # Room dimensions (measure your enclosed area)
        self.room_width = 3.0   # meters (MEASURE THIS!)  
        self.room_height = 2.5  # meters (MEASURE THIS!)
        
        # LiDAR data
        self.latest_scan = None
        self.lidar_sub = self.create_subscription(
            LaserScan, 
            '/scan',  # Your LiDAR topic
            self.lidar_callback,
            10
        )
    
    def lidar_callback(self, msg):
        """Store latest LiDAR scan data"""
        self.latest_scan = msg
    
    def auto_discover_landmarks(self, initial_position=(0.0, 0.0)):
        """Automatically map landmark positions during initial calibration"""
        print("Auto-discovering landmark positions...")
        print("   Place robot at known starting position and press Enter")
        input("   Ready? ")
        
        # Wait for LiDAR data (ROS2 is spinning in background)
        print("   Waiting for LiDAR data...")
        wait_count = 0
        while self.latest_scan is None and wait_count < 50:  # Wait up to 5 seconds
            time.sleep(0.1)
            wait_count += 1
        
        if self.latest_scan is None:
            print("   ERROR: No LiDAR data received! Check if /scan topic is publishing")
            print("   Try: ros2 topic list | grep scan")
            print("   Try: ros2 topic echo /scan --once")
            return False
        
        print(f"   LiDAR data received ({len(self.latest_scan.ranges)} points)")
        
        # Scan for cylinders from known starting position
        if self.latest_scan:
            cylinders = self.detect_cylinders(self.latest_scan)
            print(f"   Found {len(cylinders)} potential landmarks")
            
            # Filter to keep only the best candidates
            good_landmarks = []
            for cylinder in cylinders:
                # Prefer closer objects and high confidence
                if cylinder['distance'] < 3.0:  # Within 3m
                    good_landmarks.append(cylinder)
            
            # Sort by confidence and distance, keep top candidates
            good_landmarks.sort(key=lambda x: (x['confidence'] == 'HIGH', -x['distance']), reverse=True)
            selected_landmarks = good_landmarks[:6]  # Keep up to 6 best landmarks
            
            for cylinder in selected_landmarks:
                # Calculate global position from robot position + relative detection
                x_global = initial_position[0] + cylinder['distance'] * np.cos(cylinder['angle'])
                y_global = initial_position[1] + cylinder['distance'] * np.sin(cylinder['angle'])
                
                # Store in landmark map
                self.landmarks[cylinder['id']] = (x_global, y_global)
                conf_symbol = "[HIGH]" if cylinder['confidence'] == 'HIGH' else "[LOW]"
                print(f"   Mapped {cylinder['id']} at ({x_global:.2f}, {y_global:.2f}) - {cylinder['confidence']} {conf_symbol}")
        
        print(f"   Discovered {len(self.landmarks)} landmarks")
        if len(self.landmarks) == 0:
            print(f"   WARNING: No landmarks detected! Try placing larger/closer cylindrical objects")
            print(f"   Place cans, bottles, or tubes within 2m of robot")
        else:
            print(f"   Using {len(self.landmarks)} landmarks for localization")
        return len(self.landmarks) > 0

    def detect_cylinders(self, scan):
        """Detect cylindrical landmarks by finding arcs and fitting circles"""
        if not scan:
            return []
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        angles = angles + LIDAR_YAW_OFFSET
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        
        # Convert to Cartesian coordinates
        valid_mask = ~(np.isinf(ranges) | np.isnan(ranges) | (ranges < 0.1) | (ranges > 4.0))
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        print(f"   LiDAR scan: {len(valid_ranges)} valid points")
        
        # Find arc segments that could be cylinders
        arc_segments = self._find_arc_segments(x_points, y_points, valid_ranges, valid_angles)
        
        # Fit circles to each arc segment
        cylinders_found = []
        for i, arc in enumerate(arc_segments):
            circle_fit = self._fit_circle_to_arc(arc)
            
            if circle_fit:
                # Calculate distance and angle to circle center from robot (at origin)
                cx, cy, radius = circle_fit['center_x'], circle_fit['center_y'], circle_fit['radius']
                distance_to_center = math.sqrt(cx**2 + cy**2)
                angle_to_center = math.atan2(cy, cx)
                
                # Validate cylinder properties - RELAXED criteria
                if (0.01 < radius < 0.30 and  # Broader cylinder radius (1-30cm)
                    0.2 < distance_to_center < 4.0 and  # Broader distance range
                    circle_fit['quality'] > 0.4):  # Lower quality threshold
                    
                    cylinder = {
                        'id': f'cylinder_{i+1}',
                        'angle': angle_to_center,
                        'distance': distance_to_center,
                        'radius': radius,
                        'center_x': cx,
                        'center_y': cy,
                        'confidence': 'HIGH' if circle_fit['quality'] > 0.85 else 'MEDIUM',
                        'quality': circle_fit['quality'],
                        'arc_points': len(arc['x_points'])
                    }
                    
                    cylinders_found.append(cylinder)
                    print(f"   CYLINDER {i+1}: center=({cx:.2f}, {cy:.2f})m, "
                          f"radius={radius:.3f}m, quality={circle_fit['quality']:.3f}")
        
        # Sort by quality and limit to 2 best
        cylinders_found.sort(key=lambda x: x['quality'], reverse=True)
        
        print(f"   Found {len(cylinders_found)} total cylinder candidates")
        for i, cyl in enumerate(cylinders_found[:5]):  # Show top 5
            print(f"     {i+1}. Quality: {cyl['quality']:.3f}, Radius: {cyl['radius']:.3f}m, Distance: {cyl['distance']:.2f}m")
        
        cylinders_found = cylinders_found[:2]  # Keep top 2
        
        print(f"   Using top {len(cylinders_found)} cylinders for localization")
        return cylinders_found
    
    def _find_arc_segments(self, x_points, y_points, ranges, angles):
        """Find arc segments in the point cloud that could be cylinders"""
        arc_segments = []
        
        # Group nearby points into potential arc segments
        i = 0
        while i < len(x_points) - 5:  # Need at least 6 points for an arc
            # Start a new potential arc
            arc_points_x = [x_points[i]]
            arc_points_y = [y_points[i]]
            arc_ranges = [ranges[i]]
            arc_angles = [angles[i]]
            
            # Extend arc while points are close together
            j = i + 1
            while j < len(x_points):
                # Check if next point is close enough to be part of same object
                dist_between = math.sqrt((x_points[j] - x_points[j-1])**2 + 
                                       (y_points[j] - y_points[j-1])**2)
                
                # Points should be close and roughly same distance from origin - RELAXED
                range_diff = abs(ranges[j] - ranges[j-1])
                
                if dist_between < 0.20 and range_diff < 0.15:  # More permissive thresholds
                    arc_points_x.append(x_points[j])
                    arc_points_y.append(y_points[j])
                    arc_ranges.append(ranges[j])
                    arc_angles.append(angles[j])
                    j += 1
                else:
                    break
            
            # Check if we have enough points for a potential cylinder arc - RELAXED
            if len(arc_points_x) >= 4:  # Reduced from 6 to 4 points
                # Check if points form a reasonable arc (not a straight line)
                angular_span = abs(arc_angles[-1] - arc_angles[0])
                
                if 0.05 < angular_span < 1.5:  # Broader span range (3-86 degrees)
                    arc_segments.append({
                        'x_points': arc_points_x,
                        'y_points': arc_points_y,
                        'ranges': arc_ranges,
                        'angles': arc_angles,
                        'span': angular_span
                    })
                    print(f"   Arc segment: {len(arc_points_x)} points, "
                          f"span={math.degrees(angular_span):.1f}°")
            
            i = j if j > i else i + 1
        
        return arc_segments
    
    def _fit_circle_to_arc(self, arc):
        """Fit a circle to an arc segment using least squares"""
        x_points = np.array(arc['x_points'])
        y_points = np.array(arc['y_points'])
        
        if len(x_points) < 3:
            return None
        
        try:
            # Use algebraic circle fitting (Kasa method)
            # Set up the system of equations: (x-cx)² + (y-cy)² = r²
            # Rearranged: x² + y² - 2*cx*x - 2*cy*y + (cx² + cy² - r²) = 0
            
            A = np.column_stack([2*x_points, 2*y_points, np.ones(len(x_points))])
            b = x_points**2 + y_points**2
            
            # Solve for [cx, cy, cx²+cy²-r²]
            params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            
            if rank < 3:
                return None
                
            cx, cy = params[0], params[1]
            radius = math.sqrt(params[2] + cx**2 + cy**2)
            
            # Calculate quality based on how well points fit the circle
            fitted_distances = np.sqrt((x_points - cx)**2 + (y_points - cy)**2)
            distance_errors = np.abs(fitted_distances - radius)
            max_error = np.max(distance_errors)
            mean_error = np.mean(distance_errors)
            
            # Quality metric (0-1, higher is better)
            quality = max(0.0, 1.0 - (mean_error / 0.05))  # Penalize > 5cm error
            
            return {
                'center_x': cx,
                'center_y': cy,
                'radius': radius,
                'quality': quality,
                'max_error': max_error,
                'mean_error': mean_error
            }
            
        except Exception as e:
            print(f"   ERROR: Circle fitting failed: {e}")
            return None
    
    def estimate_cylinder_radius(self, distances):
        """Rough cylinder radius estimation"""
        if len(distances) < 3:
            return 0.10  # Default 10cm for larger objects
        
        # Use range spread as radius estimate (better for large objects)
        clean_distances = distances[~np.isinf(distances) & ~np.isnan(distances)]
        if len(clean_distances) > 0:
            radius = np.std(clean_distances) * 2  # Scale up for better detection
            return min(0.40, max(0.05, radius))  # 5cm to 40cm range
        return 0.10
    
    def detect_walls(self, scan):
        """Detect wall distances for room-relative positioning"""
        if not scan:
            return {}
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        angles = angles + LIDAR_YAW_OFFSET
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        wall_distances = {}
        
        # Look for walls in different directions
        angle_sectors = {
            'wall_north': (-15, 15),     # Front (degrees)
            'wall_east': (75, 105),      # Right  
            'wall_south': (165, 195),    # Back
            'wall_west': (255, 285)      # Left
        }
        
        for wall_name, (min_deg, max_deg) in angle_sectors.items():
            min_rad = math.radians(min_deg)
            max_rad = math.radians(max_deg)
            
            # Find ranges in this sector
            mask = (angles >= min_rad) & (angles <= max_rad)
            sector_ranges = ranges[mask]
            
            if len(sector_ranges) > 0:
                # Use median distance to wall
                valid_ranges = sector_ranges[sector_ranges > 0.1]  # Filter out invalid readings
                if len(valid_ranges) > 0:
                    wall_distances[wall_name] = np.median(valid_ranges)
        
        return wall_distances
    
    def calculate_position_from_cylinders(self, cylinders):
        """Calculate position using cylinder trilateration"""
        if len(cylinders) < 2:
            return None
        
        # Find distances to known cylinders with unique matching
        print(f"   Detected {len(cylinders)} cylinders, known landmarks: {len(self.landmarks)}")
        
        # Calculate all possible matches first
        matches = []
        for i, cyl in enumerate(cylinders):
            print(f"   Checking cylinder at distance {cyl['distance']:.2f}m, angle {np.degrees(cyl['angle']):.1f}°")
            
            for landmark_id, landmark_pos in self.landmarks.items():
                # Calculate expected angle and distance to this landmark from origin
                expected_angle = math.atan2(landmark_pos[1], landmark_pos[0])
                expected_dist = math.sqrt(landmark_pos[0]**2 + landmark_pos[1]**2)
                
                # Calculate matching error (distance + angle weighted)
                dist_error = abs(expected_dist - cyl['distance'])
                angle_error = abs(expected_angle - cyl['angle'])
                if angle_error > math.pi:
                    angle_error = 2*math.pi - angle_error  # Handle wrap-around
                
                total_error = dist_error + angle_error * 0.5  # Weight angle less than distance
                
                print(f"     {landmark_id}: expected d={expected_dist:.2f}m, a={np.degrees(expected_angle):.1f}°, error={total_error:.3f}")
                print(f"       dist_error={dist_error:.3f}, angle_error={np.degrees(angle_error):.1f}°")
                
                if dist_error < 2.0 and angle_error < math.pi:  # Within 2m and 180°
                    print(f"       CANDIDATE: Adding to matches")
                    matches.append({
                        'cylinder_idx': i,
                        'landmark_id': landmark_id,
                        'error': total_error,
                        'cylinder': cyl,
                        'position': landmark_pos
                    })
                else:
                    print(f"       REJECTED: dist_error={dist_error:.3f} >= 2.0 or angle_error={np.degrees(angle_error):.1f}° >= 180°")
        
        # Sort matches by error (best first) and assign uniquely
        matches.sort(key=lambda x: x['error'])
        detected_landmarks = []
        used_cylinders = set()
        used_landmarks = set()
        
        for match in matches:
            if match['cylinder_idx'] not in used_cylinders and match['landmark_id'] not in used_landmarks:
                detected_landmarks.append({
                    'id': match['landmark_id'],
                    'position': match['position'],
                    'distance': match['cylinder']['distance'],
                    'angle': match['cylinder']['angle']
                })
                used_cylinders.add(match['cylinder_idx'])
                used_landmarks.add(match['landmark_id'])
                print(f"     Matched to {match['landmark_id']}")
        
        # Report unmatched cylinders
        for i, cyl in enumerate(cylinders):
            if i not in used_cylinders:
                print(f"     No match found")
        
        # Use the two closest landmarks for trilateration
        if len(detected_landmarks) >= 2:
            # Sort by distance (closest first)
            detected_landmarks.sort(key=lambda x: x['distance'])
            
            landmark1 = detected_landmarks[0]
            landmark2 = detected_landmarks[1]
            
            # Calculate position using trilateration
            pos = self.trilaterate_2_points(
                landmark1['position'], landmark1['distance'],
                landmark2['position'], landmark2['distance']
            )
            
            if pos is not None:
                print(f"   Trilateration result: ({pos[0]:.3f}, {pos[1]:.3f})m")
                print(f"   Using landmarks {landmark1['id']} and {landmark2['id']} for positioning")
                return pos
            else:
                print(f"   ERROR: Trilateration failed with landmarks {landmark1['id']} and {landmark2['id']}")
                return None
        elif len(detected_landmarks) == 1:
            print(f"   WARNING: Only 1 landmark detected, cannot triangulate position")
            print(f"   Detected: {detected_landmarks[0]['id']} at distance {detected_landmarks[0]['distance']:.2f}m")
            return None
        else:
            print(f"   ERROR: No landmarks detected for positioning")
            return None
    
    def trilaterate_2_points(self, p1, d1, p2, d2):
        """Calculate position from 2 known points and distances"""
        x1, y1 = p1
        x2, y2 = p2
        
        # Calculate intersection of two circles
        d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if d > d1 + d2 or d < abs(d1 - d2) or d == 0:
            print(f"   WARNING: No trilateration solution: d={d:.3f}, d1={d1:.3f}, d2={d2:.3f}")
            return None  # No solution
        
        try:
            a = (d1**2 - d2**2 + d**2) / (2 * d)
            h_squared = d1**2 - a**2
            
            if h_squared < 0:
                print(f"   WARNING: Invalid trilateration: h_squared={h_squared}")
                return None
                
            h = math.sqrt(h_squared)
            
            px = x1 + a * (x2 - x1) / d
            py = y1 + a * (y2 - y1) / d
            
            # Two possible solutions
            x_1 = px + h * (y2 - y1) / d
            y_1 = py - h * (x2 - x1) / d
            
            x_2 = px - h * (y2 - y1) / d
            y_2 = py + h * (x2 - x1) / d
            
            # Choose the solution closer to origin (more reasonable for small robot movements)
            dist1 = math.sqrt(x_1**2 + y_1**2)
            dist2 = math.sqrt(x_2**2 + y_2**2)
            
            if dist1 < dist2:
                print(f"   Trilateration result: ({x_1:.3f}, {y_1:.3f})m")
                return (x_1, y_1)
            else:
                print(f"   Trilateration result: ({x_2:.3f}, {y_2:.3f})m")
                return (x_2, y_2)
                
        except Exception as e:
            print(f"   ERROR: Trilateration error: {e}")
            return None
    
    def calculate_position_from_walls(self, wall_distances):
        """Calculate position from wall distances"""
        if len(wall_distances) < 2:
            return None
        
        x, y = None, None
        
        # Calculate X from east/west walls
        if 'wall_east' in wall_distances and 'wall_west' in wall_distances:
            x = wall_distances['wall_west']
        elif 'wall_east' in wall_distances:
            x = self.room_width - wall_distances['wall_east']
        elif 'wall_west' in wall_distances:
            x = wall_distances['wall_west']
        
        # Calculate Y from north/south walls
        if 'wall_north' in wall_distances and 'wall_south' in wall_distances:
            y = wall_distances['wall_south']
        elif 'wall_north' in wall_distances:
            y = self.room_height - wall_distances['wall_north']
        elif 'wall_south' in wall_distances:
            y = wall_distances['wall_south']
        
        if x is not None and y is not None:
            return (x, y)
        return None
    
    def get_position(self):
        """Get current robot position using landmarks"""
        if not self.latest_scan:
            return None
        
        # Try cylinder-based localization first
        cylinders = self.detect_cylinders(self.latest_scan)
        cyl_pos = self.calculate_position_from_cylinders(cylinders)
        
        if cyl_pos:
            return cyl_pos
        
        # For precision landmark testing, don't fall back to wall-based positioning
        # as it uses room-scale coordinates instead of local test area coordinates
        print("   LANDMARK POSITIONING FAILED: Insufficient landmarks for trilateration")
        return None

class LocalizationComparison(RectanglePathController):
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):
        super().__init__(serial_port, baud_rate)
        
        # Initialize ROS2 for LiDAR data
        rclpy.init()
        self.landmark_tracker = LandmarkTracker()
        
        # Start ROS2 spinning in background thread
        import threading
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_shutdown = threading.Event()
        self.ros_thread.start()
        
        # Data storage for comparison
        self.dead_reckoning_positions = []
        self.landmark_positions = []
        self.ground_truth_positions = []
        self.intended_corners = []
        
        print("Localization Comparison System Initialized")
        print("Make sure your landmarks are placed and measured!")
    
    def _spin_ros(self):
        """Background thread to spin ROS2 node for LiDAR data"""
        while not self.ros_shutdown.is_set() and rclpy.ok():
            rclpy.spin_once(self.landmark_tracker, timeout_sec=0.1)
    
    def get_manual_measurement(self, corner_num):
        """Get ground truth position via manual measurement"""
        print(f"\nMANUAL MEASUREMENT - Corner {corner_num}")
        print("   Use tape measure from room reference points")
        print("   Measure to center of robot")
        
        while True:
            try:
                x = float(input(f"   X coordinate (meters): "))
                y = float(input(f"   Y coordinate (meters): "))
                
                confirm = input(f"   Confirm position ({x:.2f}, {y:.2f})? (y/n): ")
                if confirm.lower() == 'y':
                    return (x, y)
            except ValueError:
                print("   ERROR: Please enter valid numbers")
    
    def calculate_dead_reckoning_position(self):
        """Calculate position from encoder data using proper dead reckoning"""
        pos_data = self.get_current_position()
        if not pos_data:
            return (0.0, 0.0)
        
        current_left = pos_data['left_ticks']
        current_right = pos_data['right_ticks']
        
        # Initialize dead reckoning state if first call
        if not hasattr(self, '_dr_last_left') or not hasattr(self, '_dr_last_right'):
            self._dr_last_left = current_left
            self._dr_last_right = current_right
            self._dr_x = 0.0
            self._dr_y = 0.0
            self._dr_theta = 0.0  # Start facing forward (0 radians)
            return (0.0, 0.0)
        
        # Calculate change in encoder ticks since last update
        delta_left = current_left - self._dr_last_left
        delta_right = current_right - self._dr_last_right
        
        # Convert ticks to distances (meters)
        left_distance = delta_left * self.meters_per_tick
        right_distance = delta_right * self.meters_per_tick
        
        # Calculate robot motion using differential drive kinematics
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheelbase
        
        # Update robot pose
        delta_x = distance * math.cos(self._dr_theta + delta_theta / 2.0)
        delta_y = distance * math.sin(self._dr_theta + delta_theta / 2.0)
        
        self._dr_x += delta_x
        self._dr_y += delta_y
        self._dr_theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        while self._dr_theta > math.pi:
            self._dr_theta -= 2.0 * math.pi
        while self._dr_theta < -math.pi:
            self._dr_theta += 2.0 * math.pi
        
        # Update last encoder values
        self._dr_last_left = current_left
        self._dr_last_right = current_right
        
        return (self._dr_x, self._dr_y)
    
    def _reset_dead_reckoning(self):
        """Reset dead reckoning state to origin"""
        # Clear any existing state
        if hasattr(self, '_dr_last_left'):
            del self._dr_last_left
        if hasattr(self, '_dr_last_right'):
            del self._dr_last_right
        if hasattr(self, '_dr_x'):
            del self._dr_x
        if hasattr(self, '_dr_y'):
            del self._dr_y
        if hasattr(self, '_dr_theta'):
            del self._dr_theta
        print("Dead reckoning state reset")
    
    def log_corner_with_all_methods(self, corner_num, width, height):
        """Log position using all three methods"""
        print(f"\nCORNER {corner_num} - All Methods Comparison")
        
        # Method 1: Dead Reckoning (existing)
        dr_pos = self.calculate_dead_reckoning_position()
        print(f"   Dead Reckoning: ({dr_pos[0]:.3f}, {dr_pos[1]:.3f})m")
        
        # Method 2: Landmark-Based (new)
        print("Getting landmark position...")
        if corner_num == 0:
            # Starting position is always (0,0) for landmark-based method
            lm_pos = (0.0, 0.0)
            print("   Landmark-Based: Starting position (0.0, 0.0)m")
        else:
            rclpy.spin_once(self.landmark_tracker, timeout_sec=1.0)  # Get fresh LiDAR data
            lm_pos = self.landmark_tracker.get_position()
        
        # Method 3: Ground Truth (manual)
        gt_pos = self.get_manual_measurement(corner_num)
        
        # Store positions
        self.dead_reckoning_positions.append(dr_pos)
        self.landmark_positions.append(lm_pos if lm_pos else (0.0, 0.0))
        self.ground_truth_positions.append(gt_pos)
        
        # Calculate intended position
        if corner_num == 0:
            intended_pos = (0.0, 0.0)
        elif corner_num == 1:
            intended_pos = (width, 0.0)
        elif corner_num == 2:
            intended_pos = (width, height)
        elif corner_num == 3:
            intended_pos = (0.0, height)
        else:  # corner 4 - back to start
            intended_pos = (0.0, 0.0)
        
        self.intended_corners.append(intended_pos)
        
        # Display comparison
        print(f"   Intended:       {intended_pos}")
        print(f"   Dead Reckoning: {dr_pos}")
        print(f"   Landmark-Based: {lm_pos if lm_pos else 'No landmarks detected'}")
        print(f"   Ground Truth:   {gt_pos}")
        
        # Calculate errors
        dr_error = math.sqrt((dr_pos[0] - gt_pos[0])**2 + (dr_pos[1] - gt_pos[1])**2)
        if lm_pos:
            lm_error = math.sqrt((lm_pos[0] - gt_pos[0])**2 + (lm_pos[1] - gt_pos[1])**2)
            print(f"   DR Error: {dr_error:.3f}m, LM Error: {lm_error:.3f}m")
        else:
            print(f"   DR Error: {dr_error:.3f}m, LM Error: N/A")
    
    def execute_comparison_rectangle(self, width, height):
        """Execute rectangle with full comparison tracking"""
        print(f"\nStarting Localization Comparison Test")
        print(f"Rectangle: {width}m × {height}m")
        
        # Reset encoders
        if not self.reset_encoders():
            print("ERROR: Failed to reset encoders")
            return False
        
        # Reset dead reckoning state
        self._reset_dead_reckoning()
        
        # Execute rectangle with comparison at each corner
        sides = [width, height, width, height]
        side_names = ["Width", "Height", "Width", "Height"]
        
        # Log starting position (corner 0)
        time.sleep(1)
        self.log_corner_with_all_methods(0, width, height)
        
        for i, (distance, name) in enumerate(zip(sides, side_names)):
            print(f"\n--- Side {i+1}: {name} ({distance}m) ---")
            
            # Move straight
            if not self.move_straight(distance):
                print(f"ERROR: Failed to move {name}")
                return False
            
            # Log corner position with all methods
            self.log_corner_with_all_methods(i + 1, width, height)
            
            # Turn left (except after last side)
            if i < 3:
                if not self.turn_left_90():
                    print("ERROR: Failed to turn left")
                    return False
                time.sleep(1)
        
        print("\nRectangle navigation completed!")
        return True
    
    def plot_comparison(self):
        """Generate comparison plots with fixed 100x100cm grid"""
        if len(self.ground_truth_positions) < 2:
            print("ERROR: Not enough data for plotting")
            return
        
        # Set up the plot with square aspect ratio for 100x100cm grid
        plt.figure(figsize=(10, 10))
        
        # Extract coordinates (convert to centimeters for better visibility)
        intended_x = [p[0] * 100 for p in self.intended_corners]  # cm
        intended_y = [p[1] * 100 for p in self.intended_corners]  # cm
        
        dr_x = [p[0] * 100 for p in self.dead_reckoning_positions]  # cm
        dr_y = [p[1] * 100 for p in self.dead_reckoning_positions]  # cm
        
        lm_x = [p[0] * 100 for p in self.landmark_positions]  # cm  
        lm_y = [p[1] * 100 for p in self.landmark_positions]  # cm
        
        gt_x = [p[0] * 100 for p in self.ground_truth_positions]  # cm
        gt_y = [p[1] * 100 for p in self.ground_truth_positions]  # cm
        
        # Apply small offsets to prevent line overlap
        offset = 1.5  # 1.5cm offset for visibility
        
        # Create offset versions for dead reckoning and landmark data
        dr_x_offset = [x + offset for x in dr_x]
        dr_y_offset = [y + offset for y in dr_y]
        lm_x_offset = [x - offset for x in lm_x] 
        lm_y_offset = [y - offset for y in lm_y]
        
        # Plot with distinct styles and colors to prevent overlap
        # 1. Intended rectangle (reference - no offset)
        plt.plot(intended_x, intended_y, color='black', linestyle='--', linewidth=4, 
                marker='s', markersize=12, markerfacecolor='white', markeredgecolor='black',
                markeredgewidth=2, label='Intended Rectangle', alpha=0.9, zorder=1)
        
        # 2. Ground Truth (most important - thickest, no offset)
        plt.plot(gt_x, gt_y, color='darkgreen', linestyle='-', linewidth=5, 
                marker='*', markersize=16, markerfacecolor='gold', markeredgecolor='darkgreen',
                markeredgewidth=2, label='Ground Truth', alpha=1.0, zorder=4)
        
        # 3. Dead Reckoning (offset slightly to show)
        plt.plot(dr_x_offset, dr_y_offset, color='blue', linestyle='-', linewidth=3, 
                marker='o', markersize=10, markerfacecolor='lightblue', markeredgecolor='blue',
                markeredgewidth=2, label='Dead Reckoning', alpha=0.9, zorder=3)
        
        # 4. Landmark-Based (different offset)
        plt.plot(lm_x_offset, lm_y_offset, color='red', linestyle='-.', linewidth=3, 
                marker='^', markersize=11, markerfacecolor='pink', markeredgecolor='red',
                markeredgewidth=2, label='Landmark-Based', alpha=0.9, zorder=2)
        
        # Add corner numbers using ground truth positions
        for i, (x, y) in enumerate([(p[0]*100, p[1]*100) for p in self.ground_truth_positions]):
            plt.annotate(f'C{i}', (x, y), xytext=(12, 12), textcoords='offset points',
                        fontsize=12, fontweight='bold', color='darkred',
                        bbox=dict(boxstyle='round,pad=0.4', facecolor='white', 
                                edgecolor='darkred', alpha=0.9, linewidth=1.5))
        
        # Add a legend explaining the offset
        plt.figtext(0.02, 0.02, 'Note: Dead Reckoning and Landmark paths slightly offset for visibility', 
                   fontsize=10, style='italic', alpha=0.7)
        
        # Set FIXED 100x100cm grid with 10cm increments
        ax = plt.gca()
        
        # Force axis limits to exactly 100x100cm
        ax.set_xlim(-5, 105)  # -5 to 105cm to show a bit of border
        ax.set_ylim(-5, 105)  # -5 to 105cm to show a bit of border
        
        # Major ticks every 10cm (0, 10, 20, ... 100)
        major_ticks = np.arange(0, 110, 10)
        ax.set_xticks(major_ticks)
        ax.set_yticks(major_ticks)
        
        # Minor ticks every 5cm for finer grid
        minor_ticks = np.arange(0, 105, 5)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(minor_ticks, minor=True)
        
        # Grid styling
        ax.grid(True, which="major", alpha=0.6, linewidth=1.0, color='gray')
        ax.grid(True, which="minor", alpha=0.3, linewidth=0.5, linestyle=':', color='gray')
        
        # Labels and formatting
        plt.xlabel('X Position (cm)', fontsize=14, fontweight='bold')
        plt.ylabel('Y Position (cm)', fontsize=14, fontweight='bold')
        plt.title('Robot Localization Comparison\n100×100cm Grid (10cm increments)', 
                 fontsize=16, fontweight='bold', pad=20)
        
        # Legend with better positioning
        plt.legend(fontsize=12, loc='upper right', 
                  bbox_to_anchor=(0.98, 0.98), framealpha=0.9)
        
        # Ensure square aspect ratio
        plt.axis('equal')
        
        # Add border around the plot area
        ax.spines['top'].set_linewidth(2)
        ax.spines['bottom'].set_linewidth(2)
        ax.spines['left'].set_linewidth(2)
        ax.spines['right'].set_linewidth(2)
        
        # Save and show
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'localization_comparison_100x100_{timestamp}.png'
        plt.tight_layout()
        plt.savefig(filename, dpi=300, bbox_inches='tight', facecolor='white')
        plt.show()
        
        print(f"Fixed 100×100cm comparison plot saved as: {filename}")
    
    def calculate_accuracy_metrics(self):
        """Calculate accuracy statistics"""
        if len(self.ground_truth_positions) < 2:
            return {}
        
        dr_errors = []
        lm_errors = []
        
        for i in range(len(self.ground_truth_positions)):
            gt = self.ground_truth_positions[i]
            dr = self.dead_reckoning_positions[i]
            lm = self.landmark_positions[i]
            
            # Calculate errors
            dr_error = math.sqrt((dr[0] - gt[0])**2 + (dr[1] - gt[1])**2)
            lm_error = math.sqrt((lm[0] - gt[0])**2 + (lm[1] - gt[1])**2)
            
            dr_errors.append(dr_error)
            lm_errors.append(lm_error)
        
        results = {
            'dead_reckoning_errors': dr_errors,
            'landmark_based_errors': lm_errors,
            'dr_avg_error': np.mean(dr_errors),
            'lm_avg_error': np.mean(lm_errors),
            'dr_max_error': np.max(dr_errors),
            'lm_max_error': np.max(lm_errors),
            'winner': 'Dead Reckoning' if np.mean(dr_errors) < np.mean(lm_errors) else 'Landmark-Based'
        }
        
        return results
    
    def _convert_to_json_serializable(self, obj):
        """Convert numpy and other non-serializable types to JSON-friendly formats"""
        if isinstance(obj, dict):
            return {key: self._convert_to_json_serializable(value) for key, value in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [self._convert_to_json_serializable(item) for item in obj]
        elif isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif hasattr(obj, 'item'):  # Handle numpy scalars
            return obj.item()
        else:
            return obj
    
    def save_comparison_results(self, width, height):
        """Save detailed comparison results"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"localization_comparison_{timestamp}.json"
        
        accuracy_metrics = self.calculate_accuracy_metrics()
        
        results = {
            'test_info': {
                'rectangle_dimensions': {'width': float(width), 'height': float(height)},
                'timestamp': timestamp,
                'test_type': 'localization_comparison'
            },
            'position_data': {
                'intended_corners': self.intended_corners,
                'dead_reckoning_positions': self.dead_reckoning_positions,
                'landmark_based_positions': self.landmark_positions,
                'ground_truth_positions': self.ground_truth_positions
            },
            'accuracy_metrics': accuracy_metrics,
            'robot_parameters': {
                'wheel_radius': float(self.wheel_radius),
                'wheelbase': float(self.wheelbase),
                'ticks_per_rev': int(self.ticks_per_rev),
                'meters_per_tick': float(self.meters_per_tick)
            }
        }
        
        # Convert all data to JSON-serializable format
        results = self._convert_to_json_serializable(results)
        
        try:
            with open(filename, 'w') as f:
                json.dump(results, f, indent=2)
            print(f"Comparison results saved to: {filename}")
            
            # Print summary
            print(f"\nACCURACY SUMMARY:")
            print(f"   Dead Reckoning - Avg Error: {accuracy_metrics['dr_avg_error']:.3f}m")
            print(f"   Landmark-Based - Avg Error: {accuracy_metrics['lm_avg_error']:.3f}m")
            print(f"   Winner: {accuracy_metrics['winner']}")
            
        except Exception as e:
            print(f"ERROR: Failed to save results: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        self.close()  # Close serial connection
        
        # Stop ROS2 spinning
        self.ros_shutdown.set()
        if hasattr(self, 'ros_thread'):
            self.ros_thread.join(timeout=1.0)
        
        if rclpy.ok():
            self.landmark_tracker.destroy_node()
            rclpy.shutdown()

def main():
    comparison = None
    try:
        # Initialize comparison system
        comparison = LocalizationComparison()
        
        # Get rectangle dimensions
        width, height = comparison.get_rectangle_dimensions()
        if width is None:
            return
        
        print(f"\nSETUP CHECKLIST:")
        print(f"   ESP32 connected and running")  
        print(f"   LiDAR spinning and publishing to /scan")
        
        # Auto-discovery option
        use_auto = input(f"\nAuto-discover landmark positions? (y/n): ")
        if use_auto.lower().startswith('y'):
            if comparison.landmark_tracker.auto_discover_landmarks():
                print(f"   Landmarks auto-mapped!")
            else:
                print(f"   Auto-discovery failed, using manual coordinates")
        else:
            print(f"   Using manual landmark coordinates in code")
        
        ready = input(f"\nReady to start comparison test? (y/n): ")
        if ready.lower() != 'y':
            print("Test cancelled")
            return
        
        # Execute rectangle with comparison
        success = comparison.execute_comparison_rectangle(width, height)
        
        if success:
            # Generate plots and save results
            comparison.plot_comparison()
            comparison.save_comparison_results(width, height)
            print("\nLocalization comparison completed successfully!")
        else:
            print("\nComparison test failed")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if comparison:
            comparison.cleanup()

if __name__ == '__main__':
    main()

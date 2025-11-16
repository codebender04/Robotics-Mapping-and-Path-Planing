#!/usr/bin/env python3
"""
Auto-Landmark Map Creator - Handles Different Cylinder Sizes
- Automatically detects ANY 2 cylinders regardless of size
- Measures each cylinder's actual radius
- No size assumptions needed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import pickle
from datetime import datetime
from scipy import ndimage
import cv2
import math

class AutoLandmarkMapCreator(Node):
    def __init__(self):
        super().__init__('auto_landmark_map_creator')  # FIXED THIS LINE
        
        # Fixed 30x30 grid parameters
        self.grid_size = 30
        self.map_resolution = 0.1  # 10cm per cell
        
        # LiDAR configuration for RPLidar A1 pointing LEFT
        self.lidar_yaw_offset = np.pi / 2
        
        # Landmark storage (will be auto-discovered)
        self.landmarks = {}  # {'cylinder_1': {'position': (x,y), 'radius': r}, ...}
        self.landmarks_discovered = False
        
        # No fixed radius assumptions - will measure actual sizes
        self.min_cylinder_radius = 0.02  # 2cm minimum (very small)
        self.max_cylinder_radius = 0.20  # 20cm maximum (quite large)
        
        # Map storage
        self.individual_maps = []
        self.final_binary_map = None
        
        # Current state
        self.current_scan = None
        self.is_recording = False
        self.recorded_scans = []
        
        # Subscriber
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, lidar_qos
        )
        
        self._initialize_grid()
        self.get_logger().info("🗺️ Auto-Landmark Map Creator Ready")
        self.get_logger().info("🔍 Will detect ANY 2 cylinders of ANY size")
    
    def _initialize_grid(self):
        """Initialize an empty 30x30 grid"""
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.grid_origin = np.array([-1.5, -1.5])
    
    def lidar_callback(self, msg):
        """Process incoming LIDAR data"""
        self.current_scan = msg
        if self.is_recording:
            robot_pose = self.get_robot_pose_from_landmarks()
            self.recorded_scans.append({
                'scan': msg,
                'timestamp': time.time(),
                'pose': robot_pose
            })
    
    def auto_discover_landmarks(self):
        """Use the proven detection method from localization"""
        if not self.current_scan:
            self.get_logger().warn("No LiDAR data for landmark discovery")
            return False
        
        self.get_logger().info("🕵️ Auto-discovering landmarks using proven detection...")
        
        # Use the proven detection method
        cylinders = self.detect_cylinders(self.current_scan)
        
        if len(cylinders) < 2:
            self.get_logger().warn(f"Only found {len(cylinders)} cylinders, need 2")
            return False
        
        # Use the detected cylinders as landmarks
        self.landmarks = {}
        for i, cylinder in enumerate(cylinders):
            landmark_id = f'cylinder_{i+1}'
            
            # Convert from robot-relative to global coordinates
            # During discovery, robot is at (0,0) facing 0°
            global_x = cylinder['distance'] * math.cos(cylinder['angle'])
            global_y = cylinder['distance'] * math.sin(cylinder['angle'])
            
            self.landmarks[landmark_id] = {
                'position': (global_x, global_y),
                'radius': cylinder['radius'],
                'quality': cylinder['quality']
            }
            
            self.get_logger().info(f"✅ {landmark_id}: position=({global_x:.2f}, {global_y:.2f}), radius={cylinder['radius']:.3f}m")
        
        self.landmarks_discovered = True
        self.get_logger().info("🎯 Landmark discovery complete! Ready for mapping.")
        return True
    
    def detect_cylinders(self, scan):
        """Copy from your working localization script"""
        if not scan:
            return []

        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        angles = angles + self.lidar_yaw_offset  # Use class variable
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # Convert to Cartesian coordinates
        valid_mask = ~(np.isinf(ranges) | np.isnan(ranges) | (ranges < 0.05))
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
                if (0.01 < radius < 0.5 and  # Broader cylinder radius (1-30cm)
                    0.2 < distance_to_center < 12.0 and  # Broader distance range
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
        """Copy from your working localization script"""
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
        """Copy from your working localization script"""
        x_points = np.array(arc['x_points'])
        y_points = np.array(arc['y_points'])
        
        if len(x_points) < 3:
            return None
        
        try:
            # Use algebraic circle fitting (Kasa method)
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
    
    def group_and_average_cylinders(self, all_detections):
        """Group similar cylinder detections and average their properties"""
        if not all_detections:
            return []
        
        # Group by similar position (angle and distance)
        groups = []
        used_indices = set()
        
        for i, det in enumerate(all_detections):
            if i in used_indices:
                continue
                
            group = [det]
            used_indices.add(i)
            
            for j, other_det in enumerate(all_detections):
                if j in used_indices:
                    continue
                
                # Check if this detection is for the same physical object
                angle_diff = abs(det['angle'] - other_det['angle'])
                if angle_diff > math.pi:
                    angle_diff = 2*math.pi - angle_diff
                
                dist_diff = abs(det['distance'] - other_det['distance'])
                
                if angle_diff < 0.2 and dist_diff < 0.3:  # Same object
                    group.append(other_det)
                    used_indices.add(j)
            
            groups.append(group)
        
        # Average properties for each group
        unique_cylinders = []
        for group in groups:
            if len(group) >= 2:  # Only keep consistently detected objects
                avg_angle = np.mean([d['angle'] for d in group])
                avg_distance = np.mean([d['distance'] for d in group])
                avg_radius = np.mean([d['radius'] for d in group])
                avg_quality = np.mean([d['quality'] for d in group])
                
                unique_cylinders.append({
                    'avg_angle': avg_angle,
                    'avg_distance': avg_distance,
                    'avg_radius': avg_radius,
                    'avg_quality': avg_quality,
                    'detection_count': len(group),
                    'radius_std': np.std([d['radius'] for d in group])
                })
        
        return unique_cylinders
    
    def get_robot_pose_from_landmarks(self):
        """Get robot position using auto-discovered landmarks"""
        if not self.landmarks_discovered or len(self.landmarks) < 2:
            return Pose2D(x=0.0, y=0.0, theta=0.0)
        
        if self.current_scan is None:
            return Pose2D(x=0.0, y=0.0, theta=0.0)
        
        # Use the proven detection method
        cylinders = self.detect_cylinders(self.current_scan)
        
        if len(cylinders) < 2:
            return Pose2D(x=0.0, y=0.0, theta=0.0)
        
        # Match detected cylinders to known landmarks
        landmark_positions = []
        detected_distances = []
        
        # Simple matching: assume consistent detection order
        # Sort both by angle for consistent matching
        cylinders.sort(key=lambda cyl: cyl['angle'])
        landmark_items = sorted(self.landmarks.items(), 
                            key=lambda item: math.atan2(item[1]['position'][1], 
                                                        item[1]['position'][0]))
        
        for (landmark_id, landmark_data), cylinder in zip(landmark_items[:2], cylinders[:2]):
            landmark_positions.append(landmark_data['position'])
            detected_distances.append(cylinder['distance'])
        
        if len(landmark_positions) == 2:
            pos = self.improved_trilateration(
                landmark_positions[0], detected_distances[0],
                landmark_positions[1], detected_distances[1]
            )
            
            if pos:
                return Pose2D(x=pos[0], y=pos[1], theta=0.0)
        
        return Pose2D(x=0.0, y=0.0, theta=0.0)
    
    def improved_trilateration(self, p1, d1, p2, d2):
        """Improved trilateration"""
        x1, y1 = p1
        x2, y2 = p2
        
        landmark_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if (d1 + d2 < landmark_dist or abs(d1 - d2) > landmark_dist or landmark_dist == 0):
            return None
        
        try:
            a = (d1**2 - d2**2 + landmark_dist**2) / (2 * landmark_dist)
            h = math.sqrt(d1**2 - a**2)
            
            if h < 0 or math.isnan(h):
                return None
            
            x0 = x1 + a * (x2 - x1) / landmark_dist
            y0 = y1 + a * (y2 - y1) / landmark_dist
            
            x3_1 = x0 + h * (y2 - y1) / landmark_dist
            y3_1 = y0 - h * (x2 - x1) / landmark_dist
            
            x3_2 = x0 - h * (y2 - y1) / landmark_dist
            y3_2 = y0 + h * (x2 - x1) / landmark_dist
            
            # Choose solution closer to origin (usually more reasonable)
            dist1 = math.sqrt(x3_1**2 + y3_1**2)
            dist2 = math.sqrt(x3_2**2 + y3_2**2)
            
            return (x3_1, y3_1) if dist1 < dist2 else (x3_2, y3_2)
                
        except Exception:
            return None
    
    # ... (rest of the methods: scan_to_grid, world_to_grid, record_map, etc.)
    # These remain largely the same as previous versions

    def record_map(self, map_name=None, duration=20):
        """Record a map with auto-landmark positioning"""
        if not self.landmarks_discovered:
            self.get_logger().info("🔍 First, let's auto-discover landmarks...")
            if not self.auto_discover_landmarks():
                self.get_logger().error("❌ Landmark discovery failed! Cannot record map.")
                return None
        
        if map_name is None:
            map_name = f"map_{len(self.individual_maps) + 1}"
        
        self.get_logger().info(f"🎥 Recording {map_name} for {duration} seconds")
        self.get_logger().info(f"📍 Using auto-discovered landmarks: {len(self.landmarks)} cylinders")
        
        self.is_recording = True
        self.recorded_scans = []
        
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            time.sleep(0.1)
        
        self.is_recording = False
        
        if len(self.recorded_scans) == 0:
            self.get_logger().warn("❌ No LiDAR data received during recording!")
            return None
        
        probability_grid = self.process_recorded_scans()
        
        # Calculate average position
        positions = [(scan['pose'].x, scan['pose'].y) for scan in self.recorded_scans]
        avg_x = np.mean([p[0] for p in positions])
        avg_y = np.mean([p[1] for p in positions])
        
        individual_map = {
            'name': map_name,
            'probability_grid': probability_grid,
            'timestamp': datetime.now().isoformat(),
            'num_scans': len(self.recorded_scans),
            'average_position': (avg_x, avg_y),
            'landmarks_used': self.landmarks.copy()
        }
        
        self.individual_maps.append(individual_map)
        self.get_logger().info(f"✅ Map recorded: {map_name} at ({avg_x:.3f}, {avg_y:.3f})")
        return probability_grid
    def process_recorded_scans(self):
        """Combine multiple scans into a probability grid"""
        combined_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        scan_count = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        for scan_data in self.recorded_scans:
            scan_grid = self.scan_to_grid(scan_data['scan'], scan_data['pose'])
            combined_grid += scan_grid
            scan_count[scan_grid > 0] += 1
        
        avg_grid = np.zeros_like(combined_grid)
        mask = scan_count > 0
        avg_grid[mask] = combined_grid[mask] / scan_count[mask]
        return avg_grid
    def scan_to_grid(self, scan, robot_pose):
        """Convert LIDAR scan to grid occupancy probabilities"""
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        
        if scan is None:
            return grid
        
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        
        for i, (angle, range_val) in enumerate(zip(angles, scan.ranges)):
            if (scan.range_min <= range_val <= 12.0 and  # MAXIMUM range
                not np.isinf(range_val) and not np.isnan(range_val) and
                range_val > 0.05):  # Just filter noise
                
                # Apply LiDAR yaw offset
                corrected_angle = angle + self.lidar_yaw_offset + robot_pose.theta
                
                # Calculate world coordinates
                world_x = robot_pose.x + range_val * np.cos(corrected_angle)
                world_y = robot_pose.y + range_val * np.sin(corrected_angle)
                
                grid_x, grid_y = self.world_to_grid(world_x, world_y)
                
                if self.is_within_grid(grid_x, grid_y):
                    grid[grid_y, grid_x] = 0.8
                    self.mark_free_cells(grid, robot_pose, world_x, world_y, corrected_angle)
        
        return grid

    def mark_free_cells(self, grid, robot_pose, end_x, end_y, beam_angle):
        """Mark cells along the LIDAR beam as free space"""
        start_x, start_y = robot_pose.x, robot_pose.y
        grid_start_x, grid_start_y = self.world_to_grid(start_x, start_y)
        grid_end_x, grid_end_y = self.world_to_grid(end_x, end_y)
        
        points = self.get_line_points(grid_start_x, grid_start_y, grid_end_x, grid_end_y)
        for px, py in points:
            if self.is_within_grid(px, py) and grid[py, px] == 0:
                grid[py, px] = 0.1

    def get_line_points(self, x0, y0, x1, y1):
        """Get grid points between two points"""
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points

    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_x - self.grid_origin[0]) / self.map_resolution)
        grid_y = int((world_y - self.grid_origin[1]) / self.map_resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.map_resolution + self.grid_origin[0]
        world_y = grid_y * self.map_resolution + self.grid_origin[1]
        return world_x, world_y

    def is_within_grid(self, grid_x, grid_y):
        """Check if grid coordinates are within bounds"""
        return (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size)
    def create_final_binary_map(self, occupancy_threshold=0.4):
        """Combine all maps into final binary map"""
        if len(self.individual_maps) < 1:
            self.get_logger().warn("No maps to combine!")
            return None
        
        self.get_logger().info(f"🔄 Combining {len(self.individual_maps)} maps...")
        
        # Start with first map
        combined_probability = self.individual_maps[0]['probability_grid'].copy()
        
        # Add subsequent maps
        for i in range(1, len(self.individual_maps)):
            combined_probability += self.individual_maps[i]['probability_grid']
        
        # Average the probabilities
        combined_probability /= len(self.individual_maps)
        
        # Convert to binary using threshold
        self.final_binary_map = (combined_probability >= occupancy_threshold).astype(np.uint8)
        
        # Apply noise filtering
        from scipy import ndimage
        self.final_binary_map = ndimage.median_filter(self.final_binary_map, size=2)
        
        self.get_logger().info("✅ Final binary map created!")
        return self.final_binary_map

    def visualize_map(self, binary_map, title="30×30 Map with Auto-Discovered Landmarks"):
        """Visualize the map with discovered landmarks"""
        plt.figure(figsize=(12, 10))
        
        from matplotlib.colors import ListedColormap
        cmap = ListedColormap(['black', 'white'])
        
        plt.imshow(binary_map, cmap=cmap, origin='lower', 
                  extent=[self.grid_origin[0], 
                         self.grid_origin[0] + self.grid_size * self.map_resolution,
                         self.grid_origin[1],
                         self.grid_origin[1] + self.grid_size * self.map_resolution],
                  alpha=0.7)
        
        # Add grid lines
        for i in range(self.grid_size + 1):
            x_pos = self.grid_origin[0] + i * self.map_resolution
            y_pos = self.grid_origin[1] + i * self.map_resolution
            plt.axvline(x=x_pos, color='#666666', linewidth=1.0, alpha=0.8)
            plt.axhline(y=y_pos, color='#666666', linewidth=1.0, alpha=0.8)
        
        # Add discovered landmark positions
        for landmark_id, landmark_data in self.landmarks.items():
            x, y = landmark_data['position']
            radius = landmark_data['radius']
            
            # Plot landmark position
            plt.plot(x, y, 'ro', markersize=12, markeredgecolor='black', markeredgewidth=2)
            
            # Draw circle showing actual measured radius
            circle = plt.Circle((x, y), radius, color='red', fill=False, linestyle='--', linewidth=2)
            plt.gca().add_patch(circle)
            
            plt.text(x, y + 0.15, f"{landmark_id}\nr={radius:.3f}m", 
                    ha='center', va='bottom', fontweight='bold', fontsize=9,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.title(title, fontsize=16, fontweight='bold', pad=20)
        plt.xlabel('X (meters)', fontsize=12, fontweight='bold')
        plt.ylabel('Y (meters)', fontsize=12, fontweight='bold')
        plt.gca().set_aspect('equal')
        plt.tight_layout()
        plt.show()
    def visualize_partial_maps(self):
        """Visualize all recorded partial maps with orientation arrows"""
        if not self.individual_maps:
            print("❌ No partial maps recorded yet!")
            return
        
        print(f"📊 Visualizing {len(self.individual_maps)} partial maps...")
        
        # Create subplot grid
        num_maps = len(self.individual_maps)
        cols = min(2, num_maps)
        rows = (num_maps + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(cols * 8, rows * 8))
        if num_maps == 1:
            axes = [axes]
        else:
            axes = axes.flatten()
        
        for i, map_data in enumerate(self.individual_maps):
            ax = axes[i]
            probability_grid = map_data['probability_grid']
            
            # Create colormap: blue (low prob) -> red (high prob) -> white (occupied)
            from matplotlib.colors import LinearSegmentedColormap
            colors = ['blue', 'cyan', 'green', 'yellow', 'orange', 'red', 'white']
            cmap = LinearSegmentedColormap.from_list('probability', colors, N=100)
            
            # Plot the probability grid
            im = ax.imshow(probability_grid, cmap=cmap, origin='lower', 
                        extent=[self.grid_origin[0], 
                                self.grid_origin[0] + self.grid_size * self.map_resolution,
                                self.grid_origin[1],
                                self.grid_origin[1] + self.grid_size * self.map_resolution],
                        vmin=0, vmax=1)
            
            # Add grid lines
            for j in range(self.grid_size + 1):
                x_pos = self.grid_origin[0] + j * self.map_resolution
                y_pos = self.grid_origin[1] + j * self.map_resolution
                ax.axvline(x=x_pos, color='gray', linewidth=0.5, alpha=0.7)
                ax.axhline(y=y_pos, color='gray', linewidth=0.5, alpha=0.7)
            
            # Add robot position and orientation
            avg_pos = map_data['average_position']
            
            # Robot position (red)
            ax.plot(avg_pos[0], avg_pos[1], 'ro', markersize=12, markeredgecolor='darkred', 
                    markeredgewidth=2, label='Robot Position')
            
            # Robot front orientation (red arrow)
            robot_arrow_length = 0.3
            ax.arrow(avg_pos[0], avg_pos[1], robot_arrow_length, 0, 
                    head_width=0.08, head_length=0.08, fc='red', ec='darkred', 
                    linewidth=3, label='Robot Front')
            
            # LiDAR front orientation (blue arrow) - 90° offset since LiDAR points left
            lidar_arrow_length = 0.25
            ax.arrow(avg_pos[0], avg_pos[1], 0, lidar_arrow_length, 
                    head_width=0.07, head_length=0.07, fc='blue', ec='darkblue', 
                    linewidth=2, label='LiDAR Front')
            
            # Add title and info
            ax.set_title(f"Map: {map_data['name']}\nPosition: ({avg_pos[0]:.2f}, {avg_pos[1]:.2f})\nScans: {map_data['num_scans']}", 
                        fontsize=12, fontweight='bold')
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.legend()
            
            # Add colorbar
            plt.colorbar(im, ax=ax, label='Occupancy Probability')
        
        # Hide unused subplots
        for i in range(num_maps, len(axes)):
            axes[i].set_visible(False)
        
        plt.tight_layout()
        plt.suptitle(f'Partial Maps - Raw LiDAR Data ({num_maps} recordings)', 
                    fontsize=16, fontweight='bold', y=1.02)
        
        # Save to file and show
        plt.savefig('partial_maps_with_orientation.png', dpi=150, bbox_inches='tight')
        print("✅ Saved visualization to partial_maps_with_orientation.png")
        plt.show(block=True)
    def save_binary_map(self, filename=None):
        """Save the binary map to file"""
        if self.final_binary_map is None:
            self.get_logger().warn("No binary map to save!")
            return False
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"final_map_{timestamp}"
        
        # Save as pickle with all data
        map_data = {
            'binary_map': self.final_binary_map,
            'resolution': self.map_resolution,
            'origin': self.grid_origin,
            'grid_size': self.grid_size,
            'landmarks': self.landmarks,
            'individual_maps': self.individual_maps,
            'timestamp': datetime.now().isoformat()
        }
        
        with open(f"{filename}.pkl", 'wb') as f:
            pickle.dump(map_data, f)
        
        # Save visualization
        plt.figure(figsize=(10, 10))
        from matplotlib.colors import ListedColormap
        cmap = ListedColormap(['black', 'white'])
        plt.imshow(self.final_binary_map, cmap=cmap, origin='lower', alpha=0.8)
        
        # Add grid lines
        for i in range(self.grid_size + 1):
            plt.axvline(x=i, color='#666666', linewidth=1.0, alpha=0.6)
            plt.axhline(y=i, color='#666666', linewidth=1.0, alpha=0.6)
        
        # Add orientation indicators
        plt.arrow(15, 15, 3, 0, head_width=1, head_length=1, fc='red', ec='red', linewidth=2)
        plt.arrow(15, 15, 0, 3, head_width=1, head_length=1, fc='blue', ec='blue', linewidth=2)
        plt.text(20, 15, 'Robot Front', color='red', fontsize=10, fontweight='bold')
        plt.text(15, 20, 'LiDAR Front', color='blue', fontsize=10, fontweight='bold')
        
        plt.title('Final Combined Map')
        plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f"💾 Map saved as {filename}.pkl and {filename}.png")
        return True

def main():
    rclpy.init()
    creator = AutoLandmarkMapCreator()
    
    def spin_node():
        rclpy.spin(creator)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        print("\n" + "="*60)
        print("🗺️  AUTO-LANDMARK MAP CREATOR")
        print("="*60)
        print("Detects ANY 2 cylinders of ANY size")
        print("No pre-specified positions needed")
        print("Automatically measures each cylinder's actual radius")
        
        while rclpy.ok():
            print("\nOptions:")
            print("1. Auto-discover landmarks (DO THIS FIRST)")
            print("2. Record new map (20 seconds)") 
            print("3. Combine all maps → Create binary map")
            print("4. Visualize current binary map")
            print("5. Visualize partial maps (RAW LiDAR DATA)")  # ADD THIS LINE
            print("6. Save binary map")
            print("7. Exit")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == '1':
                success = creator.auto_discover_landmarks()
                if success:
                    print("✅ Landmark discovery successful!")
                else:
                    print("❌ Landmark discovery failed. Check cylinder visibility.")
                    
            elif choice == '2':
                if not creator.landmarks_discovered:
                    print("❌ Discover landmarks first!")
                    continue
                map_name = input("Enter map name: ").strip() or None
                result = creator.record_map(map_name, duration=20)
                if result is None:
                    print("❌ Recording failed!")
                    
            elif choice == '3':
                if len(creator.individual_maps) >= 1:
                    binary_map = creator.create_final_binary_map()
                    if binary_map is not None:
                        print("✅ Binary map created!")
                        creator.visualize_map(binary_map)
                else:
                    print("❌ Record at least one map first!")
                    
            elif choice == '4':
                if creator.final_binary_map is not None:
                    creator.visualize_map(creator.final_binary_map)
                else:
                    print("❌ Create binary map first!")
                    
            elif choice == '5':  # ADD THIS
                creator.visualize_partial_maps()
                    
            elif choice == '6':
                if creator.final_binary_map is not None:
                    filename = input("Enter filename: ").strip()
                    creator.save_binary_map(filename)
                else:
                    print("❌ No binary map to save!")
                    
            elif choice == '7':
                print("👋 Exiting...")
                break
                
            else:
                print("❌ Invalid choice!")
                
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        creator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

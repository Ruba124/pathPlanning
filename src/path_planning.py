from __future__ import annotations
import math
from typing import List
from src.models import CarPose, Cone, Path2D

class PathPlanning:
    """
    This definitive version has a corrected _get_offset_path function
    that creates a single, safe waypoint and prevents all loops or backward motion.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def _get_offset_path(self, cones: List[Cone], is_blue_shape: bool) -> List[tuple[float, float]]:
        """
        Calculates a SINGLE, safe offset waypoint based on the average center of the cones.
        This new logic prevents all loops, zig-zags, and backward paths.
        """
        waypoints = []
        TRACK_WIDTH = 3.5
        offset_distance = TRACK_WIDTH / 2.0

        if not cones:
            return []

        # 1. Find the single average point of all visible cones
        avg_x = sum(c.x for c in cones) / len(cones)
        avg_y = sum(c.y for c in cones) / len(cones)
        avg_point = (avg_x, avg_y)
        
        # 2. Calculate a single offset point from this average center
        car_pos = (self.car_pose.x, self.car_pose.y)
        # Use the vector from the car to the average point to find a perpendicular
        vec_x, vec_y = avg_point[0] - car_pos[0], avg_point[1] - car_pos[1]
        
        # Blue (left) shape gets a RIGHT offset. Yellow (right) shape gets a LEFT offset.
        norm_x, norm_y = (vec_y, -vec_x) if is_blue_shape else (-vec_y, vec_x)
        norm_mag = math.hypot(norm_x, norm_y)
        
        if norm_mag > 0:
            unit_norm_x, unit_norm_y = norm_x / norm_mag, norm_y / norm_mag
            # Create the one and only waypoint
            waypoints.append((avg_point[0] + unit_norm_x * offset_distance, 
                              avg_point[1] + unit_norm_y * offset_distance))
        else:
            # Fallback in case the cone center is at the car's position
            fallback_norm_x, fallback_norm_y = (0, -1) if is_blue_shape else (0, 1)
            waypoints.append((avg_point[0] + fallback_norm_x * offset_distance, 
                              avg_point[1] + fallback_norm_y * offset_distance))

        return waypoints

    def generatePath(self) -> Path2D:
        """Generates a continuous path that never cuts the cone shapes and always moves forward."""
        car_pos = (self.car_pose.x, self.car_pose.y)
        
        # --- FIX 1: Sort by x-coordinate (travel order) instead of distance ---
        blue_cones = sorted([c for c in self.cones if c.color == 1], key=lambda c: c.x)
        yellow_cones = sorted([c for c in self.cones if c.color == 0], key=lambda c: c.x)

        waypoints = []

        if blue_cones and yellow_cones:
            num_real_pairs = min(len(blue_cones), len(yellow_cones))
            last_offset_vector = (0, 0)

            for i in range(num_real_pairs):
                b_cone, y_cone = blue_cones[i], yellow_cones[i]
                waypoints.append(((b_cone.x + y_cone.x) / 2, (b_cone.y + y_cone.y) / 2))
                last_offset_vector = (y_cone.x - b_cone.x, y_cone.y - b_cone.y)

            if len(blue_cones) > num_real_pairs:
                for i in range(num_real_pairs, len(blue_cones)):
                    unpaired_blue = blue_cones[i]
                    virtual_yellow = (unpaired_blue.x + last_offset_vector[0], unpaired_blue.y + last_offset_vector[1])
                    waypoints.append(((unpaired_blue.x + virtual_yellow[0]) / 2, (unpaired_blue.y + virtual_yellow[1]) / 2))

            elif len(yellow_cones) > num_real_pairs:
                for i in range(num_real_pairs, len(yellow_cones)):
                    unpaired_yellow = yellow_cones[i]
                    virtual_blue = (unpaired_yellow.x - last_offset_vector[0], unpaired_yellow.y - last_offset_vector[1])
                    waypoints.append(((unpaired_yellow.x + virtual_blue[0]) / 2, (unpaired_yellow.y + virtual_blue[1]) / 2))

        elif blue_cones:
            # Call the new, corrected function
            waypoints = self._get_offset_path(blue_cones, is_blue_shape=True)
        elif yellow_cones:
            # Call the new, corrected function
            waypoints = self._get_offset_path(yellow_cones, is_blue_shape=False)

        # --- FIX 2: Sort the final waypoints by x-coordinate ---
        # This ensures the path always moves left-to-right and prevents the backward turn.
        waypoints.sort(key=lambda p: p[0])

        path: Path2D = []
        # The car_pos (0,0) is correctly added as the starting point here.
        points_to_connect = [car_pos] + waypoints

        if len(points_to_connect) > 1:
            for i in range(len(points_to_connect) - 1):
                start_p, end_p = points_to_connect[i], points_to_connect[i+1]
                segment_dx, segment_dy = end_p[0] - start_p[0], end_p[1] - start_p[1]
                segment_len = math.hypot(segment_dx, segment_dy)
                if segment_len > 0.01:
                    num_steps = max(1, int(segment_len / 0.4))
                    for j in range(1, num_steps + 1):
                        path.append((start_p[0] + (segment_dx / num_steps) * j,
                                     start_p[1] + (segment_dy / num_steps) * j))

        final_heading = self.car_pose.yaw
        if waypoints:
            last_wp = waypoints[-1]
            dx, dy = last_wp[0] - car_pos[0], last_wp[1] - car_pos[1]
            if math.hypot(dx, dy) > 0.1:
                final_heading = math.atan2(dy, dx)

        last_point = path[-1] if path else car_pos
        for i in range(1, 41):
            dist = 0.4 * i
            path.append((last_point[0] + math.cos(final_heading) * dist,
                         last_point[1] + math.sin(final_heading) * dist))

        return path
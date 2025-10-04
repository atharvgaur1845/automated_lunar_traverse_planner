#!/usr/bin/env python3
"""
YOLO-based crater detection with tiered navigation and a safe-start protocol.
"""

import numpy as np
import os
import math
from pathlib import Path
from typing import List, Tuple
from dataclasses import dataclass
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import matplotlib.patches as mpatches

# Configuration
CONFIG = {
    'min_traverse_distance': 300.0,
    'crater_avoidance_radius': 2.0,
    'path_safety_margin': 0.5,      
    'min_science_stops': 20,
    'map_size_meters': 100.0,
    'sun_elevation': 25.0,
    'sun_azimuth': 180.0,
    'solar_efficiency_threshold': 0.9,
}

@dataclass
class CraterDetection:
    center_x: float; center_y: float; radius: float; confidence: float; class_id: int

@dataclass
class ScientificWaypoint:
    coordinates: Tuple[float, float]; stop_id: int; science_type: str
    priority_score: float; instruments: List[str]; estimated_duration: int
    illumination_status: str; crater_proximity: float; color: str

class LunarNavigation:
    def __init__(self, output_dir: Path, map_name: str):
        self.map_name = map_name
        self.output_dir = output_dir / self.map_name
        self.output_dir.mkdir(exist_ok=True)
        (self.output_dir / "annotated_images").mkdir(exist_ok=True)

        self.craters: List[CraterDetection] = []
        self.solar_regions = []
        self.potential_waypoints: List[ScientificWaypoint] = []
        self.visited_waypoints: List[ScientificWaypoint] = []
        self.traverse_path: List[Tuple[float, float]] = []
        
        map_edge = CONFIG['map_size_meters'] / 2 - 5.0
        self.start_point = (-map_edge, -map_edge)

    def load_yolo_data_for_map(self, annotation_path: Path) -> bool:
        print(f"Loading data for map: {self.map_name}...")
        try:
            with open(annotation_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        _, x_norm, y_norm, w_norm, h_norm = [float(p) for p in parts[:5]]
                        x_m = (x_norm - 0.5) * CONFIG['map_size_meters']
                        y_m = (0.5 - y_norm) * CONFIG['map_size_meters']
                        rad_m = (w_norm + h_norm) * CONFIG['map_size_meters'] / 4.0
                        self.craters.append(CraterDetection(
                            center_x=x_m, center_y=y_m, radius=rad_m,
                            confidence=float(parts[5]) if len(parts) > 5 else 1.0, class_id=0))
            print(f"Successfully processed, detected {len(self.craters)} craters.")
            return True
        except Exception as e:
            print(f"Error processing {annotation_path}: {e}"); return False

    def _ensure_safe_start(self):
        map_edge = CONFIG['map_size_meters'] / 2 - 5.0
        while not self._is_point_safe(self.start_point)[0]:
            print(f"Start point {self.start_point} is inside a hazard. Relocating...")
            new_x = self.start_point[0] + 10.0
            if new_x > map_edge:
                print("Could not find a safe start point. Mission aborted for this map.")
                return False # Cannot find a safe start
            self.start_point = (new_x, self.start_point[1])
        print(f"Safe start point confirmed at: {self.start_point}")
        return True
    def generate_solar_illumination_map(self):
        sun_elev_rad=math.radians(CONFIG['sun_elevation']);sun_azim_rad=math.radians(CONFIG['sun_azimuth'])
        if sun_elev_rad<=0:return
        for c in self.craters:
            if c.radius>1.5:
                s_len=(c.radius*0.3)/math.tan(sun_elev_rad);s_dx=-s_len*math.sin(sun_azim_rad);s_dy=-s_len*math.cos(sun_azim_rad)
                v=[(c.center_x+s_dx+c.radius*0.9*math.cos(a),c.center_y+s_dy+c.radius*0.4*math.sin(a))for a in np.linspace(0,2*math.pi,12,endpoint=False)]
                self.solar_regions.append({'polygon':v})
    def calculate_illumination(self,p):
        for r in self.solar_regions:
            if self._point_in_polygon(p,r['polygon']):return 0.15
        return 1.0
    def _point_in_polygon(self,p,poly):
        x,y=p;n=len(poly)
        if n<3:return False
        inside=False;p1x,p1y=poly[0]
        for i in range(n+1):
            p2x,p2y=poly[i%n]
            if y>min(p1y,p2y)and y<=max(p1y,p2y)and x<=max(p1x,p2x):
                if p1y!=p2y:xinters=(y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                if p1x==p2x or x<=xinters:inside=not inside
            p1x,p1y=p2x,p2y
        return inside

    def identify_potential_scientific_targets(self):
        targets = []
        map_bound = CONFIG['map_size_meters'] / 2
        for crater in self.craters:
            if crater.radius >= 1.0:
                for angle in np.linspace(0, 2*math.pi, 16, endpoint=False):
                    rim_dist = crater.radius + 2.5
                    rim_x, rim_y = crater.center_x + rim_dist*math.cos(angle), crater.center_y + rim_dist*math.sin(angle)
                    if not (abs(rim_x) < map_bound - 10 and abs(rim_y) < map_bound - 10): continue
                    if self.calculate_illumination((rim_x, rim_y)) > CONFIG['solar_efficiency_threshold']:
                        priority = (crater.confidence*7.0 + (crater.radius/5.0)*2.0)
                        targets.append(ScientificWaypoint(coordinates=(rim_x, rim_y), stop_id=0, science_type='crater_rim_geology',
                            priority_score=priority, instruments=['Spectrometer'], estimated_duration=50,
                            illumination_status='sunlit', crater_proximity=rim_dist, color="#001aff"))
        for x in np.arange(-map_bound + 10, map_bound - 10, 15):
            for y in np.arange(-map_bound + 10, map_bound - 10, 15):
                point = (x, y)
                if self.calculate_illumination(point) < CONFIG['solar_efficiency_threshold']: continue
                min_dist = min((abs(math.dist(point, (c.center_x, c.center_y)) - c.radius) for c in self.craters), default=float('inf'))
                if min_dist > 20.0:
                    targets.append(ScientificWaypoint(coordinates=point, stop_id=0, science_type='regolith_sampling', priority_score=6.5,
                        instruments=['Scoop'], estimated_duration=30, illumination_status='sunlit',
                        crater_proximity=min_dist, color='#ff9900'))
        self.potential_waypoints = targets

    def plan_optimal_path(self):
        if not self._ensure_safe_start():
            return

        self.traverse_path = [self.start_point]
        current_pos = self.start_point
        
        while self._calculate_path_distance(self.traverse_path) < CONFIG['min_traverse_distance']:
            # Tier 1: Dynamic Exploration
            best_target = self._find_best_next_target(current_pos)
            if best_target:
                safe_path = self._find_safe_path_to_target(current_pos, best_target.coordinates)
                if safe_path:
                    print(f"Tier 1:routing to S{len(self.visited_waypoints)+1}")
                    self.traverse_path.extend(safe_path[1:])
                    current_pos = self.traverse_path[-1]
                    best_target.stop_id = len(self.visited_waypoints) + 1
                    self.visited_waypoints.append(best_target)
                    self.potential_waypoints.remove(best_target)
                    continue

            # Tier 2: Rotational Search
            print("Tier 1 failed. Engaging Tier 2: Rotational Search")
            escape_path = self._execute_rotational_search(current_pos)
            if escape_path:
                print("Tier 2: Found an escape vector. Moving to new position")
                self.traverse_path.extend(escape_path[1:])
                current_pos = self.traverse_path[-1]
                continue

            # Tier 3: Hazard Breach
            print("Tier 2 failed. Engaging Tier 3: Hazard Breach")
            breach_path = self._execute_hazard_breach(current_pos)
            if breach_path:
                print("WARNING: All paths blocked. Breaching hazard to escape")
                self.traverse_path.extend(breach_path[1:])
                current_pos = self.traverse_path[-1]
                continue

            print("All navigation tiers failed. Rover is trapped. Ending mission.")
            break

    def _execute_rotational_search(self, current_pos):
        for angle in np.linspace(0, 2*math.pi, 8, endpoint=False):
            candidate_point = (current_pos[0] + 15.0 * math.cos(angle),
                               current_pos[1] + 15.0 * math.sin(angle))
            if self._is_point_safe(candidate_point)[0]:
                path = self._find_safe_path_to_target(current_pos, candidate_point)
                if path:
                    return path
        return None

    def _execute_hazard_breach(self, current_pos):
        map_corners = [(x, y) for x in [-50, 50] for y in [-50, 50]]
        escape_target = max(map_corners, key=lambda p: math.dist(current_pos, p))
        
        return self._find_safe_path_to_target(current_pos, escape_target, is_breaching=True)


    def _find_best_next_target(self, current_pos):
        best_target = None; highest_score = -float('inf')
        valid_targets = [wp for wp in self.potential_waypoints if not any(math.dist(wp.coordinates, v.coordinates) < 15.0 for v in self.visited_waypoints)]
        for target in valid_targets:
            score = self._calculate_waypoint_score(current_pos, target)
            if score > highest_score:
                highest_score = score; best_target = target
        return best_target

    def _calculate_path_distance(self, path):
        return sum(math.dist(path[i], path[i-1]) for i in range(1, len(path)))

    def _calculate_waypoint_score(self, current_pos, waypoint):
        dist = math.dist(current_pos, waypoint.coordinates)
        if dist < 1.0: return -float('inf')
        dist_eff = max(0, 100.0 - dist)/100.0; sci_prio = waypoint.priority_score/15.0
        return dist_eff*0.45 + sci_prio*0.55

    def _find_safe_path_to_target(self, start, end, is_breaching=False):
        final_path = [start]; current_pos = start
        for _ in range(20):
            if math.dist(current_pos, end) < 2.0:
                final_path.append(end); return final_path
            points, safety, craters = self._check_line_safety(current_pos, end, is_breaching)
            if all(safety):
                final_path.extend(points[1:]); return final_path
            first_unsafe = safety.index(False); arc_start = points[first_unsafe - 1]
            try:
                first_safe = safety.index(True, first_unsafe); arc_end = points[first_safe]; crater = craters[first_unsafe]
            except ValueError: return None
            arc_points = self._generate_rim_arc(arc_start, arc_end, crater, is_breaching)
            if not arc_points: return None
            final_path.extend(points[1:first_unsafe]); final_path.extend(arc_points)
            current_pos = final_path[-1]
        return None

    def _check_line_safety(self, start, end, is_breaching=False):
        points, status, craters = [], [], []
        dist = math.dist(start, end); steps = int(dist / 2.0) + 1
        for i in range(steps + 1):
            t = i/steps; px, py = start[0]*(1-t)+end[0]*t, start[1]*(1-t)+end[1]*t
            is_safe, crater = self._is_point_safe((px, py), is_breaching)
            points.append((px, py)); status.append(is_safe); craters.append(crater)
        return points, status, craters

    def _generate_rim_arc(self, arc_start, arc_end, crater, is_breaching=False):
        arc_points = []; center = (crater.center_x, crater.center_y); radius = crater.radius + 1.5
        start_angle = math.atan2(arc_start[1]-center[1], arc_start[0]-center[0])
        end_angle = math.atan2(arc_end[1]-center[1], arc_end[0]-center[0])
        if abs(end_angle-start_angle) > math.pi:
            if end_angle > start_angle: start_angle += 2*math.pi
            else: end_angle += 2*math.pi
        for i in range(1, 4):
            frac = i/3.0; angle = start_angle + (end_angle-start_angle)*frac
            px, py = center[0]+radius*math.cos(angle), center[1]+radius*math.sin(angle)
            if not self._is_point_safe((px,py), is_breaching)[0]: return []
            if i < 3: arc_points.append((px, py))
        return arc_points

    def _is_point_safe(self, point, is_breaching=False):
        map_bound = CONFIG['map_size_meters']/2 - 1.0
        if not (abs(point[0]) < map_bound and abs(point[1]) < map_bound): return False, None
        for crater in self.craters:
            dist = math.dist(point, (crater.center_x, crater.center_y))
            is_haz = crater.radius >= CONFIG['crater_avoidance_radius']
            #during a breach,ignore navigable craters but still avoid hazardous ones
            if is_breaching and not is_haz:
                continue
            clearance = crater.radius + CONFIG['path_safety_margin'] if is_haz else crater.radius
            if dist < clearance: return False, crater
        return True, None

    def generate_annotated_visualization(self):
        filename = f"{self.map_name}_traverse_map.png"
        print(f"Generating annotated plt: {filename}")
        fig, ax = plt.subplots(figsize=(18, 18))
        map_size = CONFIG['map_size_meters']/2
        ax.set_facecolor("#ffffff"); ax.set_xlim(-map_size, map_size); ax.set_ylim(-map_size, map_size)
        ax.set_xlabel('East Distance (meters)', fontsize=14); ax.set_ylabel('North Distance (meters)', fontsize=14)
        ax.set_title(f'Lunar Traverse Plan: {self.map_name}\n', fontsize=18, fontweight='bold')
        ax.grid(True, linestyle=':', color='black', alpha=0.3); ax.set_aspect('equal', adjustable='box')
        for region in self.solar_regions: ax.add_patch(Polygon(region['polygon'], alpha=0.5, fc='black', ec=None))
        for c in self.craters:
            is_haz = c.radius >= CONFIG['crater_avoidance_radius']
            fc, ec = ("#ff4242", '#8b0000') if is_haz else ('#a0a0a0', '#404000')
            ax.add_patch(Circle((c.center_x, c.center_y), c.radius, fill=True, fc=fc, alpha=0.8, ec=ec, lw=1.5))
            if is_haz:
                ax.add_patch(Circle((c.center_x, c.center_y), c.radius + CONFIG['path_safety_margin'], fill=False, ec='#ff4d4d', ls='--', alpha=0.9, lw=2))
        ax.plot(self.start_point[0], self.start_point[1], 'p', c='lime', ms=20, mec='black', label='Start Point', zorder=10)
        ax.add_patch(Circle(self.start_point, 5.0, fill=False, ec='magenta', ls='-', alpha=1.0, lw=3, zorder=9))
        main_waypoints_coords = [wp.coordinates for wp in self.visited_waypoints] + [self.start_point]
        other_points = [p for p in self.traverse_path if p not in main_waypoints_coords]
        if other_points:
            px, py = zip(*other_points)
            ax.scatter(px, py, c='magenta', s=50, marker='D', label='Navigational Stop', zorder=7)
        for wp in self.visited_waypoints:
            ax.scatter(wp.coordinates[0], wp.coordinates[1], c=wp.color, s=120, alpha=0.9, ec='black', lw=1.5, zorder=8)
            ax.annotate(f'S{wp.stop_id}', wp.coordinates, xytext=(8, 8), textcoords='offset points', fontsize=11, fontweight='bold')
        if len(self.traverse_path) > 1:
            px, py = zip(*self.traverse_path)
            ax.plot(px, py, '#00ffff', linewidth=4, alpha=0.9, label='Traverse Path', zorder=6, solid_capstyle='round')
        legend_elements = [
            mpatches.Patch(color='#ff4d4d', alpha=0.8, label='Hazardous Crater'), mpatches.Patch(color='none', ec='#ff4d4d', hatch='---', label='Safety Margin'),
            mpatches.Patch(color="#7db873", alpha=0.8, label='Navigable Crater'), plt.Line2D([0], [0], marker='p', color='w', mfc='lime', ms=20, label='Start Point'),
            mpatches.Patch(color='none', ec='magenta', lw=3, label='Landing Zone'),
            mpatches.Patch(color='black', alpha=0.5, label='Crater Shadow'),
            plt.Line2D([0], [0], marker='o', color='w', mfc='#00ff00', ms=12, label='Waypoint (Crater Rim)'),
            plt.Line2D([0], [0], marker='o', color='w', mfc='#ff9900', ms=12, label='Waypoint (Regolith Sample)'),
            plt.Line2D([0], [0], marker='D', color='w', mfc='magenta', ms=10, label='Navigational Stop'),
            plt.Line2D([0], [0], color='#00ffff', linewidth=4, label='Traverse Path')]
        ax.legend(handles=legend_elements, loc='upper left', fontsize=12, frameon=True, facecolor='white', framealpha=0.8)
        output_path = self.output_dir / "annotated_images" / f"{self.map_name}_traverse_map.png"
        plt.savefig(output_path, dpi=250, bbox_inches='tight'); plt.close()


    def execute_full_analysis(self, annotation_path: Path):
        if not self.load_yolo_data_for_map(annotation_path): return
        self.generate_solar_illumination_map()
        self.identify_potential_scientific_targets()
        self.plan_optimal_path()
        self.generate_annotated_visualization()
        print("\nAnalysis Complete.")
        print(f"Map Name: {self.map_name}")
        print(f"Total Traverse Distance: {self._calculate_path_distance(self.traverse_path):.1f}m")

def main():
    image_dir = Path("./dataset/train/images"); annotation_dir = Path("./dataset/train/labels")
    output_dir_base = Path("./output_with_sun_simulated"); output_dir_base.mkdir(exist_ok=True)
    image_files = sorted(list(image_dir.glob("*.jpg")))
    if not image_files: print(f"Error: No .jpg images found in '{image_dir}'."); return
    for image_path in image_files:
        annotation_path = annotation_dir / f"{image_path.stem}.txt"
        print(f"\n{'='*35} PROCESSING MAP: {image_path.name} {'='*35}")
        if not annotation_path.exists(): print(f"Warning: Annotation file not found. Skipping."); continue
        try:
            nav = LunarNavigation(output_dir=output_dir_base, map_name=image_path.stem)
            nav.execute_full_analysis(annotation_path)
        except Exception as e:
            print(f"An unexpected error occurred while processing {image_path.name}: {e}")

if __name__ == "__main__":
    main()
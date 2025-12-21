import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pandas as pd
import math
import csv
from math import radians, sin, cos, sqrt, atan2, degrees

class EcoDrivingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(EcoDrivingEnv, self).__init__()

        # --- CONFIGURATION ---
        self.target_time = 60.0 
        self.coords_file = 'eme_straight.csv'
        self.power_curve_file = '70_power_curve.csv'
        self.history = []
        
        # Load Data
        try:
            self.df = pd.read_csv(self.power_curve_file)
            self.df.columns = [c.strip() for c in self.df.columns]
        except:
            print("Using dummy data.")
            self.df = pd.DataFrame({'rpm': range(1000, 10000, 500), 'hp': range(10, 100, 5), 'torque': [50.0]*18})

        self.coords = self.read_coords(self.coords_file)
        if not self.coords: 
            self.coords = [(33.6, 73.0), (33.61, 73.0)] 

        # Physics Constants
        self.drivetrain_efficiency = .85
        self.tire_coeff = 0.01 
        self.g = 9.81
        self.prim_red = 1
        self.sec_red = 1
        self.gear_ratios = [37.06, 21.86, 15.30, 9.889]
        self.wheel_radius = 0.29
        self.frontal_area = 1.298
        self.drag_coeff = 0.327335
        self.mass = 220
        self.wheelbase = 1.6 
        self.pi = math.pi
        self.time_step = 0.1

        # Action: [Throttle, Gear]
        self.action_space = spaces.Box(low=0, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # STOPPED START
        self.init_speed_ms = 0.0 
        self.rpm = 1500 
        self.gear = 1
        
        self.distance_covered = 0.0
        self.current_time = 0.0
        self.steering_angle = 0.0
        self.current_coord_idx = 0
        self.segment_dist_covered = 0.0
        self.total_course_dist = self.calculate_total_course_distance()
        self.update_navigation_target()
        self.history = []
        return self._get_obs(), {}

    def _get_obs(self):
        dist_remaining = self.target_segment_dist - self.segment_dist_covered
        progress_pct = self.distance_covered / (self.total_course_dist + 1e-6)
        
        norm_speed = self.init_speed_ms / 40.0
        norm_rpm = self.rpm / 10000.0
        norm_dist = dist_remaining / 1000.0
        norm_steer = self.steering_angle / 45.0
        norm_gear = float(self.gear) / 4.0
        
        return np.array([norm_speed, norm_rpm, norm_dist, norm_steer, norm_gear, progress_pct], dtype=np.float32)

    def step(self, action):
        throttle_input = float(action[0]) 
        
        # Gear Logic
        g_val = float(action[1])
        if g_val < 0.25: self.gear = 1
        elif g_val < 0.50: self.gear = 2
        elif g_val < 0.75: self.gear = 3
        else: self.gear = 4

        # Force Gear 1 if stopped
        if self.init_speed_ms < 2.0:
            self.gear = 1

        # Steering
        bearing_diff = (self.next_bearing - self.current_bearing + 180) % 360 - 180
        self.steering_angle = bearing_diff 

        # --- PHYSICS ENGINE ---
        self.rpm_round()
        torque = self.get_torque()
        
        # Launch Logic: Connect Throttle to RPM if stopped
        if self.init_speed_ms < 1.0:
            if throttle_input > 0.05:
                # Fake RPM rise to generate torque lookup
                self.rpm = 1500 + (throttle_input * 2000) 
                torque = max(torque, 40.0) 
            else:
                self.rpm = 1500
        else:
            ratio = self.gear_ratios[self.gear - 1]
            self.rpm = (self.init_speed_ms * ratio * 60) / (2 * self.pi * self.wheel_radius)
            if self.rpm < 1500: self.rpm = 1500

        # Calculate Force
        ratio = self.gear_ratios[self.gear - 1]
        throttle_torque = torque * self.prim_red * self.sec_red * ratio * self.drivetrain_efficiency
        corrected_torque = throttle_torque * throttle_input
        
        drive_force = corrected_torque / self.wheel_radius
        drag_force = 0.5 * self.drag_coeff * self.frontal_area * (self.init_speed_ms ** 2) * 1.225
        rolling_resistance = self.mass * self.g * self.tire_coeff
        
        net_force = drive_force - drag_force - rolling_resistance
        accel = net_force / self.mass
        
        final_speed_ms = self.init_speed_ms + (accel * self.time_step)
        if final_speed_ms < 0: final_speed_ms = 0
        
        # --- END PHYSICS ---

        # Navigation
        dist_step = (self.init_speed_ms + final_speed_ms) / 2 * self.time_step
        self.distance_covered += dist_step
        self.segment_dist_covered += dist_step
        self.current_time += self.time_step

        reached_destination = False
        if self.segment_dist_covered >= self.target_segment_dist:
            residual = self.segment_dist_covered - self.target_segment_dist
            self.current_coord_idx += 1
            self.segment_dist_covered = residual
            if self.current_coord_idx >= len(self.coords) - 1:
                reached_destination = True
            else:
                self.update_navigation_target()

        # Log
        lat, lon = self.get_current_coords()
        self.history.append({
            'time': round(self.current_time, 2),
            'latitude': lat, 'longitude': lon,
            'throttle': throttle_input * 100, 
            'gear': self.gear,
            'speed_kmh': final_speed_ms * 3.6, 
            'rpm': self.rpm
        })

        # --- REWARDS ---
        terminated = False
        truncated = False
        reward = 0
        
        speed_kmh = final_speed_ms * 3.6
        
        # 1. THE LAVA FLOOR (LOWERED TO 10 KM/H)
        if speed_kmh < 10.0:
            reward -= 1.0 # High penalty for sticking around 0-9 km/h
        elif speed_kmh < 35:
            # Once you escape 10km/h, you get rewarded for speed
            reward += (speed_kmh * 0.05)
        else:
            reward -= 1.0
            
        # 2. Launch Cookie (Encourage pressing gas at start)
        if speed_kmh < 5.0 and throttle_input > 0.1:
            reward += 0.5

        # 3. Completion
        if reached_destination:
            terminated = True
            reward += 10
            
        if self.current_time > self.target_time * 2.0:
             truncated = True
             reward -= 10.0
        
        

        self.init_speed_ms = final_speed_ms

        return self._get_obs(), reward, terminated, truncated, {}

    # --- HELPERS ---
    def calculate_total_course_distance(self):
        d = 0
        if len(self.coords) < 2: return 0
        for i in range(len(self.coords)-1):
             d += self.haversine(self.coords[i][0], self.coords[i][1], self.coords[i+1][0], self.coords[i+1][1]) * 1000
        return d

    def update_navigation_target(self):
        if self.current_coord_idx < len(self.coords) - 1:
            p1 = self.coords[self.current_coord_idx]
            p2 = self.coords[self.current_coord_idx + 1]
            self.target_segment_dist = self.haversine(p1[0], p1[1], p2[0], p2[1]) * 1000 
            self.current_bearing = self.calculate_bearing(p1[0], p1[1], p2[0], p2[1])
            if self.current_coord_idx < len(self.coords) - 2:
                p3 = self.coords[self.current_coord_idx + 2]
                self.next_bearing = self.calculate_bearing(p2[0], p2[1], p3[0], p3[1])
            else: self.next_bearing = self.current_bearing
        else:
            self.target_segment_dist = 0; self.current_bearing = 0; self.next_bearing = 0

    def get_current_coords(self):
        if not self.coords: return 0.0, 0.0
        if self.current_coord_idx >= len(self.coords) - 1: return self.coords[-1]
        p1 = self.coords[self.current_coord_idx]
        p2 = self.coords[self.current_coord_idx + 1]
        ratio = self.segment_dist_covered / (self.target_segment_dist + 1e-6)
        ratio = max(0, min(1, ratio))
        lat = p1[0] + (p2[0] - p1[0]) * ratio
        lon = p1[1] + (p2[1] - p1[1]) * ratio
        return lat, lon

    def read_coords(self, filename):
        coords = []
        try:
            with open(filename, 'r', encoding='utf-8-sig') as csvfile:
                import os
                if os.stat(filename).st_size == 0: return []
                reader = csv.DictReader(csvfile)
                reader.fieldnames = [name.strip() for name in reader.fieldnames]
                for row in reader:
                    try:
                        coords.append((float(row.get('latitude', 0)), float(row.get('longitude', 0))))
                    except ValueError: continue
        except: return []
        return coords

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371.0
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1; dlon = lon2 - lon1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        return R * 2 * atan2(sqrt(a), sqrt(1 - a))

    def calculate_bearing(self, lat1, lon1, lat2, lon2):     
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        return (degrees(atan2(x, y)) + 360) % 360

    def rpm_round(self):
        idx = (self.df['rpm'] - self.rpm).abs().idxmin()
        self.rounded_rpm = self.df.loc[idx, 'rpm']

    def get_torque(self):
        val = self.df.loc[self.df['rpm'] == self.rounded_rpm, 'torque']
        return float(val.values[0]) if not val.empty else 0.0
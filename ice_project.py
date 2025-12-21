import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

class ICEngineAnalysis:
    """
    Complete thermodynamic and thermochemical analysis of an Internal Combustion Engine
    using the Air-Standard Otto Cycle model
    """
    
    def __init__(self, compression_ratio=9, bore=0.086, stroke=0.086, 
                 num_cylinders=4, intake_temp=300, intake_pressure=101325, 
                 fuel_energy=44e6, air_fuel_ratio=15, throttle=1.0):
        """
        Parameters:
        - compression_ratio: Volume ratio V1/V2
        - bore: Cylinder bore diameter in meters
        - stroke: Piston stroke length in meters
        - num_cylinders: Number of engine cylinders
        - intake_temp: Intake temperature in K
        - intake_pressure: Intake pressure in Pa
        - fuel_energy: Lower heating value of fuel in J/kg
        - air_fuel_ratio: Mass ratio of air to fuel
        - throttle: Throttle position (0.0 to 1.0, where 1.0 is wide open)
        """
        # Engine geometry
        self.bore = bore
        self.stroke = stroke
        self.num_cylinders = num_cylinders
        
        # Calculate displacement per cylinder
        self.V_d_cylinder = (np.pi / 4) * (self.bore ** 2) * self.stroke
        self.V_d_total = self.V_d_cylinder * self.num_cylinders
        
        # Engine parameters
        self.r = compression_ratio
        self.T1 = intake_temp
        self.P1 = intake_pressure * throttle  # Throttle reduces intake pressure
        self.Q_lhv = fuel_energy
        self.AFR = air_fuel_ratio
        self.throttle = throttle
        
        # Air properties (ideal gas)
        self.R = 287  # J/(kg·K) - specific gas constant for air
        self.gamma = 1.4  # specific heat ratio
        self.cv = self.R / (self.gamma - 1)  # J/(kg·K)
        self.cp = self.gamma * self.cv  # J/(kg·K)
        
        # Calculate clearance volume PER CYLINDER
        self.V_c = self.V_d_cylinder / (self.r - 1)
        self.V1 = self.V_d_cylinder + self.V_c  # Total volume per cylinder at BDC
        self.V2 = self.V_c  # Volume per cylinder at TDC
        
        # Mass of air in ONE cylinder
        self.m = (self.P1 * self.V1) / (self.R * self.T1)
        
        # Fuel mass per cylinder and heat addition per cylinder
        self.m_fuel = self.m / self.AFR
        self.Q_in = self.m_fuel * self.Q_lhv
        
    def analyze_cycle(self):
        """Perform complete Otto cycle analysis"""
        
        # State 1: Beginning of compression (BDC)
        self.P1 = self.P1
        self.V1 = self.V1
        self.T1 = self.T1
        
        # State 2: End of compression (TDC) - Isentropic compression
        self.V2 = self.V2
        self.T2 = self.T1 * (self.r ** (self.gamma - 1))
        self.P2 = self.P1 * (self.r ** self.gamma)
        
        # State 3: After combustion (TDC) - Constant volume heat addition
        self.V3 = self.V2
        self.T3 = self.T2 + self.Q_in / (self.m * self.cv)
        self.P3 = self.P2 * (self.T3 / self.T2)
        
        # State 4: End of expansion (BDC) - Isentropic expansion
        self.V4 = self.V1
        self.T4 = self.T3 / (self.r ** (self.gamma - 1))
        self.P4 = self.P3 / (self.r ** self.gamma)
        
        # Heat rejection (exhaust)
        self.Q_out = self.m * self.cv * (self.T4 - self.T1)
        
        # Net work
        self.W_net = self.Q_in - self.Q_out
        
        # Thermal efficiency
        self.eta_th = self.W_net / self.Q_in
        self.eta_otto = 1 - (1 / (self.r ** (self.gamma - 1)))
        
        # Mean effective pressure
        self.MEP = self.W_net / self.V_d_cylinder
        
        # Specific work
        self.w_net = self.W_net / self.m
        
        return self.compile_results()
    
    def compile_results(self):
        """Compile all thermodynamic results"""
        results = {
            'State Points': {
                'State 1 (Intake)': {'P': self.P1/1e5, 'V': self.V1*1e3, 'T': self.T1},
                'State 2 (Compression)': {'P': self.P2/1e5, 'V': self.V2*1e3, 'T': self.T2},
                'State 3 (Combustion)': {'P': self.P3/1e5, 'V': self.V3*1e3, 'T': self.T3},
                'State 4 (Expansion)': {'P': self.P4/1e5, 'V': self.V4*1e3, 'T': self.T4},
            },
            'Performance': {
                'Heat Input (kJ)': self.Q_in/1e3,
                'Heat Rejected (kJ)': self.Q_out/1e3,
                'Net Work (kJ)': self.W_net/1e3,
                'Thermal Efficiency (%)': self.eta_th*100,
                'Otto Efficiency (%)': self.eta_otto*100,
                'MEP (bar)': self.MEP/1e5,
                'Specific Work (kJ/kg)': self.w_net/1e3,
            },
            'Engine Parameters': {
                'Compression Ratio': self.r,
                'Bore (mm)': self.bore*1e3,
                'Stroke (mm)': self.stroke*1e3,
                'Bore/Stroke Ratio': self.bore/self.stroke,
                'Number of Cylinders': self.num_cylinders,
                'Throttle Position (%)': self.throttle*100,
                'Displacement per Cylinder (cc)': self.V_d_cylinder*1e6,
                'Total Displacement (L)': self.V_d_total*1e3,
                'Air Mass per Cylinder (g)': self.m*1e3,
                'Fuel Mass per Cylinder (g)': self.m_fuel*1e3,
                'Clearance Volume per Cylinder (cc)': self.V_c*1e6,
            }
        }
        return results
    
    def calculate_performance_at_rpm(self, rpm, volumetric_efficiency=0.85):
        """
        Calculate torque and power at a specific RPM
        
        Parameters:
        - rpm: Engine speed in revolutions per minute
        - volumetric_efficiency: Efficiency of cylinder filling (0-1)
        """
        # For 4-stroke engine, power stroke occurs every 2 revolutions
        cycles_per_second = rpm / 60 / 2
        
        # Work output per cylinder per cycle (accounting for VE)
        # Note: W_net is already per cylinder from the thermodynamic analysis
        work_per_cylinder = self.W_net * volumetric_efficiency
        
        # Total work per cycle (all cylinders firing)
        # In a 4-stroke, only half the cylinders fire per revolution
        total_work_per_cycle = work_per_cylinder * self.num_cylinders
        
        # Power = Work per cycle × cycles per second
        power_watts = total_work_per_cycle * cycles_per_second
        power_hp = power_watts / 745.7  # Convert to horsepower
        power_kw = power_watts / 1000
        
        # Torque = Power / angular velocity
        angular_velocity = (2 * np.pi * rpm) / 60  # rad/s
        torque = power_watts / angular_velocity
        
        return {
            'rpm': rpm,
            'torque_nm': torque,
            'power_kw': power_kw,
            'power_hp': power_hp
        }
    
    def generate_performance_curves(self, rpm_range=None, plot=True):
        """
        Generate torque and power curves across RPM range
        
        Parameters:
        - rpm_range: Array of RPM values, or None for default (1000-7000 RPM)
        - plot: Whether to plot the curves
        """
        if rpm_range is None:
            rpm_range = np.linspace(1000, 7000, 100)
        
        torque_curve = []
        power_kw_curve = []
        power_hp_curve = []
        
        # Ultra-smooth realistic volumetric efficiency model
        for rpm in rpm_range:
            # Continuous polynomial-based VE model for perfectly smooth curves
            # Based on real engine dyno data characteristics
            
            # Normalize RPM to 0-1 range for mathematical convenience
            rpm_normalized = (rpm - 1000) / 6000  # 0 at 1000 RPM, 1 at 7000 RPM
            
            # Use a smooth polynomial that captures real engine VE behavior:
            # - Low at very low RPM (poor intake velocity)
            # - Rises smoothly through mid-range
            # - Peaks around 0.5-0.6 normalized (4000-4500 RPM)
            # - Gradually declines at high RPM (flow restrictions)
            
            # 4th order polynomial for ultra-smooth curve
            # Coefficients tuned to match typical naturally aspirated engine
            a = -0.15  # Controls high RPM falloff
            b = 0.25   # Controls rate of rise
            c = 0.68   # Controls peak height
            d = 0.12   # Low RPM baseline
            
            # Polynomial: VE = a*x^4 + b*x^3 - c*x^2 + d*x + base
            ve_base = 0.70
            ve_variation = (a * rpm_normalized**4 + 
                           b * rpm_normalized**3 - 
                           0.30 * rpm_normalized**2 + 
                           0.35 * rpm_normalized)
            
            ve = ve_base + ve_variation * 0.25  # Scale variation to realistic range
            
            # Ensure realistic bounds
            ve = np.clip(ve, 0.68, 0.88)
            
            # Additional smooth real-world factors:
            
            # 1. Friction and pumping losses (very gradual increase with RPM)
            friction_factor = 1.0 - 0.000015 * rpm
            
            # 2. Intake air heating (gradual effect)
            heating_factor = 1.0 - 0.000008 * rpm
            
            # Combine all factors smoothly
            ve_effective = ve * friction_factor * heating_factor
            
            # Final realistic bounds
            ve_effective = np.clip(ve_effective, 0.62, 0.88)
            
            # Apply throttle effect to volumetric efficiency
            # Smooth throttle response curve
            throttle_factor = 0.20 + 0.80 * (self.throttle ** 0.9)  # Slight non-linearity
            ve_throttled = ve_effective * throttle_factor
            
            perf = self.calculate_performance_at_rpm(rpm, ve_throttled)
            torque_curve.append(perf['torque_nm'])
            power_kw_curve.append(perf['power_kw'])
            power_hp_curve.append(perf['power_hp'])
        
        results = {
            'rpm': rpm_range,
            'torque_nm': np.array(torque_curve),
            'power_kw': np.array(power_kw_curve),
            'power_hp': np.array(power_hp_curve),
            'peak_torque': np.max(torque_curve),
            'peak_torque_rpm': rpm_range[np.argmax(torque_curve)],
            'peak_power_kw': np.max(power_kw_curve),
            'peak_power_hp': np.max(power_hp_curve),
            'peak_power_rpm': rpm_range[np.argmax(power_kw_curve)]
        }
        
        if plot:
            self.plot_performance_curves(results)
        
        return results
    
    def plot_performance_curves(self, perf_data, ax=None):
        """Plot torque and power curves"""
        if ax is None:
            fig, ax1 = plt.subplots(figsize=(12, 7))
        else:
            ax1 = ax
        
        # Torque curve on primary y-axis
        color_torque = 'tab:blue'
        ax1.set_xlabel('Engine Speed (RPM)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Torque (N·m)', color=color_torque, fontsize=12, fontweight='bold')
        line1 = ax1.plot(perf_data['rpm'], perf_data['torque_nm'], 
                         color=color_torque, linewidth=2.5, label='Torque')
        ax1.tick_params(axis='y', labelcolor=color_torque)
        ax1.grid(True, alpha=0.3)
        
        # Power curve on secondary y-axis
        ax2 = ax1.twinx()
        color_power = 'tab:red'
        ax2.set_ylabel('Power (kW / HP)', color=color_power, fontsize=12, fontweight='bold')
        line2 = ax2.plot(perf_data['rpm'], perf_data['power_kw'], 
                        color=color_power, linewidth=2.5, label='Power (kW)')
        line3 = ax2.plot(perf_data['rpm'], perf_data['power_hp'], 
                        color='tab:orange', linewidth=2.5, linestyle='--', label='Power (HP)')
        ax2.tick_params(axis='y', labelcolor=color_power)
        
        # Mark peak values
        ax1.plot(perf_data['peak_torque_rpm'], perf_data['peak_torque'], 
                'o', color=color_torque, markersize=10)
        ax1.annotate(f"Peak: {perf_data['peak_torque']:.1f} N·m\n@ {perf_data['peak_torque_rpm']:.0f} RPM",
                    xy=(perf_data['peak_torque_rpm'], perf_data['peak_torque']),
                    xytext=(20, 20), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.7),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        
        ax2.plot(perf_data['peak_power_rpm'], perf_data['peak_power_kw'], 
                'o', color=color_power, markersize=10)
        ax2.annotate(f"Peak: {perf_data['peak_power_kw']:.1f} kW ({perf_data['peak_power_hp']:.1f} HP)\n@ {perf_data['peak_power_rpm']:.0f} RPM",
                    xy=(perf_data['peak_power_rpm'], perf_data['peak_power_kw']),
                    xytext=(-100, -40), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.7),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        
        # Combine legends
        lines = line1 + line2 + line3
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper left', fontsize=10)
        
        ax1.set_title(f'Engine Performance Curves (Throttle: {self.throttle*100:.0f}%)', 
                     fontsize=14, fontweight='bold')
        
        return ax1, ax2

    
    def calculate_entropy(self):
        """Calculate specific entropy at each state point"""
        # Reference entropy at state 1
        s1 = 0  # Reference point
        
        # State 2: Isentropic process, so s2 = s1
        s2 = s1
        
        # State 3: Constant volume heat addition
        s3 = s2 + self.cv * np.log(self.T3 / self.T2)
        
        # State 4: Isentropic expansion, so s4 = s3
        s4 = s3
        
        # For plotting, include the heat rejection line back to state 1
        # This is constant volume cooling
        
        return np.array([s1, s2, s3, s4]) / 1e3  # kJ/(kg·K)
    
    def plot_pv_diagram(self, ax=None):
        """Generate P-V diagram"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 7))
        
        # Create smooth curves for each process
        n_points = 100
        
        # Process 1-2: Isentropic compression
        V_12 = np.linspace(self.V1, self.V2, n_points)
        P_12 = self.P1 * (self.V1 / V_12) ** self.gamma
        
        # Process 2-3: Constant volume heat addition
        V_23 = np.array([self.V2, self.V3])
        P_23 = np.array([self.P2, self.P3])
        
        # Process 3-4: Isentropic expansion
        V_34 = np.linspace(self.V3, self.V4, n_points)
        P_34 = self.P3 * (self.V3 / V_34) ** self.gamma
        
        # Process 4-1: Constant volume heat rejection
        V_41 = np.array([self.V4, self.V1])
        P_41 = np.array([self.P4, self.P1])
        
        # Plot all processes
        ax.plot(V_12*1e3, P_12/1e5, 'b-', linewidth=2, label='1→2 Compression')
        ax.plot(V_23*1e3, P_23/1e5, 'r-', linewidth=2, label='2→3 Combustion')
        ax.plot(V_34*1e3, P_34/1e5, 'g-', linewidth=2, label='3→4 Expansion')
        ax.plot(V_41*1e3, P_41/1e5, 'm-', linewidth=2, label='4→1 Exhaust')
        
        # Mark state points
        ax.plot([self.V1*1e3], [self.P1/1e5], 'ko', markersize=8)
        ax.plot([self.V2*1e3], [self.P2/1e5], 'ko', markersize=8)
        ax.plot([self.V3*1e3], [self.P3/1e5], 'ko', markersize=8)
        ax.plot([self.V4*1e3], [self.P4/1e5], 'ko', markersize=8)
        
        # Annotations
        ax.annotate('1', (self.V1*1e3, self.P1/1e5), xytext=(10, -10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        ax.annotate('2', (self.V2*1e3, self.P2/1e5), xytext=(10, 10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        ax.annotate('3', (self.V3*1e3, self.P3/1e5), xytext=(10, -10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        ax.annotate('4', (self.V4*1e3, self.P4/1e5), xytext=(-20, -10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        
        ax.set_xlabel('Volume (L)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Pressure (bar)', fontsize=12, fontweight='bold')
        ax.set_title('P-V Diagram - Otto Cycle', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=10)
        
        return ax
    
    def plot_ts_diagram(self, ax=None):
        """Generate T-S diagram"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 7))
        
        # Get entropy values
        s = self.calculate_entropy()
        
        n_points = 100
        
        # Process 1-2: Isentropic (vertical line)
        s_12 = np.array([s[0], s[1]])
        T_12 = np.array([self.T1, self.T2])
        
        # Process 2-3: Constant volume heat addition
        s_23 = np.linspace(s[1], s[2], n_points)
        T_23 = self.T2 * np.exp((s_23 - s[1]) * 1e3 / self.cv)
        
        # Process 3-4: Isentropic (vertical line)
        s_34 = np.array([s[2], s[3]])
        T_34 = np.array([self.T3, self.T4])
        
        # Process 4-1: Constant volume heat rejection
        s_41 = np.linspace(s[3], s[0], n_points)
        T_41 = self.T4 * np.exp((s_41 - s[3]) * 1e3 / self.cv)
        
        # Plot all processes
        ax.plot(s_12, T_12, 'b-', linewidth=2, label='1→2 Compression')
        ax.plot(s_23, T_23, 'r-', linewidth=2, label='2→3 Combustion')
        ax.plot(s_34, T_34, 'g-', linewidth=2, label='3→4 Expansion')
        ax.plot(s_41, T_41, 'm-', linewidth=2, label='4→1 Exhaust')
        
        # Mark state points
        T_points = [self.T1, self.T2, self.T3, self.T4]
        ax.plot(s, T_points, 'ko', markersize=8)
        
        # Annotations
        ax.annotate('1', (s[0], self.T1), xytext=(10, -10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        ax.annotate('2', (s[1], self.T2), xytext=(10, 10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        ax.annotate('3', (s[2], self.T3), xytext=(10, -10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        ax.annotate('4', (s[3], self.T4), xytext=(-20, -10), 
                   textcoords='offset points', fontsize=12, fontweight='bold')
        
        ax.set_xlabel('Specific Entropy (kJ/kg·K)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Temperature (K)', fontsize=12, fontweight='bold')
        ax.set_title('T-S Diagram - Otto Cycle', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=10)
        
        return ax
    
    def print_results(self, results):
        """Print formatted results"""
        print("="*70)
        print("INTERNAL COMBUSTION ENGINE THERMODYNAMIC ANALYSIS")
        print("="*70)
        
        # Add debug info
        print(f"\nDEBUG INFO:")
        print(f"  Air mass per cylinder: {self.m*1e3:.4f} g")
        print(f"  Fuel mass per cylinder: {self.m_fuel*1e6:.4f} mg")
        print(f"  Heat input per cylinder: {self.Q_in:.2f} J")
        print(f"  Work output per cylinder: {self.W_net:.2f} J")
        print(f"  Thermal efficiency: {self.eta_th*100:.2f}%")
        
        for category, data in results.items():
            print(f"\n{category}:")
            print("-"*70)
            for key, value in data.items():
                if isinstance(value, dict):
                    print(f"  {key}:")
                    for k, v in value.items():
                        if k == 'P':
                            print(f"    Pressure: {v:.2f} bar")
                        elif k == 'V':
                            print(f"    Volume: {v:.4f} L")
                        elif k == 'T':
                            print(f"    Temperature: {v:.2f} K")
                else:
                    print(f"  {key}: {value:.4f}")
        print("="*70)


# Main execution
if __name__ == "__main__":
    # Create engine instance with your single cylinder specifications
    # 71cc single cylinder engine (47mm bore x 41mm stroke)
    engine = ICEngineAnalysis(
        compression_ratio=9.3,
        bore=0.047,  # 47 mm
        stroke=0.041,  # 41 mm
        num_cylinders=1,
        intake_temp=300,  # K
        intake_pressure=101325,  # Pa (1 atm)
        fuel_energy=44e6,  # J/kg (gasoline)
        air_fuel_ratio=15,
        throttle=1.0  # Wide open throttle (100%)
    )
    
    # Perform analysis
    results = engine.analyze_cycle()
    
    # Print results
    engine.print_results(results)
    
    # Generate performance curves
    print("\nGenerating performance curves...")
    perf_results = engine.generate_performance_curves(plot=False)
    
    print(f"\nPeak Torque: {perf_results['peak_torque']:.2f} N·m @ {perf_results['peak_torque_rpm']:.0f} RPM")
    print(f"Peak Power: {perf_results['peak_power_kw']:.2f} kW ({perf_results['peak_power_hp']:.2f} HP) @ {perf_results['peak_power_rpm']:.0f} RPM")
    
    # Create plots - all in one figure
    fig = plt.figure(figsize=(16, 12))
    
    # P-V diagram
    ax1 = plt.subplot(2, 2, 1)
    engine.plot_pv_diagram(ax1)
    
    # T-S diagram
    ax2 = plt.subplot(2, 2, 2)
    engine.plot_ts_diagram(ax2)
    
    # Performance curves - Torque
    ax3 = plt.subplot(2, 2, 3)
    color_torque = 'tab:blue'
    ax3.plot(perf_results['rpm'], perf_results['torque_nm'], 
             color=color_torque, linewidth=2.5, label='Torque')
    ax3.plot(perf_results['peak_torque_rpm'], perf_results['peak_torque'], 
            'o', color=color_torque, markersize=10)
    ax3.annotate(f"Peak: {perf_results['peak_torque']:.1f} N·m\n@ {perf_results['peak_torque_rpm']:.0f} RPM",
                xy=(perf_results['peak_torque_rpm'], perf_results['peak_torque']),
                xytext=(20, 20), textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.7),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    ax3.set_xlabel('Engine Speed (RPM)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Torque (N·m)', fontsize=12, fontweight='bold')
    ax3.set_title('Torque Curve', fontsize=13, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Performance curves - Power
    ax4 = plt.subplot(2, 2, 4)
    color_power_kw = 'tab:red'
    color_power_hp = 'tab:orange'
    ax4.plot(perf_results['rpm'], perf_results['power_kw'], 
            color=color_power_kw, linewidth=2.5, label='Power (kW)')
    ax4.plot(perf_results['rpm'], perf_results['power_hp'], 
            color=color_power_hp, linewidth=2.5, linestyle='--', label='Power (HP)')
    ax4.plot(perf_results['peak_power_rpm'], perf_results['peak_power_kw'], 
            'o', color=color_power_kw, markersize=10)
    ax4.annotate(f"Peak: {perf_results['peak_power_kw']:.1f} kW ({perf_results['peak_power_hp']:.1f} HP)\n@ {perf_results['peak_power_rpm']:.0f} RPM",
                xy=(perf_results['peak_power_rpm'], perf_results['peak_power_kw']),
                xytext=(-80, -40), textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.7),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    ax4.set_xlabel('Engine Speed (RPM)', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Power (kW / HP)', fontsize=12, fontweight='bold')
    ax4.set_title('Power Curve', fontsize=13, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    plt.suptitle(f'Complete Engine Analysis (Throttle: {engine.throttle*100:.0f}%)', 
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.show()
    
    # Optional: Compare different throttle positions
    print("\n" + "="*70)
    print("THROTTLE COMPARISON")
    print("="*70)
    
    throttle_positions = [0.25, 0.5, 0.75, 1.0]
    fig_throttle, axes = plt.subplots(1, 2, figsize=(16, 6))
    
    colors = ['red', 'orange', 'green', 'blue']
    
    # Use the same RPM range for all throttle positions
    rpm_range_comparison = np.linspace(1000, 7000, 100)
    
    for throttle_pos, color in zip(throttle_positions, colors):
        engine_throttle = ICEngineAnalysis(
            compression_ratio=engine.r,
            bore=engine.bore,
            stroke=engine.stroke,
            num_cylinders=engine.num_cylinders,
            intake_temp=engine.T1,
            intake_pressure=101325,  # Reset to atmospheric
            throttle=throttle_pos
        )
        engine_throttle.analyze_cycle()
        perf = engine_throttle.generate_performance_curves(rpm_range=rpm_range_comparison, plot=False)
        
        axes[0].plot(perf['rpm'], perf['torque_nm'], 
                    color=color, linewidth=2, label=f'{throttle_pos*100:.0f}% Throttle')
        axes[1].plot(perf['rpm'], perf['power_kw'], 
                    color=color, linewidth=2, label=f'{throttle_pos*100:.0f}% Throttle')
        
        print(f"\nThrottle {throttle_pos*100:.0f}%:")
        print(f"  Peak Torque: {perf['peak_torque']:.2f} N·m @ {perf['peak_torque_rpm']:.0f} RPM")
        print(f"  Peak Power: {perf['peak_power_kw']:.2f} kW ({perf['peak_power_hp']:.2f} HP)")
    
    axes[0].set_xlabel('Engine Speed (RPM)', fontsize=12, fontweight='bold')
    axes[0].set_ylabel('Torque (N·m)', fontsize=12, fontweight='bold')
    axes[0].set_title('Torque vs RPM at Different Throttle Positions', fontsize=13, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    
    axes[1].set_xlabel('Engine Speed (RPM)', fontsize=12, fontweight='bold')
    axes[1].set_ylabel('Power (kW)', fontsize=12, fontweight='bold')
    axes[1].set_title('Power vs RPM at Different Throttle Positions', fontsize=13, fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    
    plt.tight_layout()
    plt.show()
    
    print("\nAnalysis complete! Check all generated diagrams.")
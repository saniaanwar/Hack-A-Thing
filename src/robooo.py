import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
import serial
import threading
import time

class JoystickController:
    """Serial communication with nRF7002 joystick - EXACT COPY FROM DEBUG"""
    
    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.x_raw = 2048
        self.y_raw = 2048
        self.ir_state = 0
        self.connected = False
        self.center_x = 2048
        self.center_y = 2048
        
        # Track min/max for debugging
        self.x_min = 4095
        self.x_max = 0
        self.y_min = 4095
        self.y_max = 0
        
        if port:
            self.connect(port)
    
    @staticmethod
    def find_nrf_port():
        return "/dev/cu.usbmodem0010507895413"
    
    def connect(self, port):
        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=0.1)
            time.sleep(2)
            self.connected = True
            print(f"✓ Connected to {port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            self.connected = False
            return False
    
    def calibrate_center(self):
        print("🎯 Calibrating center... (leave joystick centered)")
        time.sleep(1)
        self.center_x = self.x_raw
        self.center_y = self.y_raw
        print(f"✓ Center: X={self.center_x}, Y={self.center_y}")
    
    def start_reading(self):
        self.running = True
        thread = threading.Thread(target=self._read_loop, daemon=True)
        thread.start()
    
    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('JOY:'):
                    data = line[4:].split(',')
                    if len(data) == 3:
                        self.x_raw = int(data[0])
                        self.y_raw = int(data[1])
                        self.ir_state = int(data[2])
                        
                        # Track ranges
                        self.x_min = min(self.x_min, self.x_raw)
                        self.x_max = max(self.x_max, self.x_raw)
                        self.y_min = min(self.y_min, self.y_raw)
                        self.y_max = max(self.y_max, self.y_raw)
            except:
                pass
    
    def get_normalized_xy(self):
        """Get normalized joystick position (-1 to 1) - EXACT FROM DEBUG"""
        # Calculate offset from center
        x_offset = self.x_raw - self.center_x
        y_offset = self.y_raw - self.center_y
        
        # Determine max range on each side
        x_left_range = self.center_x - self.x_min
        x_right_range = self.x_max - self.center_x
        y_down_range = self.center_y - self.y_min
        y_up_range = self.y_max - self.center_y
        
        # Normalize based on direction
        if x_offset < 0:  # Moving left
            x_norm = x_offset / max(x_left_range, 1) if x_left_range > 0 else 0
        else:  # Moving right
            x_norm = x_offset / max(x_right_range, 1) if x_right_range > 0 else 0
        
        if y_offset < 0:  # Moving down (raw values decrease)
            y_norm = -(y_offset / max(y_down_range, 1)) if y_down_range > 0 else 0
        else:  # Moving up (raw values increase)
            y_norm = -(y_offset / max(y_up_range, 1)) if y_up_range > 0 else 0
        
        # Clamp
        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))
        
        # Deadzone
        deadzone = 0.10
        if abs(x_norm) < deadzone:
            x_norm = 0
        if abs(y_norm) < deadzone:
            y_norm = 0
        
        return x_norm, y_norm
    
    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


class UR3Robot:
    """UR3 6-DOF Robot"""
    
    def __init__(self):
        self.DH = {
            'a': [0, -0.24365, -0.21325, 0, 0, 0],
            'd': [0.1519, 0, 0, 0.11235, 0.08535, 0.0819],
            'alpha': [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        }
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    def dh_matrix(self, a, d, alpha, theta):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, angles):
        T = np.eye(4)
        transforms = [T.copy()]
        for i in range(6):
            Ti = self.dh_matrix(
                self.DH['a'][i],
                self.DH['d'][i],
                self.DH['alpha'][i],
                angles[i]
            )
            T = T @ Ti
            transforms.append(T.copy())
        return transforms
    
    def get_joint_positions(self, angles):
        transforms = self.forward_kinematics(angles)
        return np.array([T[:3, 3] for T in transforms])


class JoystickRobotGUI:
    """GUI for Joystick-controlled Robot Arm"""
    
    def __init__(self):
        self.robot = UR3Robot()
        self.joystick = None
        self.running = False
        self.joint_speed = 0.03
        self.active_joint_1 = 0  # Base (LEFT/RIGHT)
        self.active_joint_2 = 1  # Shoulder (UP/DOWN)
        
        self.root = tk.Tk()
        self.root.title("🎮 Joystick Robot Control")
        self.root.configure(bg='#0f172a')
        self.root.geometry("1400x1000")
        
        self.create_widgets()
        self.setup_joystick()
        
    def setup_joystick(self):
        port = JoystickController.find_nrf_port()
        if port:
            self.joystick = JoystickController(port)
            if self.joystick.connected:
                self.joystick.start_reading()
                time.sleep(1.5)
                self.joystick.calibrate_center()
                
                # Reset ranges after calibration
                self.joystick.x_min = self.joystick.x_raw
                self.joystick.x_max = self.joystick.x_raw
                self.joystick.y_min = self.joystick.y_raw
                self.joystick.y_max = self.joystick.y_raw
                
                self.connection_label.config(text=f"✓ Connected: {port}", fg='#22c55e')
                self.running = True
                
                print("\n" + "="*80)
                print("🎮 JOYSTICK ROBOT CONTROL - ALL 4 DIRECTIONS WORKING!")
                print("="*80)
                print("\n⚠️  IMPORTANT: Move joystick in ALL 4 directions first!")
                print("   This calibrates the ranges for smooth control\n")
                print("Controls:")
                print("  ⬅️  LEFT   → Base rotates LEFT")
                print("  ➡️  RIGHT  → Base rotates RIGHT")
                print("  ⬆️  UP     → Shoulder lifts UP")
                print("  ⬇️  DOWN   → Shoulder goes DOWN")
                print("\n" + "="*80 + "\n")
                
                self.update_loop()
            else:
                self.connection_label.config(text="✗ Connection failed", fg='#dc2626')
        else:
            self.connection_label.config(text="✗ Port not found", fg='#dc2626')
    
    def create_widgets(self):
        # Title
        title_frame = tk.Frame(self.root, bg='#0f172a')
        title_frame.pack(pady=10, padx=20, fill=tk.X)
        
        tk.Label(title_frame, text="🎮 Joystick Robot Control",
                font=('Arial', 24, 'bold'), fg='white', bg='#0f172a').pack(anchor='w')
        
        tk.Label(title_frame, text="⬅️➡️ LEFT/RIGHT → Base (Top Joint) | ⬆️⬇️ UP/DOWN → Shoulder",
                font=('Arial', 12), fg='#22c55e', bg='#0f172a').pack(anchor='w')
        
        self.connection_label = tk.Label(title_frame, text="Connecting...",
                font=('Arial', 11), fg='#fbbf24', bg='#0f172a')
        self.connection_label.pack(anchor='w')
        
        # Canvas
        canvas_frame = tk.Frame(self.root, bg='#1e293b', highlightbackground='#334155', highlightthickness=2)
        canvas_frame.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)
        
        self.fig = plt.Figure(figsize=(11, 7), facecolor='#1e293b')
        self.ax = self.fig.add_subplot(111, projection='3d', facecolor='#1e293b')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Controls
        control_frame = tk.Frame(self.root, bg='#1e293b', highlightbackground='#22c55e', highlightthickness=2)
        control_frame.pack(pady=10, padx=20, fill=tk.X)
        
        tk.Label(control_frame, text="🎛️ Joint Pair:", font=('Arial', 12, 'bold'),
                fg='#cbd5e1', bg='#1e293b').pack(side=tk.LEFT, padx=10)
        
        pairs = [
            ("Base + Shoulder", 0, 1),
            ("Shoulder + Elbow", 1, 2),
            ("Elbow + Wrist1", 2, 3),
            ("Wrist1 + Wrist2", 3, 4),
            ("Wrist2 + Wrist3", 4, 5),
        ]
        
        for name, j1, j2 in pairs:
            tk.Button(control_frame, text=name, font=('Arial', 9, 'bold'),
                     bg='#3b82f6', fg='white', width=14, height=2,
                     command=lambda j1=j1, j2=j2: self.set_active_joints(j1, j2),
                     relief=tk.FLAT, cursor='hand2').pack(side=tk.LEFT, padx=2)
        
        tk.Label(control_frame, text="Speed:", font=('Arial', 10, 'bold'),
                fg='#cbd5e1', bg='#1e293b').pack(side=tk.LEFT, padx=8)
        
        self.speed_var = tk.DoubleVar(value=1.0)
        tk.Scale(control_frame, from_=0.2, to=3.0, resolution=0.1,
                variable=self.speed_var, orient=tk.HORIZONTAL,
                font=('Arial', 9), bg='#334155', fg='white',
                length=120).pack(side=tk.LEFT, padx=3)
        
        tk.Button(control_frame, text="🏠 Reset", font=('Arial', 11, 'bold'),
                 bg='#dc2626', fg='white', width=10, height=2,
                 command=self.reset, relief=tk.FLAT, cursor='hand2').pack(side=tk.LEFT, padx=5)
        
        # Info
        info_frame = tk.Frame(self.root, bg='#0f172a')
        info_frame.pack(pady=10, padx=20, fill=tk.X)
        
        joy_box = tk.Frame(info_frame, bg='#334155')
        joy_box.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        
        tk.Label(joy_box, text="🕹️ Joystick", font=('Arial', 11, 'bold'),
                fg='#cbd5e1', bg='#334155').pack(pady=5)
        
        self.joy_text = tk.Text(joy_box, height=6, font=('Courier', 9),
                               bg='#334155', fg='#94a3b8', state=tk.DISABLED)
        self.joy_text.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        
        joint_box = tk.Frame(info_frame, bg='#334155')
        joint_box.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        
        tk.Label(joint_box, text="🤖 Joints", font=('Arial', 11, 'bold'),
                fg='#cbd5e1', bg='#334155').pack(pady=5)
        
        self.joint_text = tk.Text(joint_box, height=6, font=('Courier', 9),
                                 bg='#334155', fg='#94a3b8', state=tk.DISABLED)
        self.joint_text.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        
        self.update_plot()
    
    def set_active_joints(self, j1, j2):
        self.active_joint_1 = j1
        self.active_joint_2 = j2
        names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
        print(f"\n✓ Now controlling: {names[j1]} (L/R) & {names[j2]} (U/D)\n")
        self.update_plot()
    
    def update_loop(self):
        if not self.running or not self.joystick:
            return
        
        x_joy, y_joy = self.joystick.get_normalized_xy()
        
        if abs(x_joy) > 0 or abs(y_joy) > 0:
            speed = self.joint_speed * self.speed_var.get()
            self.robot.joint_angles[self.active_joint_1] += x_joy * speed
            self.robot.joint_angles[self.active_joint_2] += y_joy * speed
            self.robot.joint_angles = np.clip(self.robot.joint_angles, -2*np.pi, 2*np.pi)
            self.update_plot()
        
        self.update_info(x_joy, y_joy)
        self.root.after(20, self.update_loop)
    
    def update_plot(self):
        self.ax.clear()
        
        positions = self.robot.get_joint_positions(self.robot.joint_angles)
        colors = ['#ef4444', '#f59e0b', '#10b981', '#3b82f6', '#8b5cf6', '#ec4899']
        
        for i in range(len(positions) - 1):
            lw = 8 if i in [self.active_joint_1, self.active_joint_2] else 4
            alpha = 1.0 if i in [self.active_joint_1, self.active_joint_2] else 0.6
            
            self.ax.plot([positions[i][0], positions[i+1][0]],
                        [positions[i][1], positions[i+1][1]],
                        [positions[i][2], positions[i+1][2]],
                        'o-', linewidth=lw, markersize=12, 
                        color=colors[i], alpha=alpha)
        
        # Highlight active joints
        for joint_idx in [self.active_joint_1, self.active_joint_2]:
            if joint_idx + 1 < len(positions):
                pos = positions[joint_idx + 1]
                self.ax.scatter([pos[0]], [pos[1]], [pos[2]],
                              color='#fbbf24', s=400, marker='o', 
                              edgecolors='white', linewidths=3, zorder=10)
        
        # End-effector
        ee = positions[-1]
        self.ax.scatter([ee[0]], [ee[1]], [ee[2]],
                       color='#22c55e', s=250, marker='*', zorder=11)
        
        # Ground
        xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 2), np.linspace(-0.5, 0.5, 2))
        zz = np.zeros_like(xx)
        self.ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')
        
        self.ax.set_xlabel('X (m)', color='white')
        self.ax.set_ylabel('Y (m)', color='white')
        self.ax.set_zlabel('Z (m)', color='white')
        self.ax.tick_params(colors='white', labelsize=8)
        self.ax.set_xlim([-0.6, 0.6])
        self.ax.set_ylim([-0.6, 0.6])
        self.ax.set_zlim([0, 0.8])
        self.ax.view_init(elev=20, azim=-70)
        self.ax.grid(True, alpha=0.2)
        
        self.canvas.draw()
    
    def update_info(self, x_joy, y_joy):
        self.joy_text.config(state=tk.NORMAL)
        self.joy_text.delete(1.0, tk.END)
        
        x_bar = '█' * max(0, int(abs(x_joy) * 15))
        y_bar = '█' * max(0, int(abs(y_joy) * 15))
        
        self.joy_text.insert(tk.END, f"X (L/R): {x_joy:+.2f} {x_bar}\n")
        self.joy_text.insert(tk.END, f"Y (U/D): {y_joy:+.2f} {y_bar}\n")
        if self.joystick:
            self.joy_text.insert(tk.END, f"\nRaw: {self.joystick.x_raw:4d}, {self.joystick.y_raw:4d}\n")
            self.joy_text.insert(tk.END, f"Range X: {self.joystick.x_min}-{self.joystick.x_max}\n")
            self.joy_text.insert(tk.END, f"Range Y: {self.joystick.y_min}-{self.joystick.y_max}\n")
        self.joy_text.config(state=tk.DISABLED)
        
        names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
        self.joint_text.config(state=tk.NORMAL)
        self.joint_text.delete(1.0, tk.END)
        for i, angle in enumerate(self.robot.joint_angles):
            marker = "🟡" if i in [self.active_joint_1, self.active_joint_2] else "⚪"
            self.joint_text.insert(tk.END, f"{marker} {names[i]:8s}: {np.degrees(angle):6.1f}°\n")
        self.joint_text.config(state=tk.DISABLED)
    
    def reset(self):
        self.robot.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.active_joint_1 = 0
        self.active_joint_2 = 1
        self.update_plot()
        print("🏠 Reset to home")
    
    def run(self):
        self.root.mainloop()
        if self.joystick:
            self.joystick.stop()


def main():
    print("\n" + "=" * 80)
    print("🎮 JOYSTICK ROBOT CONTROL")
    print("=" * 80)
    print("\n✅ This is the EXACT code from the working debug version!")
    print("\n⚠️  After starting, move joystick in ALL 4 directions once")
    print("   to calibrate the ranges, then control will be smooth!\n")
    print("=" * 80 + "\n")
    
    gui = JoystickRobotGUI()
    gui.run()


if __name__ == "__main__":
    main()

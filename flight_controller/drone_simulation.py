"""
Drone virtuale pilotato dai comandi motore ricevuti via seriale (TCP/QEMU).

Architettura
------------
1. Un thread in background apre un server TCP (come nello script originale),
   accetta la connessione di QEMU e legge righe del tipo "m1 m2 m3 m4".
   I 4 valori vengono salvati in uno stato condiviso (thread-safe).
2. Il thread principale esegue:
   - un modello fisico accurato di un quadricottero (dinamica a 6 DOF);
   - un'animazione 3D in tempo reale con matplotlib.

Il modello integra le equazioni di Newton-Euler con un integratore RK4 e
sotto-passi a passo fisso, quindi reagisce in modo fisicamente coerente a
QUALSIASI combinazione di comandi motore (non solo a quella del demo in main.c).

Requisiti: numpy, matplotlib  ->  pip install numpy matplotlib
"""

import socket
import select
import time
import threading
from collections import deque

import numpy as np

import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registra la proiezione 3D)

# ============================================================================
#  CONFIGURAZIONE RETE (come nello script originale)
# ============================================================================
HOST = '127.0.0.1'
PORT = 1234

# ============================================================================
#  PARAMETRI FISICI DEL QUADRICOTTERO
# ============================================================================
MASS = 0.8
GRAVITY = 9.81
ARM_LENGTH = 0.25

INERTIA = np.array([0.008, 0.008, 0.015])

DRAG_LINEAR = 0.25
DRAG_ANGULAR = 0.02

YAW_TORQUE_COEFF = 0.02

HOVER_CMD = 2.1
K_THRUST = MASS * GRAVITY / (4.0 * HOVER_CMD ** 2)

GROUND_FRICTION = 0.95
GROUND_ANG_DAMP = 0.90

PHYS_SUBSTEP = 0.002
MAX_FRAME_DT = 0.05

# ============================================================================
#  ANIMAZIONE
# ============================================================================
FRAME_MS = 25
VIEW_HALF = 1.5
TRAIL_LEN = 200

# ============================================================================
#  MATEMATICA DEI QUATERNIONI
# ============================================================================
def quat_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_rotmat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

# ============================================================================
#  MODELLO FISICO
# ============================================================================
class Quadcopter:
    def __init__(self):
        d = ARM_LENGTH / np.sqrt(2.0)
        self.motor_pos = np.array([
            [ d, -d, 0.0],
            [ d,  d, 0.0],
            [-d,  d, 0.0],
            [-d, -d, 0.0],
        ])
        self.arm = d
        self.reset()

    def reset(self):
        self.state = np.zeros(13)
        self.state[6] = 1.0

    @property
    def position(self):
        return self.state[0:3]

    @property
    def velocity(self):
        return self.state[3:6]

    @property
    def rotmat(self):
        return quat_to_rotmat(self.state[6:10])

    def euler_deg(self):
        r = self.rotmat
        roll = np.arctan2(r[2,1], r[2,2])
        pitch = np.arctan2(-r[2,0], np.hypot(r[2,1], r[2,2]))
        yaw = np.arctan2(r[1,0], r[0,0])
        return np.degrees([roll, pitch, yaw])

    def _derivatives(self, s, thrusts):
        v = s[3:6]
        q = s[6:10]
        w = s[10:13]

        rot = quat_to_rotmat(q)

        t1, t2, t3, t4 = thrusts
        thrust_total = t1 + t2 + t3 + t4

        tau_roll  = self.arm * (-t1 + t2 + t3 - t4)
        tau_pitch = self.arm * (-t1 - t2 + t3 + t4)
        tau_yaw   = YAW_TORQUE_COEFF * (-t1 + t2 - t3 + t4)
        tau = np.array([tau_roll, tau_pitch, tau_yaw]) - DRAG_ANGULAR * w

        force_world = (rot @ np.array([0,0,thrust_total])
                       + np.array([0,0,-MASS*GRAVITY])
                       - DRAG_LINEAR * v)
        accel = force_world / MASS

        q_dot = 0.5 * quat_mult(q, np.array([0, w[0], w[1], w[2]]))

        iw = INERTIA * w
        w_dot = (tau - np.cross(w, iw)) / INERTIA

        ds = np.empty(13)
        ds[0:3] = v
        ds[3:6] = accel
        ds[6:10] = q_dot
        ds[10:13] = w_dot
        return ds

    def step(self, motor_cmds, dt):
        cmds = np.clip(np.asarray(motor_cmds), 0.0, None)
        thrusts = K_THRUST * cmds**2

        n = max(1, int(np.ceil(dt / PHYS_SUBSTEP)))
        h = dt / n
        s = self.state

        for _ in range(n):
            k1 = self._derivatives(s, thrusts)
            k2 = self._derivatives(s + 0.5*h*k1, thrusts)
            k3 = self._derivatives(s + 0.5*h*k2, thrusts)
            k4 = self._derivatives(s + h*k3, thrusts)
            s = s + (h/6)*(k1 + 2*k2 + 2*k3 + k4)

            qn = np.linalg.norm(s[6:10])
            if qn > 0:
                s[6:10] /= qn

            if s[2] < 0:
                s[2] = 0
                if s[5] < 0:
                    s[5] = 0
                s[3:5] *= GROUND_FRICTION
                s[10:13] *= GROUND_ANG_DAMP

        self.state = s

# ============================================================================
#  STATO CONDIVISO
# ============================================================================
class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._cmds = [0,0,0,0]
        self.connected = False

    def set_cmds(self, cmds):
        with self._lock:
            self._cmds = list(cmds)

    def get_cmds(self):
        with self._lock:
            return list(self._cmds)

def parse_motor_line(text):
    parts = text.split()
    if len(parts) < 4:
        return None
    try:
        return [float(parts[i]) for i in range(4)]
    except ValueError:
        return None

# ============================================================================
#  THREAD TCP — MODIFICATO PER INVIARE L’ALTEZZA
# ============================================================================
def serial_reader(shared, stop_event, model):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        server_socket.settimeout(1.0)

        print(f"[*] Server in ascolto su {HOST}:{PORT}...")

        while not stop_event.is_set():
            try:
                client_socket, addr = server_socket.accept()
            except socket.timeout:
                continue

            print(f"[+] QEMU connesso da {addr}")
            shared.connected = True
            client_socket.setblocking(False)

            buf = b""
            while not stop_event.is_set():
                try:
                    ready, _, _ = select.select([client_socket], [], [], 0.5)
                except OSError:
                    break

                if not ready:
                    continue

                try:
                    chunk = client_socket.recv(4096)
                except BlockingIOError:
                    continue

                if not chunk:
                    break

                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode().strip()
                    if not line:
                        continue

                    cmds = parse_motor_line(line)
                    if cmds is not None:
                        shared.set_cmds(cmds)

                        # --- INVIO ALTEZZA A QEMU (solo numero + newline) ---
                        try:
                            z = model.position[2]
                            client_socket.send(f"{z:.3f}\n".encode())
                        except Exception:
                            pass

                    else:
                        print("python reply:", line)

            client_socket.close()
            shared.connected = False
            shared.set_cmds([0,0,0,0])

    finally:
        server_socket.close()
        print("[*] Server spento.")

# ============================================================================
#  VISUALIZZATORE 3D — CON LINEA QUOTA 100m
# ============================================================================
class DroneVisualizer:
    def __init__(self, model, shared, stop_event):
        self.model = model
        self.shared = shared
        self.stop_event = stop_event
        self.last_time = None
        self.trail = deque(maxlen=TRAIL_LEN)

        self.center = np.zeros(3)
        self.motor_pos = model.motor_pos
        self.heading_pt = np.array([1.6*model.arm, 0, 0])

        ang = np.linspace(0, 2*np.pi, 18)
        r = 0.45 * model.arm
        unit_circle = np.column_stack([r*np.cos(ang), r*np.sin(ang), np.zeros_like(ang)])
        self.rotor_circles = [m + unit_circle for m in self.motor_pos]

        self.fig = plt.figure(figsize=(10,8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_box_aspect((1,1,1))

        self._build_ground()

        # --- LINEA QUOTA 100m ---
        z_mark = 100.0
        self.alt_line, = self.ax.plot(
            [-VIEW_HALF, VIEW_HALF],
            [0, 0],
            [z_mark, z_mark],
            color='magenta',
            lw=2,
            linestyle='--'
        )

        arm_colors = ['#d62728','#d62728','#1f77b4','#1f77b4']
        self.arm_lines = [self.ax.plot([],[],[],color=c,lw=3)[0] for c in arm_colors]
        self.rotor_lines = [self.ax.plot([],[],[],color=c,lw=2)[0] for c in arm_colors]
        self.heading_line, = self.ax.plot([],[],[],color='#2ca02c',lw=2)
        self.center_pt, = self.ax.plot([],[],[],'o',color='black',ms=5)
        self.trail_line, = self.ax.plot([],[],[],color='#ff7f0e',lw=1,alpha=0.7)

        self.hud = self.ax.text2D(0.02,0.95,"",transform=self.ax.transAxes,
                                  family='monospace',fontsize=10,
                                  bbox=dict(boxstyle='round',fc='white',alpha=0.7))

        self.fig.canvas.mpl_connect('close_event', self._on_close)

    def _build_ground(self):
        g, step = 25, 1
        xs, ys, zs = [], [], []
        for x in range(-g,g+1,step):
            xs += [x,x,np.nan]
            ys += [-g,g,np.nan]
            zs += [0,0,np.nan]
        for y in range(-g,g+1,step):
            xs += [-g,g,np.nan]
            ys += [y,y,np.nan]
            zs += [0,0,np.nan]
        self.ax.plot(xs,ys,zs,color='0.85',lw=0.5)

    def _on_close(self,_):
        self.stop_event.set()

    @staticmethod
    def _set_line(line, pts):
        line.set_data(pts[:,0], pts[:,1])
        line.set_3d_properties(pts[:,2])

    def update(self,_):
        now = time.perf_counter()
        if self.last_time is None:
            self.last_time = now
        dt = min(now - self.last_time, MAX_FRAME_DT)
        self.last_time = now

        cmds = self.shared.get_cmds()
        if dt > 0:
            self.model.step(cmds, dt)

        p = self.model.position
        rot = self.model.rotmat

        def to_world(pts):
            return p + (rot @ np.atleast_2d(pts).T).T

        thrusts = K_THRUST * np.clip(np.asarray(cmds),0,None)**2
        tmax = max(thrusts.max(),1e-6)

        for i in range(4):
            seg = to_world(np.vstack([self.center, self.motor_pos[i]]))
            self._set_line(self.arm_lines[i], seg)

            circ = to_world(self.rotor_circles[i])
            self._set_line(self.rotor_lines[i], circ)
            self.rotor_lines[i].set_alpha(0.25 + 0.75 * thrusts[i]/tmax)

        heading = to_world(np.vstack([self.center, self.heading_pt]))
        self._set_line(self.heading_line, heading)

        self.center_pt.set_data([p[0]],[p[1]])
        self.center_pt.set_3d_properties([p[2]])

        self.trail.append(p.copy())
        trail = np.array(self.trail)
        self._set_line(self.trail_line, trail)

        cx,cy,cz = p
        zc = max(VIEW_HALF, cz)
        self.ax.set_xlim(cx-VIEW_HALF, cx+VIEW_HALF)
        self.ax.set_ylim(cy-VIEW_HALF, cy+VIEW_HALF)
        self.ax.set_zlim(zc-VIEW_HALF, zc+VIEW_HALF)

        # --- MANTIENI LA LINEA DI QUOTA CENTRATA ---
        self.alt_line.set_data([p[0]-VIEW_HALF, p[0]+VIEW_HALF],
                               [p[1], p[1]])
        self.alt_line.set_3d_properties([100.0, 100.0])

        roll,pitch,yaw = self.model.euler_deg()
        speed = np.linalg.norm(self.model.velocity)
        conn = "CONNESSO" if self.shared.connected else "in attesa di QEMU"

        self.hud.set_text(
            f"Stato : {conn}\n"
            f"Quota : {p[2]:6.2f} m\n"
            f"Vel.  : {speed:6.2f} m/s\n"
            f"Roll  : {roll:+6.1f} deg\n"
            f"Pitch : {pitch:+6.1f} deg\n"
            f"Yaw   : {yaw:+6.1f} deg\n"
            f"Motori: {cmds[0]:.1f} {cmds[1]:.1f} {cmds[2]:.1f} {cmds[3]:.1f}"
        )

        return (self.arm_lines + self.rotor_lines +
                [self.heading_line, self.center_pt, self.trail_line,
                 self.hud, self.alt_line])

    def run(self):
        self.anim = matplotlib.animation.FuncAnimation(
            self.fig, self.update, interval=FRAME_MS,
            blit=False, cache_frame_data=False)
        plt.show()

# ============================================================================
#  MAIN
# ============================================================================
def main():
    print("=== Drone virtuale pilotato via seriale ===")

    shared = SharedState()
    stop_event = threading.Event()
    model = Quadcopter()

    reader = threading.Thread(
        target=serial_reader,
        args=(shared, stop_event, model),
        daemon=True
    )
    reader.start()

    viz = DroneVisualizer(model, shared, stop_event)
    try:
        viz.run()
    finally:
        stop_event.set()
        reader.join(timeout=2.0)

if __name__ == "__main__":
    main()

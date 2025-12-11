#!/usr/bin/env python3
"""
scenario1_pd_paper_exact.py

PD controller baseline for Scenario-1 (paper).
Includes:
 - True initial conditions from paper
 - Ground-starting tilted ellipse reference
 - Start/End markers & labels
 - Paper-style 3D view
"""

import os
import math
import pickle
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------
# OUTPUT DIRECTORY
# ------------------------------------------------------------
OUTDIR = "AFONFTSMC_Scenario3_Results"
os.makedirs(OUTDIR, exist_ok=True)
print("[INFO] Outputs ->", OUTDIR)

# ------------------------------------------------------------
# PHYSICAL PARAMETERS
# ------------------------------------------------------------
m = 0.74
I11 = 0.4e-2; I22 = 0.4e-2; I33 = 0.84e-2
g = 9.81

fs = 200
dt = 1.0 / fs

T = 20.0                 # one loop (paper)
t = np.arange(0, T + dt, dt)
N = len(t)

# ------------------------------------------------------------
# PAPER-MATCHING REFERENCE TRAJECTORY
# (tilted ellipse, starts on ground)
# ------------------------------------------------------------
omega = 0.35
phase_y = math.radians(25)
phase_z = math.radians(55)

def xr_ref(tt):
    return 0.5 * math.cos(omega * tt)

def yr_ref(tt):
    return 0.45 * math.sin(omega * tt + phase_y)

def zr_ref(tt):
    # offset/amplitude tuned for zr(0)=0 and match Fig.7(a)
    return 2.0 - 2.0 * math.cos(omega * tt + phase_z)

def psir_ref(tt):
    return 0.5

# Disturbances (simple)
def dpx(tt):
    if 10 <= tt <= 30:
        return -(0.08 * math.sin(0.11 * tt) + 0.04 * math.sin(0.5 * tt))
    return 0.0
def dpy(tt):
    if 10 <= tt <= 50:
        return 0.05 * math.sin(0.4 * tt) + 0.05 * math.cos(0.7 * tt)
    return 0.0
def dpz(tt):
    return 0.05 * math.cos(0.7 * tt) + 0.07 * math.sin(0.3 * tt)

def dphi(tt): return 0.5 * math.cos(0.4 * tt) + 1
def dtheta(tt): return 0.5 * math.sin(0.5 * tt) + 1
def dpsi(tt): return 0.05 * math.sin(0.7 * tt) + 0.1

# ------------------------------------------------------------
# STATE ARRAYS
# ------------------------------------------------------------
x = np.zeros(N);  y = np.zeros(N);  z = np.zeros(N)
vx = np.zeros(N); vy = np.zeros(N); vz = np.zeros(N)
phi = np.zeros(N); theta = np.zeros(N); psi = np.zeros(N)
p_om = np.zeros(N); q_om = np.zeros(N); r_om = np.zeros(N)

# ------------------------------------------------------------
# TRUE PAPER INITIAL CONDITIONS (page 7)
# ------------------------------------------------------------
x[0] = 0.05
y[0] = 0.01
z[0] = 0.00

vx[0] = 0.01
vy[0] = 0.01
vz[0] = 0.10

phi[0]   = 0.01
theta[0] = 0.01
psi[0]   = 0.10

p_om[0] = 0.0
q_om[0] = 0.0
r_om[0] = 0.0

# ------------------------------------------------------------
# PD GAINS
# ------------------------------------------------------------
Kp_pos = np.array([30.0, 25.0, 60.0])
Kd_pos = np.array([12.0, 10.0, 22.0])

Kp_att = np.array([60.0, 60.0, 30.0])
Kd_att = np.array([6.0, 6.0, 2.0])

# Logging
uF = np.zeros(N)
tau_x = np.zeros(N); tau_y = np.zeros(N); tau_z = np.zeros(N)
ux = np.zeros(N); uy = np.zeros(N); uz = np.zeros(N)

# ------------------------------------------------------------
# RK4 INTEGRATOR
# ------------------------------------------------------------
def rk4_step(state, f, dt):
    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# ------------------------------------------------------------
# SIMPLIFIED QUAD DYNAMICS
# ------------------------------------------------------------
def plant_dyn(state, ux_cmd, uy_cmd, uz_cmd, tx_cmd, ty_cmd, tz_cmd, tt):
    sx, sy, sz, svx, svy, svz, sphi, stheta, spsi, sp, sq, sr = state

    # small damping
    Kpx = Kpy = Kpz = 0.01

    ddx = -Kpx * svx + dpx(tt) + ux_cmd
    ddy = -Kpy * svy + dpy(tt) + uy_cmd
    ddz = -Kpz * svz + dpz(tt) + uz_cmd

    dphi_dt = sp
    dtheta_dt = sq
    dpsi_dt = sr

    dp_dt = (tx_cmd - 0.02 * sp) / I11 + dphi(tt)/I11
    dq_dt = (ty_cmd - 0.02 * sq) / I22 + dtheta(tt)/I22
    dr_dt = (tz_cmd - 0.02 * sr) / I33 + dpsi(tt)/I33

    return np.array([svx, svy, svz, ddx, ddy, ddz,
                     dphi_dt, dtheta_dt, dpsi_dt,
                     dp_dt, dq_dt, dr_dt])

# ------------------------------------------------------------
# SIMULATION LOOP
# ------------------------------------------------------------
for k in range(1, N):
    tt = t[k]

    # Reference & derivatives
    xr = xr_ref(tt); yr = yr_ref(tt); zr = zr_ref(tt)

    xr_d = (xr_ref(tt) - xr_ref(tt-dt))/dt if k>1 else 0
    yr_d = (yr_ref(tt) - yr_ref(tt-dt))/dt if k>1 else 0
    zr_d = (zr_ref(tt) - zr_ref(tt-dt))/dt if k>1 else 0

    xr_dd = (xr_ref(tt) - 2*xr_ref(tt-dt) + xr_ref(tt-2*dt))/(dt*dt) if k>2 else 0
    yr_dd = (yr_ref(tt) - 2*yr_ref(tt-dt) + yr_ref(tt-2*dt))/(dt*dt) if k>2 else 0
    zr_dd = (zr_ref(tt) - 2*zr_ref(tt-dt) + zr_ref(tt-2*dt))/(dt*dt) if k>2 else 0

    # PD outer loop
    ex = x[k-1]-xr; ey = y[k-1]-yr; ez = z[k-1]-zr
    evx = vx[k-1]-xr_d; evy = vy[k-1]-yr_d; evz = vz[k-1]-zr_d

    ux_cmd = xr_dd - Kp_pos[0]*ex - Kd_pos[0]*evx
    uy_cmd = yr_dd - Kp_pos[1]*ey - Kd_pos[1]*evy
    uz_cmd = zr_dd - Kp_pos[2]*ez - Kd_pos[2]*evz

    ux_cmd = float(np.clip(ux_cmd, -10, 10))
    uy_cmd = float(np.clip(uy_cmd, -10, 10))
    uz_cmd = float(np.clip(uz_cmd, -10, 10))

    ux[k] = ux_cmd; uy[k] = uy_cmd; uz[k] = uz_cmd

    # Desired angles from mapping
    psir = psir_ref(tt)
    denom = uz_cmd + g
    if abs(denom) < 1e-6: denom = 1e-6

    phi_r = math.atan2((ux_cmd*math.sin(psir) - uy_cmd*math.cos(psir)), denom)
    theta_r = math.atan2((ux_cmd*math.cos(psir) + uy_cmd*math.sin(psir)), denom)

    phi_r = float(np.clip(phi_r, -0.4, 0.4))
    theta_r = float(np.clip(theta_r, -0.4, 0.4))

    uF[k] = m * math.sqrt(ux_cmd**2 + uy_cmd**2 + (uz_cmd + g)**2)
    uF[k] = float(np.clip(uF[k], 0, 30))

    # Attitude PD
    e_phi = phi[k-1]-phi_r; e_theta = theta[k-1]-theta_r; e_psi = psi[k-1]-psir
    tx = -Kp_att[0]*e_phi   - Kd_att[0]*p_om[k-1]
    ty = -Kp_att[1]*e_theta - Kd_att[1]*q_om[k-1]
    tz = -Kp_att[2]*e_psi   - Kd_att[2]*r_om[k-1]

    tau_x[k] = float(np.clip(tx, -5,  5))
    tau_y[k] = float(np.clip(ty, -5,  5))
    tau_z[k] = float(np.clip(tz, -2,  2))

    # RK4 integrate
    state_prev = np.array([x[k-1], y[k-1], z[k-1],
                           vx[k-1], vy[k-1], vz[k-1],
                           phi[k-1], theta[k-1], psi[k-1],
                           p_om[k-1], q_om[k-1], r_om[k-1]])

    def f(s):
        return plant_dyn(s, ux_cmd, uy_cmd, uz_cmd,
                         tau_x[k], tau_y[k], tau_z[k], tt)

    new = rk4_step(state_prev, f, dt)

    x[k], y[k], z[k] = new[0], new[1], new[2]
    vx[k], vy[k], vz[k] = new[3], new[4], new[5]
    phi[k], theta[k], psi[k] = new[6], new[7], new[8]
    p_om[k], q_om[k], r_om[k] = new[9], new[10], new[11]

# ------------------------------------------------------------
# PLOTS
# ------------------------------------------------------------
def savefig(name):
    plt.savefig(os.path.join(OUTDIR, name), dpi=300, bbox_inches="tight")
    plt.close()

# ------------- 3D TRAJECTORY (Paper-style) -----------------
fig = plt.figure(figsize=(8,7))
ax = fig.add_subplot(111, projection="3d")

# Proposed
ax.plot(y, x, z, color="red", lw=2, label="Proposed method")

# Reference
ax.plot([yr_ref(tt) for tt in t],
        [xr_ref(tt) for tt in t],
        [zr_ref(tt) for tt in t],
        "k--", lw=1.5, label="Reference")

# Start & end markers
ax.scatter([y[0]], [x[0]], [z[0]], s=60, c="black")
ax.text(y[0], x[0], z[0], "Start Point", color="black")

ax.scatter([y[-1]], [x[-1]], [z[-1]], s=60, c="black")
ax.text(y[-1], x[-1], z[-1], "End Point", color="black")

ax.set_xlabel("y (m)")
ax.set_ylabel("x (m)")
ax.set_zlabel("z (m)")
ax.set_xlim(-0.6, 0.6)
ax.set_ylim(-0.6, 0.6)
ax.set_zlim(0.0, 4.5)

# Paper-like view angle
ax.view_init(elev=22, azim=-40)

ax.legend(loc="upper right")
savefig("3D_trajectory.png")


# ----------- POSITION TRACKING ----------------
plt.figure(figsize=(9,7))
plt.subplot(3,1,1)
plt.plot(t, x, label="x")
plt.plot(t, [xr_ref(tt) for tt in t], "--", label="x_ref")
plt.grid(); plt.legend()

plt.subplot(3,1,2)
plt.plot(t, y, label="y")
plt.plot(t, [yr_ref(tt) for tt in t], "--", label="y_ref")
plt.grid(); plt.legend()

plt.subplot(3,1,3)
plt.plot(t, z, label="z")
plt.plot(t, [zr_ref(tt) for tt in t], "--", label="z_ref")
plt.grid(); plt.legend()
plt.suptitle("Position Tracking")
savefig("position_tracking.png")


# ----------- ATTITUDE TRACKING ----------------
plt.figure(figsize=(8,4))
plt.plot(t, phi, label="phi")
plt.plot(t, theta, label="theta")
plt.plot(t, psi, label="psi")
plt.grid(); plt.legend()
plt.title("Attitude Tracking")
savefig("attitude_tracking.png")


# ----------- CONTROL INPUTS -------------------
plt.figure(figsize=(9,6))
plt.subplot(2,1,1)
plt.plot(t, uF, label="uF")
plt.grid(); plt.legend()

plt.subplot(2,1,2)
plt.plot(t, tau_x, label="tau_x")
plt.plot(t, tau_y, label="tau_y")
plt.plot(t, tau_z, label="tau_z")
plt.grid(); plt.legend()
plt.suptitle("Control Inputs")
savefig("control_inputs.png")


# ----------- TRACKING ERRORS ------------------
ex = x - np.array([xr_ref(tt) for tt in t])
ey = y - np.array([yr_ref(tt) for tt in t])
ez = z - np.array([zr_ref(tt) for tt in t])

plt.figure(figsize=(9,4))
plt.plot(t, ex, label="ex")
plt.plot(t, ey, label="ey")
plt.plot(t, ez, label="ez")
plt.grid(); plt.legend()
plt.title("Tracking Errors")
savefig("tracking_errors.png")


# ----------- SAVE NUMERICAL RESULTS ------------
results = {
    "t": t,
    "x": x, "y": y, "z": z,
    "vx": vx, "vy": vy, "vz": vz,
    "phi": phi, "theta": theta, "psi": psi,
    "uF": uF,
    "tau_x": tau_x, "tau_y": tau_y, "tau_z": tau_z,
    "ex": ex, "ey": ey, "ez": ez
}

with open(os.path.join(OUTDIR, "results.pkl"), "wb") as f:
    pickle.dump(results, f)

print("[DONE] All outputs saved to", OUTDIR)

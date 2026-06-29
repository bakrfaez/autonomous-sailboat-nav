# -*- coding: utf-8 -*-
"""
NAVtest4 -- Autonomous Sailboat: Dry-Run Navigation Test
Q1-journal figure: 2-panel layout
  (a) GPS trajectory in local ENU coordinates (East / North, metres)
  (b) Distance-to-target convergence profile
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from scipy.optimize import least_squares

# =============================================================================
# Publication style
# =============================================================================
plt.rcParams.update({
    'font.family':      'serif',
    'font.serif':       ['Times New Roman', 'DejaVu Serif'],
    'font.size':        9.5,
    'axes.labelsize':   10.5,
    'axes.titlesize':   10.5,
    'legend.fontsize':  9,
    'xtick.labelsize':  9,
    'ytick.labelsize':  9,
    'axes.linewidth':   0.9,
    'lines.linewidth':  1.5,
    'grid.linewidth':   0.4,
    'grid.alpha':       0.45,
    'savefig.dpi':      300,
    'pdf.fonttype':     42,
    'ps.fonttype':      42,
})

# =============================================================================
# Load and clean data
# =============================================================================
CSV = r'c:\Users\abuba\Downloads\the sailboat projsct\main\NAVtest4.CSV'
df  = pd.read_csv(CSV)
df['Time_s'] = df['Time_ms'] / 1000.0

# Remove GPS longitude-sign glitches (Birmingham UK: lng ~ -1.89)
df = df[df['GPS_Lng'] < 0].copy().reset_index(drop=True)

# =============================================================================
# Local ENU coordinate conversion
# =============================================================================
nav_mask = (df['State'] == 1) & (df['Navigating'] == 1)
REF      = df[nav_mask].iloc[0]
LAT0, LNG0 = REF['GPS_Lat'], REF['GPS_Lng']

LAT_M = 111_000.0
LNG_M = 111_000.0 * np.cos(np.radians(LAT0))

def to_enu(lat, lng):
    return (lng - LNG0) * LNG_M, (lat - LAT0) * LAT_M

df['east_m'], df['north_m'] = to_enu(df['GPS_Lat'], df['GPS_Lng'])

# =============================================================================
# Phase separation
# =============================================================================
nav_df  = df[nav_mask].copy().reset_index(drop=True)
T0_NAV  = nav_df['Time_s'].iloc[0]
T1_NAV  = nav_df['Time_s'].iloc[-1]
nav_df['t_nav'] = nav_df['Time_s'] - T0_NAV

# Post-arrival: first 15 unique GPS positions only (avoid cluttered map)
post_df = (df[(df['State'] == 0) & (df['Navigating'] == 0)
               & (df['Time_s'] > T1_NAV + 0.5)]
           .drop_duplicates(subset=['GPS_Lat', 'GPS_Lng'])
           .head(15)
           .copy()
           .reset_index(drop=True))

# =============================================================================
# Estimate target position via nonlinear least squares
# =============================================================================
nav_gps = nav_df.drop_duplicates(subset=['GPS_Lat', 'GPS_Lng']).copy().reset_index(drop=True)
e_obs = nav_gps['east_m'].values
n_obs = nav_gps['north_m'].values
d_obs = nav_gps['Dist_m'].values

def residuals(p):
    return np.sqrt((p[0] - e_obs)**2 + (p[1] - n_obs)**2) - d_obs

min_idx = d_obs.argmin()
x0      = [e_obs[min_idx], n_obs[min_idx]]   # seed at closest observed position
sol     = least_squares(residuals, x0, loss='soft_l1', f_scale=2.0)
TGT_E, TGT_N = sol.x

# =============================================================================
# Detect GPS position jumps > 3 m during navigation
# =============================================================================
jump_idx = []
for i in range(1, len(nav_gps)):
    de = nav_gps.loc[i, 'east_m'] - nav_gps.loc[i-1, 'east_m']
    dn = nav_gps.loc[i, 'north_m'] - nav_gps.loc[i-1, 'north_m']
    if de*de + dn*dn > 9.0:
        jump_idx.append(i)

# =============================================================================
# Figure layout
# =============================================================================
fig = plt.figure(figsize=(9.0, 6.5))

gs = gridspec.GridSpec(
    1, 2,
    width_ratios = [1.45, 1.0],
    wspace       = 0.38,
    left         = 0.09,
    right        = 0.96,
    top          = 0.94,
    bottom       = 0.09,
)

ax_map  = fig.add_subplot(gs[0])
ax_dist = fig.add_subplot(gs[1])

CMAP = plt.cm.plasma
N    = len(nav_gps)

# =============================================================================
# (a)  GPS Trajectory Map
# =============================================================================

# Continuous trajectory coloured by elapsed time
for i in range(N - 1):
    x1 = nav_gps.loc[i,   'east_m'];  y1 = nav_gps.loc[i,   'north_m']
    x2 = nav_gps.loc[i+1, 'east_m'];  y2 = nav_gps.loc[i+1, 'north_m']
    frac  = i / max(N - 2, 1)
    color = CMAP(0.12 + 0.76 * frac)
    jump  = (i + 1) in jump_idx

    if jump:
        ax_map.plot([x1, x2], [y1, y2], '--',
                    color='#AAAAAA', linewidth=0.9, alpha=0.65, zorder=2)
    else:
        ax_map.plot([x1, x2], [y1, y2], '-',
                    color=color, linewidth=2.0,
                    solid_capstyle='round', zorder=3)

# GPS waypoint dots
fracs = np.linspace(0, 1, N)
ax_map.scatter(nav_gps['east_m'], nav_gps['north_m'],
               c=fracs, cmap='plasma', vmin=0, vmax=1,
               s=16, zorder=4, alpha=0.90, edgecolors='none')

# Arrival zone circle (7 m radius)
circle = Circle((TGT_E, TGT_N), 7.0,
                fill=False, linestyle=':', linewidth=1.2,
                edgecolor='#CC2222', alpha=0.80, zorder=2)
ax_map.add_patch(circle)

# Target marker + annotation (placed to avoid overlap with trajectory)
ax_map.plot(TGT_E, TGT_N, '*',
            color='#CC2222', markersize=16, zorder=6,
            markeredgecolor='#800000', markeredgewidth=0.6)
ax_map.annotate('Target $G$',
                xy=(TGT_E, TGT_N),
                xytext=(TGT_E + 5.0, TGT_N + 1.5),
                fontsize=8.5, color='#CC2222', ha='left',
                arrowprops=dict(arrowstyle='->', color='#CC2222', lw=0.7),
                zorder=7)

# Navigation start marker
ax_map.plot(0, 0, 'D', color='#1B7837', markersize=9, zorder=6,
            markeredgecolor='#00441B', markeredgewidth=0.8)
ax_map.text(0.6, -1.8, 'Start', fontsize=8.5, color='#1B7837',
            ha='left', va='top', zorder=7)

# Arrival detection point
if len(post_df) > 0:
    ae, an = to_enu(post_df.iloc[0]['GPS_Lat'], post_df.iloc[0]['GPS_Lng'])
    ax_map.plot(ae, an, 'o', color='#FF8000', markersize=10, zorder=6,
                markeredgecolor='#7F4000', markeredgewidth=0.8)
    ax_map.text(ae + 0.8, an + 1.0, 'Arrived', fontsize=8.5,
                color='#FF8000', ha='left', va='bottom', zorder=7)

# Post-arrival drift (manual movement, limited to 15 positions)
if len(post_df) > 1:
    ax_map.plot(post_df['east_m'], post_df['north_m'], '-',
                color='#BBBBBB', linewidth=1.0, alpha=0.70,
                zorder=1, label='Post-arrival (manual)')

# Dashed straight-line reference: start -> target
ax_map.annotate('', xy=(TGT_E, TGT_N), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='#888888',
                                lw=0.7, linestyle='dashed'))
mid_e = TGT_E / 2
mid_n = TGT_N / 2
ax_map.text(mid_e + 2.5, mid_n - 1.5,
            '{:.0f} m'.format(nav_df['Dist_m'].iloc[0]),
            fontsize=8, color='#777777', ha='left', va='top', style='italic')

# Colorbar (elapsed navigation time)
sm = ScalarMappable(cmap='plasma', norm=Normalize(0, nav_df['t_nav'].max()))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax_map,
                    shrink=0.58, pad=0.025, aspect=24, location='right')
cbar.set_label('Elapsed nav. time (s)', fontsize=8.5)
cbar.ax.tick_params(labelsize=8)

# Legend
leg_handles = [
    Line2D([0], [0], color=CMAP(0.5),  lw=2.0, label='Navigation path'),
    Line2D([0], [0], color='#AAAAAA',  lw=0.9, ls='--', label='GPS update (>3 m jump)'),
    Line2D([0], [0], color='#CC2222',  lw=1.2, ls=':', label='Arrival zone (7 m)'),
    Line2D([0], [0], color='#BBBBBB',  lw=1.0, label='Post-arrival (manual)'),
]
ax_map.legend(handles=leg_handles,
              loc='upper right', framealpha=0.92, edgecolor='#CCCCCC',
              fontsize=8, handlelength=1.6, borderpad=0.6)

ax_map.set_xlabel('Easting (m)', labelpad=4)
ax_map.set_ylabel('Northing (m)', labelpad=4)
ax_map.set_title('(a)  GPS Trajectory', loc='left', fontweight='bold', pad=5)
ax_map.set_aspect('equal')
ax_map.grid(True, linestyle=':', alpha=0.4)

# Balanced axis limits — must include start point (0, 0) and target
all_e = list(nav_gps['east_m']) + list(post_df['east_m']) + [TGT_E, 0.0]
all_n = list(nav_gps['north_m']) + list(post_df['north_m']) + [TGT_N, 0.0]
pad = 3.0
ax_map.set_xlim(min(all_e) - pad,  max(all_e) + pad + 8)
ax_map.set_ylim(min(all_n) - pad,  max(all_n) + pad + 3)

# =============================================================================
# (b)  Distance-to-Target Convergence Profile
# =============================================================================
t  = nav_df['t_nav'].values
d  = nav_df['Dist_m'].values
T_TOTAL = t[-1]

# Shaded fill under curve
ax_dist.fill_between(t, d, alpha=0.10, color='#1565C0', zorder=1)

# Coloured line (same plasma mapping as map)
for i in range(len(t) - 1):
    frac  = i / max(len(t) - 2, 1)
    color = CMAP(0.12 + 0.76 * frac)
    ax_dist.plot(t[i:i+2], d[i:i+2], '-', color=color,
                 linewidth=1.8, solid_capstyle='round', zorder=3)

# Arrival threshold
ax_dist.axhline(7.0, color='#CC2222', linestyle='--',
                linewidth=1.1, alpha=0.90, zorder=2)
ax_dist.fill_between([0, T_TOTAL + 4], 0, 7,
                     alpha=0.06, color='#CC2222', zorder=1)
ax_dist.text(T_TOTAL * 0.92, 3.5, 'Arrival zone',
             fontsize=8, color='#CC2222', ha='right', va='center',
             style='italic')

# Arrival vertical marker
ax_dist.axvline(T_TOTAL, color='#FF8000', linestyle=':',
                linewidth=1.3, zorder=2)
ax_dist.annotate(
    'Arrived\n$d$ = {:.1f} m'.format(d[-1]),
    xy=(T_TOTAL, d[-1]),
    xytext=(T_TOTAL - 35, 15),
    fontsize=8.5, color='#FF8000', ha='center',
    arrowprops=dict(arrowstyle='->', color='#FF8000',
                    lw=0.9, relpos=(0.5, 0)),
    bbox=dict(boxstyle='round,pad=0.28', fc='white',
              ec='#FF8000', alpha=0.92, lw=0.7),
    zorder=5,
)

# GPS jump markers (light dotted verticals)
for ji in jump_idx:
    tj = nav_gps.loc[ji, 'Time_s'] - T0_NAV
    if 0 < tj < T_TOTAL:
        ax_dist.axvline(tj, color='#BBBBBB', linestyle=':',
                        linewidth=0.7, alpha=0.70, zorder=1)

# Zigzag span annotation spanning the bulk of nav (5s to 95s)
zz_lo, zz_hi = 5, min(95, T_TOTAL - 5)
ax_dist.annotate('', xy=(zz_hi, 51.5), xytext=(zz_lo, 51.5),
                 arrowprops=dict(arrowstyle='<->', color='#666666', lw=0.8))
ax_dist.text((zz_lo + zz_hi) / 2, 52.4, 'Simulated zigzag',
             fontsize=8, color='#666666', ha='center', va='bottom',
             style='italic')

# Legend
leg2 = [
    Line2D([0], [0], color=CMAP(0.5),  lw=1.8,  label='Distance to target'),
    Line2D([0], [0], color='#CC2222',  lw=1.1, ls='--', label='Arrival threshold (7 m)'),
    Line2D([0], [0], color='#BBBBBB',  lw=0.8, ls=':',  label='GPS position update'),
]
ax_dist.legend(handles=leg2,
               loc='upper right', framealpha=0.92, edgecolor='#CCCCCC',
               fontsize=8, handlelength=1.6, borderpad=0.6)

ax_dist.set_xlabel('Elapsed navigation time (s)', labelpad=4)
ax_dist.set_ylabel('Distance to target (m)', labelpad=4)
ax_dist.set_title('(b)  Convergence Profile', loc='left', fontweight='bold', pad=5)
ax_dist.set_xlim(-1, T_TOTAL + 4)
ax_dist.set_ylim(-0.5, max(d) * 1.15)
ax_dist.grid(True, linestyle=':', alpha=0.4)

# =============================================================================
# Save
# =============================================================================
OUT_PDF = r'c:\Users\abuba\Downloads\the sailboat projsct\main\NAVtest4_figure.pdf'
OUT_PNG = r'c:\Users\abuba\Downloads\the sailboat projsct\main\NAVtest4_figure.png'

plt.savefig(OUT_PDF, bbox_inches='tight', dpi=300)
plt.savefig(OUT_PNG, bbox_inches='tight', dpi=300)
print("Saved NAVtest4_figure.pdf / .png")
print("Target: E={:.2f} m, N={:.2f} m  |  GPS jumps: {}".format(TGT_E, TGT_N, len(jump_idx)))

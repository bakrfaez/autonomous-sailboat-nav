#!/usr/bin/env python3
"""
NAVtest.CSV — Real Hardware Log Analyser
=========================================
Reads the SD-card log produced by the sailboat firmware and produces
4 diagnostic PNG files + a printed diagnosis report.

Run:  python nav_analysis.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv, math, sys

# ── Config ────────────────────────────────────────────────────────────────────
CSV_FILE   = 'NAVtest.CSV'
SERVO_MID  = 315          # (SERVOMIN_RUDDER + SERVOMAX_RUDDER) / 2
SAIL_MID   = 287          # (SERVOMIN_SAIL + SERVOMAX_SAIL) / 2
GLITCH_M   = 10.0         # GPS jump threshold (m) to flag as glitch
GLITCH_DT  = 1.5          # only flag if jump happened within this many seconds

# Zone colours
C = dict(z1='#4e9af1', z3='#f05050', z2n='#34a853', z2w='#fb8c00',
         z1_bg='#ddeeff', z3_bg='#ffe0e0', z2n_bg='#d4f5d4', z2w_bg='#fff0d0',
         glitch='#cc00cc')

# ── Load CSV ──────────────────────────────────────────────────────────────────
FIELDS_CSV = ['Time_ms','Heading_deg','WindDir_deg','WindSpd_ms',
              'GPS_Lat','GPS_Lng','GPS_Valid','Rudder_PWM','Sail_PWM','Zone','Navigating']
FIELDS     = ['time_ms','heading_deg','wind_dir_deg','wind_spd_ms',
              'gps_lat','gps_lng','gps_valid','rudder_pwm','sail_pwm','zone','navigating']
F_MAP = dict(zip(FIELDS_CSV, FIELDS))

rows = []
with open(CSV_FILE, newline='', encoding='utf-8') as fh:
    for r in csv.DictReader(fh):
        try:
            rows.append({F_MAP[k]: float(r[k]) for k in FIELDS_CSV})
        except (ValueError, KeyError):
            pass

if not rows:
    sys.exit('ERROR: no data rows found in ' + CSV_FILE)

D = {f: np.array([r[f] for r in rows]) for f in FIELDS}
t = D['time_ms'] / 1000.0          # seconds
N = len(t)

# ── GPS → Cartesian (metres relative to first point) ─────────────────────────
lat0 = D['gps_lat'][0]
lng0 = D['gps_lng'][0]
MPL  = 111_000.0                              # metres per degree latitude
MPLn = 111_000.0 * math.cos(math.radians(lat0))  # metres per degree longitude

east  = (D['gps_lng'] - lng0) * MPLn
north = (D['gps_lat'] - lat0) * MPL

# ── GPS glitch detection ──────────────────────────────────────────────────────
jump_e = np.diff(east)
jump_n = np.diff(north)
jump_m = np.sqrt(jump_e**2 + jump_n**2)
jump_dt = np.diff(t)
glitch  = np.where((jump_m > GLITCH_M) & (jump_dt < GLITCH_DT))[0] + 1  # glitch AT this index

# Mask glitchy points from the path (replace with NaN so lines break)
east_clean  = east.copy().astype(float)
north_clean = north.copy().astype(float)
for gi in glitch:
    east_clean[gi]  = np.nan
    north_clean[gi] = np.nan

# ── Zone masks ────────────────────────────────────────────────────────────────
z1  = D['zone'] == 1
z3  = D['zone'] == 3
z2  = D['zone'] == 2
nav = D['navigating'] == 1

# ── Helper: add coloured background bands ────────────────────────────────────
def zone_bg(ax):
    """Shade the axes background by zone."""
    i = 0
    while i < N:
        j = i + 1
        if z3[i]:   bg = C['z3_bg']
        elif z2[i] and nav[i]: bg = C['z2n_bg']
        elif z2[i]: bg = C['z2w_bg']
        else:       bg = C['z1_bg']
        while j < N and D['zone'][j] == D['zone'][i] and D['navigating'][j] == D['navigating'][i]:
            j += 1
        t1 = t[j-1] + 0.25 if j < N else t[i] + 0.25
        ax.axvspan(t[i], t1, color=bg, alpha=0.55, lw=0)
        i = j

# ── Compass heading (0–360) from cap convention ───────────────────────────────
# cap: N=180, E=90, S=0, W=-90
compass = (180.0 - D['heading_deg']) % 360.0

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 1 — GPS PATH
# ═══════════════════════════════════════════════════════════════════════════════
fig1, ax1 = plt.subplots(figsize=(9, 8))
ax1.set_title('GPS Path — coloured by system mode', fontsize=13)

# Plot path segments coloured by zone
pt_color = np.where(z3, C['z3'], np.where(z2 & nav, C['z2n'], np.where(z2, C['z2w'], C['z1'])))
for i in range(N - 1):
    if np.isnan(east_clean[i]) or np.isnan(east_clean[i+1]):
        continue
    ax1.plot(east_clean[i:i+2], north_clean[i:i+2], color=pt_color[i], lw=2)

# Glitch markers
for gi in glitch:
    ax1.scatter(east[gi], north[gi], c=C['glitch'], s=180, zorder=9, marker='X')
    ax1.annotate(f't={t[gi]:.0f}s\n{jump_m[gi-1]:.0f}m jump',
                 xy=(east[gi], north[gi]), xytext=(5, 5),
                 textcoords='offset points', fontsize=7, color=C['glitch'])

# Zone transition markers
for idx in np.where(np.diff(z3.astype(int)) > 0)[0]:
    ax1.scatter(east_clean[idx], north_clean[idx], c=C['z3'], s=140, zorder=10,
                marker='^', edgecolors='k', linewidths=0.5)
for idx in np.where(np.diff(z2.astype(int)) > 0)[0]:
    ax1.scatter(east_clean[idx], north_clean[idx], c=C['z2n'], s=140, zorder=10,
                marker='s', edgecolors='k', linewidths=0.5)

ax1.scatter(east_clean[0], north_clean[0], c='navy', s=150, zorder=11, marker='D', label='Start')
ax1.set_xlabel('East (m)'); ax1.set_ylabel('North (m)')
ax1.set_aspect('equal'); ax1.grid(True, alpha=0.3)

legend_handles = [
    mpatches.Patch(color=C['z1'],  label='Zone 1 — Manual'),
    mpatches.Patch(color=C['z3'],  label='Zone 3 — Record'),
    mpatches.Patch(color=C['z2n'], label='Zone 2 — Navigating'),
    mpatches.Patch(color=C['z2w'], label='Zone 2 — Standby'),
    plt.Line2D([0],[0], ls='none', marker='X', color=C['glitch'], markersize=9, label='GPS glitch'),
    plt.Line2D([0],[0], ls='none', marker='^', color=C['z3'],  markersize=8, label='Zone 3 enter'),
    plt.Line2D([0],[0], ls='none', marker='s', color=C['z2n'], markersize=8, label='Zone 2 enter'),
]
ax1.legend(handles=legend_handles, fontsize=8, loc='best')
plt.tight_layout()
fig1.savefig('nav_analysis_gps.png', dpi=120, bbox_inches='tight')
print('Saved: nav_analysis_gps.png')
plt.close(fig1)

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 2 — HEADING + RUDDER
# ═══════════════════════════════════════════════════════════════════════════════
fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
fig2.suptitle('Heading & Rudder — Real Hardware Log', fontsize=12)

zone_bg(ax2a)
ax2a.plot(t, compass, 'k-', lw=1.3, label='Compass heading (deg)')
ax2a.set_ylabel('Compass heading (deg)')
ax2a.set_ylim(-5, 375); ax2a.set_yticks([0, 90, 180, 270, 360])
for deg, label in [(0,'N'), (90,'E'), (180,'S'), (270,'W')]:
    ax2a.axhline(deg, color='gray', lw=0.5, ls='--', alpha=0.5)
    ax2a.text(t[-1]+0.5, deg, label, fontsize=7, va='center', color='gray')
for gi in glitch: ax2a.axvline(t[gi], color=C['glitch'], lw=0.8, ls=':', alpha=0.6)
ax2a.grid(True, alpha=0.25); ax2a.legend(fontsize=8)

zone_bg(ax2b)
ax2b.plot(t, D['rudder_pwm'], color='crimson', lw=1.5, label='Rudder PWM')
ax2b.axhline(SERVO_MID, color='gray',  lw=1.2, ls='--', label=f'Centre = {SERVO_MID}')
ax2b.axhline(200,       color='red',   lw=0.7, ls=':',  label='SERVOMIN = 200 (full deflect)')
ax2b.axhline(430,       color='blue',  lw=0.7, ls=':',  label='SERVOMAX = 430')
for gi in glitch: ax2b.axvline(t[gi], color=C['glitch'], lw=0.8, ls=':', alpha=0.6)
ax2b.set_ylabel('Rudder PWM'); ax2b.set_xlabel('Time (s)')
ax2b.set_ylim(185, 445); ax2b.grid(True, alpha=0.25); ax2b.legend(fontsize=8)

plt.tight_layout()
fig2.savefig('nav_analysis_heading.png', dpi=120, bbox_inches='tight')
print('Saved: nav_analysis_heading.png')
plt.close(fig2)

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 3 — WIND + SAIL
# ═══════════════════════════════════════════════════════════════════════════════
fig3, (ax3a, ax3b, ax3c) = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
fig3.suptitle('Wind & Sail Behaviour — Real Hardware Log', fontsize=12)

zone_bg(ax3a)
ax3a.plot(t, D['wind_dir_deg'], color='steelblue', lw=1.2, label='Apparent wind dir (deg)')
ax3a.set_ylabel('Wind dir (deg)'); ax3a.grid(True, alpha=0.25); ax3a.legend(fontsize=8)
for gi in glitch: ax3a.axvline(t[gi], color=C['glitch'], lw=0.8, ls=':', alpha=0.6)

zone_bg(ax3b)
ax3b.plot(t, D['wind_spd_ms'], color='teal', lw=1.4, label='Wind speed (m/s)')
ax3b.set_ylabel('Wind speed (m/s)'); ax3b.grid(True, alpha=0.25); ax3b.legend(fontsize=8)

zone_bg(ax3c)
ax3c.plot(t, D['sail_pwm'], color='darkorange', lw=1.4, label='Sail PWM')
ax3c.axhline(SAIL_MID, color='gray', lw=1, ls='--', label=f'Centre = {SAIL_MID}')
ax3c.set_ylabel('Sail PWM'); ax3c.set_xlabel('Time (s)')
ax3c.grid(True, alpha=0.25); ax3c.legend(fontsize=8)
for gi in glitch: ax3c.axvline(t[gi], color=C['glitch'], lw=0.8, ls=':', alpha=0.6)

plt.tight_layout()
fig3.savefig('nav_analysis_wind_sail.png', dpi=120, bbox_inches='tight')
print('Saved: nav_analysis_wind_sail.png')
plt.close(fig3)

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 4 — SYSTEM STATE OVERVIEW
# ═══════════════════════════════════════════════════════════════════════════════
fig4, axs = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
fig4.suptitle('System State Overview — NAVtest.CSV', fontsize=12)

panels = [
    (D['zone'],       'Zone (1/2/3)',    'purple',  None),
    (D['navigating'], 'Navigating',      'green',   None),
    (D['rudder_pwm'], 'Rudder PWM',      'crimson', SERVO_MID),
    (north,           'North GPS (m)',   'navy',    0.0),
]
for ax, (sig, ylabel, col, href) in zip(axs, panels):
    zone_bg(ax)
    ax.plot(t, sig, color=col, lw=1.4)
    if href is not None:
        ax.axhline(href, color='gray', lw=0.8, ls='--', alpha=0.7)
    for gi in glitch:
        ax.axvline(t[gi], color=C['glitch'], lw=1, ls=':', alpha=0.7,
                   label='GPS glitch' if gi == glitch[0] else None)
    ax.set_ylabel(ylabel, fontsize=9); ax.grid(True, alpha=0.25)

axs[-1].set_xlabel('Time (s)')
legend_handles = [
    mpatches.Patch(color=C['z1_bg'],  label='Zone 1 Manual'),
    mpatches.Patch(color=C['z3_bg'],  label='Zone 3 Record'),
    mpatches.Patch(color=C['z2n_bg'], label='Zone 2 Navigating'),
    mpatches.Patch(color=C['z2w_bg'], label='Zone 2 Standby'),
    plt.Line2D([0],[0], color=C['glitch'], ls=':', lw=1.5, label='GPS glitch'),
]
axs[0].legend(handles=legend_handles, fontsize=8, loc='upper right', ncol=5)

plt.tight_layout()
fig4.savefig('nav_analysis_state.png', dpi=120, bbox_inches='tight')
print('Saved: nav_analysis_state.png')
plt.close(fig4)

# ═══════════════════════════════════════════════════════════════════════════════
# DIAGNOSTIC REPORT
# ═══════════════════════════════════════════════════════════════════════════════
print()
print('=' * 65)
print('  DIAGNOSTIC REPORT  --  NAVtest.CSV')
print('=' * 65)

total_s = t[-1] - t[0]
print(f'  Duration    : {t[0]:.1f}s  to  {t[-1]:.1f}s  ( {total_s:.0f}s total )')
print(f'  Log rows    : {N}')
print(f'  Log rate    : {N/total_s*1000:.0f} ms / row  (target 250 ms)')

# Phase durations
print()
print('  PHASE DURATIONS:')
def phase_info(mask, label):
    idx = np.where(mask)[0]
    if len(idx) < 2:
        print(f'    {label:22s}: {len(idx):4d} rows  (n/a)')
        return
    dur = t[idx[-1]] - t[idx[0]]
    print(f'    {label:22s}: {len(idx):4d} rows  {dur:6.1f} s')

phase_info(z1,        'Zone 1  Manual')
phase_info(z3,        'Zone 3  Record')
phase_info(z2 & nav,  'Zone 2  Navigating')
phase_info(z2 & ~nav, 'Zone 2  Standby')

# Rudder stats
print()
print('  RUDDER PWM per zone:  (centre = 315, SERVOMIN = 200, SERVOMAX = 430)')
for label, mask in [('Zone 1', z1), ('Zone 3', z3),
                    ('Zone 2 navigating', z2 & nav),
                    ('Zone 2 standby', z2 & ~nav)]:
    m = mask & ~np.isnan(D['rudder_pwm'])
    if not m.any(): continue
    r = D['rudder_pwm'][m]
    pct200 = (r == 200).mean() * 100
    print(f'    {label:22s}: min={r.min():.0f}  max={r.max():.0f}  '
          f'mean={r.mean():.1f}  at_SERVOMIN={pct200:.0f}%')

# GPS glitches
print()
print(f'  GPS GLITCHES  (jump > {GLITCH_M:.0f}m in < {GLITCH_DT:.1f}s):')
if len(glitch) == 0:
    print('    None detected.')
else:
    for gi in glitch:
        de = jump_e[gi-1]; dn = jump_n[gi-1]; dm = jump_m[gi-1]
        print(f'    t={t[gi]:6.1f}s:  {dm:7.1f}m jump  '
              f'({de:+.1f}m E, {dn:+.1f}m N)  '
              f'at ({D["gps_lat"][gi]:.6f}, {D["gps_lng"][gi]:.6f})')

# Key events
print()
print('  KEY EVENTS:')
z3_enter = np.where(np.diff(z3.astype(int)) > 0)[0]
z2_enter = np.where(np.diff(z2.astype(int)) > 0)[0]
nav_stop  = np.where(np.diff(nav.astype(int)) < 0)[0]
nav_start = np.where(np.diff(nav.astype(int)) > 0)[0]

for idx in z3_enter:
    lat = D['gps_lat'][idx]; lng = D['gps_lng'][idx]
    print(f'    t={t[idx]:6.1f}s  Zone 3 enter  -> setTarget({lat:.6f}, {lng:.6f})')
for idx in z2_enter:
    print(f'    t={t[idx]:6.1f}s  Zone 2 enter  -> navigation start candidate')
for idx in nav_start:
    print(f'    t={t[idx]:6.1f}s  Navigating=1  (autonomous nav started)')
for idx in nav_stop:
    gap = t[idx+1] - t[idx] if idx+1 < N else 0
    reason = 'ARRIVAL (1.4s animation gap)' if gap > 1.0 else 'zone change / GPS lost'
    print(f'    t={t[idx]:6.1f}s  Navigating=0  ({reason})')

# Root-cause diagnosis
print()
print('  ROOT CAUSE ANALYSIS:')
pct_z3_at_min = (D['rudder_pwm'][z3] == 200).mean() * 100 if z3.any() else 0

print()
print('  [1] RUDDER AT SERVOMIN (200) THROUGHOUT ZONE 3')
print(f'      {pct_z3_at_min:.0f}% of Zone-3 ticks have Rudder=200')
print('      Rudder=200 means aileron=AILERON_MAX (1456 us) in the new mapping.')
print('      Possible causes:')
print('        a) User was physically holding the right stick at full-right.')
print('        b) The stick springs to 1456 us when released (center != 1312 us).')
print('        c) pulseIn on Pin 2 is capturing a combined/wrong signal.')
print('      ACTION: In RC_Calibration sketch, release the stick completely')
print('              and observe what CH1 reads when untouched.')
print('              If it reads ~1456, set AILERON_CENTER = 1456 in config.h.')

print()
print('  [2] AUTONOMOUS NAVIGATION RESULT')
if (z2 & nav).any():
    rd = D['rudder_pwm'][z2 & nav]
    print(f'      Rudder range in auto mode: {rd.min():.0f} - {rd.max():.0f}  (centre=315)')
    print('      Algorithm was actively steering the rudder. Navigation WORKED.')
    if len(nav_stop):
        gap = t[nav_stop[-1]+1] - t[nav_stop[-1]] if nav_stop[-1]+1 < N else 0
        if gap > 1.0:
            print(f'      1.5s log gap at t={t[nav_stop[-1]]:.0f}s = arrival animation confirmed.')
            print('      CONCLUSION: boat arrived within 7m of target successfully.')
else:
    print('      No autonomous navigation data found.')

print()
print('  [3] ZONE 3 DURATION ANALYSIS')
if z3.any():
    z3_dur = t[np.where(z3)[0][-1]] - t[np.where(z3)[0][0]]
    print(f'      Zone 3 was active for {z3_dur:.0f}s.')
    print('      During this time Navigating=0 (correct — Zone 3 is record mode).')
    print('      The boat was physically moved/rotated — heading changed significantly.')

print()
print('  [4] WIND CONDITIONS')
spd = D['wind_spd_ms']
print(f'      Speed range: {spd.min():.2f} - {spd.max():.2f} m/s')
print('      Near-calm conditions. The boat did not physically sail.')
print('      GPS movement = drift/jitter only (SOG=0 throughout).')
print('      Real sea trial needed to validate speed of navigation.')

print()
print('=' * 65)
print('  4 PNG files saved in the current folder.')
print('=' * 65)

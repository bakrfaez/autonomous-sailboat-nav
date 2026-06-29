#!/usr/bin/env python3
"""
NAV027 — Publication figures for every successful autonomous segment.
Generates 2 figures per segment (GPS map + actuator dynamics).
Successful = next row after segment has Dist_m < 7.0 m (Safe Park triggered).
"""

import os, warnings
warnings.filterwarnings('ignore')
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

CSV_PATH = os.path.join(os.path.dirname(__file__), 'NAV027.CSV')
OUT_DIR  = os.path.dirname(__file__)

# ── IEEE style ────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':       'DejaVu Serif',
    'font.size':         10,
    'axes.titlesize':    10,
    'axes.labelsize':    10,
    'xtick.labelsize':   9,
    'ytick.labelsize':   9,
    'legend.fontsize':   8.5,
    'figure.dpi':        300,
    'axes.grid':         True,
    'grid.alpha':        0.25,
    'grid.linestyle':    '--',
    'grid.color':        '#888888',
    'axes.spines.top':   False,
    'axes.spines.right': False,
})

# ── Load & clean ──────────────────────────────────────────────────────────────
print("Loading data...")
df = pd.read_csv(CSV_PATH)
df = df.ffill().reset_index(drop=True)
valid = df['GPS_Lat'].between(52.43, 52.55) & df['GPS_Lng'].between(-1.95, -1.83)
df   = df[valid].reset_index(drop=True)
df['Heading_compass'] = (180.0 - df['Heading_deg']) % 360.0
df['auto'] = (df['State'] == 1) & (df['Navigating'] == 1)
df['grp']  = (df['auto'] != df['auto'].shift()).cumsum()
print(f"  {len(df):,} rows after GPS filter")

# ── Identify successful segments ──────────────────────────────────────────────
successful = []
for grp_id, group in df[df['auto']].groupby('grp'):
    last_idx = group.index[-1]
    if last_idx + 1 < len(df):
        nxt = df.iloc[last_idx + 1]
        if nxt['Dist_m'] < 7.0:
            seg = group.copy()
            seg['Time_s'] = (seg['Time_ms'] - seg['Time_ms'].iloc[0]) / 1000.0
            successful.append({'grp': grp_id, 'df': seg, 'next': nxt})

print(f"  Successful arrivals found: {len(successful)}")
for i, s in enumerate(successful):
    g = s['df']
    print(f"    Run {i+1}: grp={s['grp']}  t={g['Time_ms'].iloc[0]/1000:.0f}s  "
          f"dur={g['Time_s'].iloc[-1]:.0f}s  rows={len(g)}  "
          f"dist {g['Dist_m'].iloc[0]:.1f}m -> {s['next']['Dist_m']:.1f}m")

# ── Geo helpers ───────────────────────────────────────────────────────────────
try:
    from pyproj import Transformer
    _fwd  = Transformer.from_crs('EPSG:4326', 'EPSG:3857',  always_xy=True)
    _utm  = Transformer.from_crs('EPSG:4326', 'EPSG:32630', always_xy=True)
    _back = Transformer.from_crs('EPSG:32630','EPSG:3857',  always_xy=True)
    def to_3857(lats, lngs):
        return _fwd.transform(np.asarray(lngs, float), np.asarray(lats, float))
    def circle_3857(lat, lng, r=7.0, n=360):
        cx, cy = _utm.transform(float(lng), float(lat))
        th = np.linspace(0, 2*np.pi, n)
        return _back.transform(cx + r*np.cos(th), cy + r*np.sin(th))
    HAS_GEO = True
    print("  pyproj: OK")
except ImportError:
    HAS_GEO = False
    print("  pyproj: not found")

try:
    import contextily as ctx
    HAS_CTX = True
    print("  contextily: OK")
except ImportError:
    HAS_CTX = False
    print("  contextily: not found")

# ═════════════════════════════════════════════════════════════════════════════
# FIGURE A — GPS trajectory on satellite map
# ═════════════════════════════════════════════════════════════════════════════
def make_gps_fig(seg, run_num, total):
    g       = seg['df']
    nxt     = seg['next']
    dur_s   = g['Time_s'].iloc[-1]
    dist_i  = g['Dist_m'].iloc[0]
    dist_f  = nxt['Dist_m']
    slat, slng = g['GPS_Lat'].iloc[0],  g['GPS_Lng'].iloc[0]
    alat, alng = g['GPS_Lat'].iloc[-1], g['GPS_Lng'].iloc[-1]

    fig, ax = plt.subplots(figsize=(7, 7))

    if HAS_GEO and HAS_CTX:
        sx, sy     = to_3857([slat], [slng])
        ax_, ay_   = to_3857([alat], [alng])
        gx, gy     = to_3857(g['GPS_Lat'].values, g['GPS_Lng'].values)
        cx_c, cy_c = circle_3857(alat, alng, 7.0)

        # Square view: minimum 20 m half-span + 30 m padding
        all_x = np.concatenate([gx, sx, ax_])
        all_y = np.concatenate([gy, sy, ay_])
        cx = (all_x.min() + all_x.max()) / 2
        cy = (all_y.min() + all_y.max()) / 2
        half = max((all_x.max()-all_x.min())/2,
                   (all_y.max()-all_y.min())/2, 20.0) + 30.0
        ax.set_xlim(cx-half, cx+half)
        ax.set_ylim(cy-half, cy+half)

        # Basemap zoom=19
        for src, zm in [(ctx.providers.Esri.WorldImagery,    19),
                        (ctx.providers.OpenStreetMap.Mapnik, 19),
                        (ctx.providers.CartoDB.Positron,     19)]:
            try:
                ctx.add_basemap(ax, crs='EPSG:3857', source=src, zoom=zm,
                                attribution_size=5, reset_extent=False)
                break
            except Exception:
                pass
        ax.set_xlim(cx-half, cx+half)
        ax.set_ylim(cy-half, cy+half)

        # Trajectory
        if len(gx) > 1:
            t_ms   = g['Time_ms'].values.astype(float)
            t_norm = (t_ms - t_ms.min()) / max(t_ms.max()-t_ms.min(), 1)
            pts    = np.column_stack([gx, gy]).reshape(-1, 1, 2)
            segs_  = np.concatenate([pts[:-1], pts[1:]], axis=1)
            lc = LineCollection(segs_, cmap='plasma', norm=plt.Normalize(0,1),
                                linewidth=2.8, zorder=4, alpha=0.97)
            lc.set_array((t_norm[:-1]+t_norm[1:])/2)
            ax.add_collection(lc)
            cbar = fig.colorbar(lc, ax=ax, pad=0.01, fraction=0.03, shrink=0.72, aspect=25)
            cbar.set_label('Time progression', fontsize=8)
            cbar.set_ticks([0, 1])
            cbar.set_ticklabels([f't = 0 s', f't = {dur_s:.0f} s'])
            cbar.ax.tick_params(labelsize=7.5)

        # Circle, markers
        ax.plot(cx_c, cy_c, '--', color='#cc2222', lw=1.6, zorder=5, alpha=0.95,
                label='7 m Arrival Tolerance')
        ax.scatter(sx, sy, marker='D', s=160, color='#00cc55',
                   edgecolors='#003300', lw=1.0, zorder=7, label='Start')
        ax.scatter(ax_, ay_, marker='*', s=400, color='#cc2222',
                   edgecolors='#330000', lw=0.8, zorder=7, label='Arrival / Target')

        # Relative-metre labels
        ref_x, ref_y = cx, cy
        ax.xaxis.set_major_formatter(
            mticker.FuncFormatter(lambda v, _: f'{v-ref_x:+.0f} m'))
        ax.yaxis.set_major_formatter(
            mticker.FuncFormatter(lambda v, _: f'{v-ref_y:+.0f} m'))
        ax.set_xlabel('Relative Easting (m)', fontsize=10)
        ax.set_ylabel('Relative Northing (m)', fontsize=10)

        # Scale bar 10 m
        xl, yl = ax.get_xlim(), ax.get_ylim()
        bx0 = xl[0] + 0.04*(xl[1]-xl[0])
        by0 = yl[0] + 0.05*(yl[1]-yl[0])
        ax.plot([bx0, bx0+10], [by0, by0], color='white', lw=6,
                solid_capstyle='butt', zorder=8)
        ax.plot([bx0, bx0+10], [by0, by0], color='black', lw=3,
                solid_capstyle='butt', zorder=9)
        ax.text(bx0+5, by0+1.2, '10 m', ha='center', va='bottom',
                fontsize=8, color='white', fontweight='bold', zorder=10)

    # Legend & title
    leg = [
        Patch(facecolor=plt.cm.plasma(0.5), edgecolor='none',
              label='Trajectory  (plasma = time)'),
        Line2D([0],[0], color='#cc2222', lw=1.6, ls='--',
               label='7 m Arrival Tolerance'),
        Line2D([0],[0], marker='D', color='w', mfc='#00cc55',
               mec='#003300', ms=9, label='Start Point'),
        Line2D([0],[0], marker='*', color='w', mfc='#cc2222',
               mec='#330000', ms=14, label='Arrival / Target'),
    ]
    ax.legend(handles=leg, loc='upper left', framealpha=0.90,
              edgecolor='#444444', fontsize=8.5)
    ax.set_title(
        f'Figure 1.{run_num}: GPS Trajectory — Successful Autonomous Run {run_num} of {total}\n'
        f'Duration: {dur_s:.0f} s    Start dist: {dist_i:.1f} m    Final dist: {dist_f:.1f} m',
        fontsize=10, fontweight='bold', pad=7)
    ax.tick_params(labelsize=8.5)

    fig.tight_layout()
    base = os.path.join(OUT_DIR, f'NAV027_Run{run_num:02d}_GPS')
    fig.savefig(base+'.png', dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(base+'.pdf', dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    GPS fig saved: {base}.png")


# ═════════════════════════════════════════════════════════════════════════════
# FIGURE B — Closed-loop actuator dynamics
# ═════════════════════════════════════════════════════════════════════════════
def make_dyn_fig(seg, run_num, total):
    g      = seg['df']
    nxt    = seg['next']
    t      = g['Time_s'].values
    dur_s  = t[-1]
    dist_i = g['Dist_m'].iloc[0]
    dist_f = nxt['Dist_m']
    n_rows = len(g)

    # Use raw cap-convention heading (S=0, reference direction for canal runs)
    heading  = g['Heading_deg'].values
    wind_dir = g['WindDir_deg'].values
    rudder   = g['Rudder_PWM'].values
    sail     = g['Sail_PWM'].values

    C_HEAD, C_RUD  = '#111111', '#cc2222'
    C_WIND, C_SAIL = '#007575', '#7b2d8b'
    GKW = dict(linestyle='--', alpha=0.20, color='#999999')

    fig, (axA, axB) = plt.subplots(2, 1, figsize=(12, 8.5), sharex=True)
    fig.subplots_adjust(hspace=0.42, top=0.88, bottom=0.09,
                        left=0.08, right=0.87)
    fig.suptitle(
        f'Figure 2.{run_num}: Actuator Dynamics During Autonomous Navigation Phase — NAV027.CSV\n'
        f'Run {run_num} of {total}  |  {n_rows} samples  |  {dur_s:.0f} s  |'
        f'  Start dist: {dist_i:.1f} m  →  Final dist: {dist_f:.1f} m',
        fontsize=11, fontweight='bold')

    # ── Subplot A: Heading + Rudder ───────────────────────────────────────────
    axAr = axA.twinx()

    # Shaded fill between heading line and 0 reference
    axA.fill_between(t, heading, 0, alpha=0.13, color='dimgray', zorder=1)
    axA.axhline(0, color='gray', lw=0.9, ls=':', alpha=0.65, zorder=2)

    lh, = axA.plot(t, heading, color=C_HEAD, lw=1.3, zorder=4,
                   label='Heading (deg)')
    lr, = axAr.plot(t, rudder,  color=C_RUD,  lw=1.1, alpha=0.90, zorder=4,
                    label='Rudder Command (PWM)')

    axA.set_ylabel('Heading (deg)', color=C_HEAD, fontsize=10)
    axAr.set_ylabel('Rudder Command (PWM)', color=C_RUD, fontsize=10)
    axA.tick_params(axis='y', labelcolor=C_HEAD)
    axAr.tick_params(axis='y', labelcolor=C_RUD)
    axA.set_title('(A)  Rudder Closed-Loop Response: Heading Error → Rudder Correction',
                  fontsize=10, fontweight='bold', loc='left', pad=5)
    axA.grid(**GKW)

    leg_A = axA.legend([lh, lr], [lh.get_label(), lr.get_label()],
                       loc='upper left', framealpha=1.0, fancybox=False,
                       edgecolor='#333333', fontsize=9)
    leg_A.set_zorder(25)

    # ── Subplot B: Wind + Sail ────────────────────────────────────────────────
    axBr = axB.twinx()

    # Shaded fill between wind direction and 0 (headwind reference)
    axB.fill_between(t, wind_dir, 0, alpha=0.13, color='teal', zorder=1)
    hw_ref = axB.axhline(0, color=C_WIND, lw=0.9, ls=':', alpha=0.70, zorder=2,
                         label='0° (headwind)')

    lw_, = axB.plot(t, wind_dir, color=C_WIND, lw=1.3, zorder=4,
                    label='Apparent wind dir. (deg)')
    ls_, = axBr.plot(t, sail,    color=C_SAIL, lw=1.1, alpha=0.90, zorder=4,
                     label='Sail command (PWM)')

    axB.set_ylabel('Apparent Wind Direction (deg)', color=C_WIND, fontsize=10)
    axBr.set_ylabel('Sail Command (PWM)',            color=C_SAIL, fontsize=10)
    axB.set_xlabel('Navigation time (s)', fontsize=10)
    axB.tick_params(axis='y', labelcolor=C_WIND)
    axBr.tick_params(axis='y', labelcolor=C_SAIL)
    axB.set_title('(B)  Sail Trim Dynamics: Apparent Wind Direction → Sail Adjustment',
                  fontsize=10, fontweight='bold', loc='left', pad=5)
    axB.grid(**GKW)

    leg_B = axB.legend([lw_, hw_ref, ls_],
                       [lw_.get_label(), hw_ref.get_label(), ls_.get_label()],
                       loc='upper left', framealpha=1.0, fancybox=False,
                       edgecolor='#333333', fontsize=9)
    leg_B.set_zorder(25)

    base = os.path.join(OUT_DIR, f'NAV027_Run{run_num:02d}_Dynamics')
    fig.savefig(base+'.png', dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(base+'.pdf', dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    Dynamics fig saved: {base}.png")


# ═════════════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════════════
print(f"\nGenerating {len(successful)*2} figures ({len(successful)} GPS + {len(successful)} Dynamics)...")
for i, seg in enumerate(successful):
    print(f"\n  Run {i+1}/{len(successful)} (grp={seg['grp']}):")
    make_gps_fig(seg, i+1, len(successful))
    make_dyn_fig(seg, i+1, len(successful))

print("\nDone. Files saved:")
for i in range(1, len(successful)+1):
    print(f"  NAV027_Run{i:02d}_GPS.png / .pdf")
    print(f"  NAV027_Run{i:02d}_Dynamics.png / .pdf")

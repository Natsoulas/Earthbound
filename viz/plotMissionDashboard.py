# /*
#  * Copyright (c) 2024 Niko Natsoulas
#  * 
#  * This source code is licensed under the MIT license found in the
#  * LICENSE file in the root directory of this source tree.
#  */

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.patheffects as pe
from mpl_toolkits.mplot3d import Axes3D
import os
import cartopy.crs as ccrs
import cartopy.feature as cfeature
import cartopy.mpl.gridliner as gd
from matplotlib.widgets import Button
import cartopy.feature as cfeature
import cartopy.mpl.ticker as mticker

# Add this near the top of the file, after imports but before any other functions
def add_gradient_background(ax):
    """Add a subtle gradient background to plots"""
    nx = 100
    ny = 100
    gradient = np.zeros((ny, nx, 4))
    gradient[:,:,3] = np.linspace(0.2, 0, nx)
    ax.imshow(gradient, extent=[ax.get_xlim()[0], ax.get_xlim()[1], 
                               ax.get_ylim()[0], ax.get_ylim()[1]], 
              aspect='auto', zorder=0)

# Use a default matplotlib style that's always available
plt.style.use('default')

# Set some default plot parameters for better visualization
plt.rcParams['figure.figsize'] = [12, 8]
plt.rcParams['axes.grid'] = True
plt.rcParams['font.size'] = 12
plt.rcParams['lines.linewidth'] = 2

# Add these settings after the existing rcParams
plt.style.use('dark_background')
plt.rcParams.update({
    'figure.facecolor': '#1e1e1e',
    'axes.facecolor': '#2d2d2d',
    'axes.edgecolor': '#666666',
    'axes.labelcolor': 'white',
    'text.color': 'white',
    'xtick.color': 'white',
    'ytick.color': 'white',
    'grid.color': '#444444',
    'legend.facecolor': '#2d2d2d',
    'legend.edgecolor': '#666666'
})

# Define a modern color palette with more distinct colors
colors = ['#4cc9f0', '#f72585', '#ffd60a', '#7209b7', '#00f5d4', '#ff9e00']
plt.rcParams['axes.prop_cycle'] = plt.cycler(color=colors)

# Get absolute paths
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
data_dir = os.path.join(project_root, 'data')
docs_dir = os.path.join(project_root, 'docs', 'images')

# Create directories if they don't exist
os.makedirs(data_dir, exist_ok=True)
os.makedirs(docs_dir, exist_ok=True)

# Check if the data file exists
data_file = os.path.join(data_dir, 'satellite_state.csv')
if not os.path.exists(data_file):
    raise FileNotFoundError(
        f"Data file not found: {data_file}\n"
        f"Please run the simulation first:\n"
        f"1. mkdir build && cd build\n"
        f"2. cmake ..\n"
        f"3. make\n"
        f"4. ./Earthbound"
    )

# Read the data
print(f"Loading data from: {data_file}")
df = pd.read_csv(data_file)

# Validate data
expected_columns = [
    "time", "x", "y", "z", "vx", "vy", "vz",
    "q1", "q2", "q3", "q4", "wx", "wy", "wz",
    "lat", "lon", "alt", "tx", "ty", "tz",
    "fx", "fy", "fz",
    "rsw_err_x", "rsw_err_y", "rsw_err_z",
    "apogee_alt", "perigee_alt"
]
missing_cols = set(expected_columns) - set(df.columns)
if missing_cols:
    raise ValueError(f"Missing expected columns: {missing_cols}")

# Print data info
print(f"Data shape: {df.shape}")
print(f"Data columns: {df.columns.tolist()}")
print(f"Time span: {df['time'].min():.1f}s to {df['time'].max():.1f}s")

# Function to create dashboard
def create_dashboard(fig):
    # Clear the figure first
    fig.clear()
    
    # Create global references
    global buttons
    buttons = []
    
    def create_orbit_tab(event=None):
        global buttons
        fig.clear()
        
        # Adjust gridspec to add more spacing
        gs = fig.add_gridspec(2, 2, height_ratios=[1.5, 1], top=0.9, bottom=0.1,
                             hspace=0.3, wspace=0.3)  # Increased horizontal and vertical spacing
        
        # Ground Track (full width, top)
        ax_map = fig.add_subplot(gs[0, :], projection=ccrs.PlateCarree())
        ax_map.add_feature(cfeature.LAND, facecolor='#2d2d2d', edgecolor='#666666')
        ax_map.add_feature(cfeature.OCEAN, facecolor='#1e1e1e')
        ax_map.add_feature(cfeature.COASTLINE, edgecolor='#4cc9f0', linewidth=1)
        
        # Add gridlines
        gl = ax_map.gridlines(draw_labels=True, x_inline=False, y_inline=False,
                             linewidth=0.5, color='gray', alpha=0.5, linestyle='--')
        
        # Customize gridline labels
        gl.top_labels = False  # Don't show labels at top
        gl.right_labels = False  # Don't show labels at right
        gl.xlabel_style = {'size': 8, 'color': 'white'}
        gl.ylabel_style = {'size': 8, 'color': 'white'}
        
        # Set grid line spacing using Cartopy's methods
        gl.xlines = True
        gl.ylines = True
        gl.xlocator = mticker.LongitudeLocator(nbins=6)  # ~60 degree spacing
        gl.ylocator = mticker.LatitudeLocator(nbins=6)   # ~30 degree spacing
        gl.xformatter = mticker.LongitudeFormatter()
        gl.yformatter = mticker.LatitudeFormatter()
        
        # Plot ground track with time-based coloring
        jumps = np.where(np.abs(np.diff(df['lon'])) > 300)[0]
        norm = plt.Normalize(df['time'].min(), df['time'].max())
        
        for lon_seg, lat_seg, time_seg in zip(np.split(df['lon'], jumps + 1),
                                            np.split(df['lat'], jumps + 1),
                                            np.split(df['time'], jumps + 1)):
            points = np.array([lon_seg, lat_seg]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap='plasma', norm=norm)
            lc.set_array(time_seg[:-1])
            ax_map.add_collection(lc)
        
        # Add colorbar with adjusted positioning
        cbar = plt.colorbar(lc, ax=ax_map, orientation='horizontal', 
                           pad=0.08,  # Increased padding between plot and colorbar
                           fraction=0.015)  # Slightly smaller colorbar height
        cbar.set_label('Mission Elapsed Time [hours]')
        cbar.ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, p: f'{x/3600:.1f}'))
        
        # Add start/end markers
        ax_map.plot(df['lon'].iloc[0], df['lat'].iloc[0], 'go', label='Start')
        ax_map.plot(df['lon'].iloc[-1], df['lat'].iloc[-1], 'ro', label='End')
        ax_map.legend()
        
        # 3D Trajectory (bottom left)
        ax_3d = fig.add_subplot(gs[1, 0], projection='3d')
        ax_3d.plot(df['x']/1000, df['y']/1000, df['z']/1000)
        ax_3d.set_xlabel('X [km]')
        ax_3d.set_ylabel('Y [km]')
        ax_3d.set_zlabel('Z [km]')
        ax_3d.set_title('Orbital Trajectory')
        
        # Orbital Altitudes (bottom right)
        ax_alt = fig.add_subplot(gs[1, 1])
        ax_alt.plot(df['time']/3600, df['apogee_alt'], 'r-', label='Apogee')
        ax_alt.plot(df['time']/3600, df['perigee_alt'], 'b-', label='Perigee')
        ax_alt.axhline(y=800, color='k', linestyle='--', label='Target')
        ax_alt.set_xlabel('Time [hours]')
        ax_alt.set_ylabel('Altitude [km]')
        ax_alt.set_title('Orbital Altitudes')
        ax_alt.legend()
        
        # Clear old buttons and create new ones
        buttons.clear()
        for i, name in enumerate(['Orbit', 'Attitude', 'Controls']):
            button_ax = fig.add_axes([0.35 + i*0.12, 0.95, 0.10, 0.03])
            btn = Button(button_ax, name, color='#2d2d2d', hovercolor='#4d4d4d')
            btn.label.set_color('white')
            if name == 'Orbit':
                btn.on_clicked(lambda x: create_orbit_tab())
            elif name == 'Attitude':
                btn.on_clicked(lambda x: create_attitude_tab())
            else:
                btn.on_clicked(lambda x: create_controls_tab())
            buttons.append(btn)
        
        plt.draw()
    
    def create_attitude_tab(event=None):
        global buttons
        fig.clear()
        gs = fig.add_gridspec(2, 2, top=0.9)
        
        # Quaternions
        ax_quat = fig.add_subplot(gs[0, :])
        ax_quat.plot(df['time'], df['q1'], label='q1 (w/scalar)')
        ax_quat.plot(df['time'], df['q2'], label='q2 (x)')
        ax_quat.plot(df['time'], df['q3'], label='q3 (y)')
        ax_quat.plot(df['time'], df['q4'], label='q4 (z)')
        ax_quat.set_xlabel('Time [s]')
        ax_quat.set_ylabel('Component Value')
        ax_quat.set_title('GCRS to Body Quaternion')
        ax_quat.legend()
        
        # Angular Velocity
        ax_ang_vel = fig.add_subplot(gs[1, 0])
        ax_ang_vel.plot(df['time'], df['wx'], label='X')
        ax_ang_vel.plot(df['time'], df['wy'], label='Y')
        ax_ang_vel.plot(df['time'], df['wz'], label='Z')
        ax_ang_vel.set_xlabel('Time [s]')
        ax_ang_vel.set_ylabel('Angular Rate [deg/s]')
        ax_ang_vel.set_title('Body Angular Velocity')
        ax_ang_vel.legend()
        
        # RSW Error
        ax_rsw = fig.add_subplot(gs[1, 1])
        ax_rsw.plot(df['time'], df['rsw_err_x'], 'r-', label='R')
        ax_rsw.plot(df['time'], df['rsw_err_y'], 'g-', label='S')
        ax_rsw.plot(df['time'], df['rsw_err_z'], 'b-', label='W')
        ax_rsw.set_xlabel('Time [s]')
        ax_rsw.set_ylabel('Error [deg]')
        ax_rsw.set_title('RSW Frame Tracking Error')
        ax_rsw.legend()
        
        # Clear old buttons and create new ones
        buttons.clear()
        for i, name in enumerate(['Orbit', 'Attitude', 'Controls']):
            button_ax = fig.add_axes([0.35 + i*0.12, 0.95, 0.10, 0.03])
            btn = Button(button_ax, name, color='#2d2d2d', hovercolor='#4d4d4d')
            btn.label.set_color('white')
            if name == 'Orbit':
                btn.on_clicked(lambda x: create_orbit_tab())
            elif name == 'Attitude':
                btn.on_clicked(lambda x: create_attitude_tab())
            else:
                btn.on_clicked(lambda x: create_controls_tab())
            buttons.append(btn)
        
        plt.draw()
    
    def create_controls_tab(event=None):
        global buttons
        fig.clear()
        gs = fig.add_gridspec(2, 1, top=0.9)
        
        # Control Torques
        ax_torque = fig.add_subplot(gs[0])
        ax_torque.plot(df['time'], df['tx']*1000, label='X')
        ax_torque.plot(df['time'], df['ty']*1000, label='Y')
        ax_torque.plot(df['time'], df['tz']*1000, label='Z')
        ax_torque.set_xlabel('Time [s]')
        ax_torque.set_ylabel('Torque [mN⋅m]')
        ax_torque.set_title('Control Torques')
        ax_torque.legend()
        
        # Control Forces
        ax_forces = fig.add_subplot(gs[1])
        ax_forces.plot(df['time'], df['fx'], label='X')
        ax_forces.plot(df['time'], df['fy'], label='Y')
        ax_forces.plot(df['time'], df['fz'], label='Z')
        ax_forces.set_xlabel('Time [s]')
        ax_forces.set_ylabel('Force [N]')
        ax_forces.set_title('Control Forces')
        ax_forces.legend()
        
        # Clear old buttons and create new ones
        buttons.clear()
        for i, name in enumerate(['Orbit', 'Attitude', 'Controls']):
            button_ax = fig.add_axes([0.35 + i*0.12, 0.95, 0.10, 0.03])
            btn = Button(button_ax, name, color='#2d2d2d', hovercolor='#4d4d4d')
            btn.label.set_color('white')
            if name == 'Orbit':
                btn.on_clicked(lambda x: create_orbit_tab())
            elif name == 'Attitude':
                btn.on_clicked(lambda x: create_attitude_tab())
            else:
                btn.on_clicked(lambda x: create_controls_tab())
            buttons.append(btn)
        
        plt.draw()

    # Show initial tab
    create_orbit_tab()
    
    return fig

# Create and save static dashboard
static_fig = plt.figure(figsize=(15, 10))
create_dashboard(static_fig)
plt.savefig(os.path.join(docs_dir, 'mission_dashboard.png'), dpi=300, bbox_inches='tight')
plt.close()

# Create interactive dashboard
interactive_fig = plt.figure(figsize=(15, 10))
create_dashboard(interactive_fig)

# Create separate interactive 3D plot window
fig3d = plt.figure(figsize=(8, 8))
ax3d = fig3d.add_subplot(111, projection='3d')
ax3d.plot(df['x']/1000, df['y']/1000, df['z']/1000)
ax3d.set_xlabel('X [km]')
ax3d.set_ylabel('Y [km]')
ax3d.set_zlabel('Z [km]')
ax3d.set_title('Interactive Orbital Trajectory')
ax3d.grid(True)

# Make 3D plot interactive
def on_move(event):
    if event.inaxes == ax3d:
        ax3d.view_init(elev=ax3d.elev, azim=ax3d.azim)
        fig3d.canvas.draw_idle()

fig3d.canvas.mpl_connect('motion_notify_event', on_move)

# Show both interactive plots
plt.show()

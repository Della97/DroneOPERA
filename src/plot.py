import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
import pandas as pd
import numpy as np
from matplotlib import cm
from matplotlib.colors import Normalize
#from matplotlib.colors import TwoSlopeNorm
from scipy.interpolate import interp1d
from scipy.interpolate import make_interp_spline

# Try to apply a cleaner style
try:
    plt.style.use('seaborn-v0_8-whitegrid')
except:
    plt.style.use('ggplot')

# Set global font sizes and styling
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Liberation Sans', 'Bitstream Vera Sans', 'sans-serif']
plt.rcParams['font.size'] = 20
plt.rcParams['axes.labelsize'] = 22
plt.rcParams['axes.titlesize'] = 24
plt.rcParams['axes.titleweight'] = 'bold'
plt.rcParams['xtick.labelsize'] = 20
plt.rcParams['ytick.labelsize'] = 20
plt.rcParams['legend.fontsize'] = 20
plt.rcParams['figure.figsize'] = (14, 10)
plt.rcParams['lines.linewidth'] = 3.0
plt.rcParams['grid.alpha'] = 0.5

def read_csv(filename):
    # Read the CSV file and split the data manually by spaces
    df = pd.read_csv(filename, header=None)
    df_split = df[0].str.split(expand=True)

    # Rename the columns for easier access
    df_split.columns = [
        'drone_id', 'x', 'y', 'z', 'battery_usage', 'time',
        'aoi_start_x', 'aoi_end_x', 'aoi_start_y', 'aoi_end_y', 'aoi_start_z', 'aoi_end_z', 
        'current_draw', 'percentage', 'mobility_ampere', 'hardware_ampere', 'computing_ampere', 'state'
    ]

    # Convert the relevant columns to numeric types
    numeric_cols = ['drone_id', 'x', 'y', 'z', 'battery_usage', 'aoi_start_x', 'aoi_end_x', 
                    'aoi_start_y', 'aoi_end_y', 'aoi_start_z', 'aoi_end_z', 'current_draw',
                    'percentage', 'mobility_ampere', 'hardware_ampere', 'computing_ampere', 'state']
    df_split[numeric_cols] = df_split[numeric_cols].apply(pd.to_numeric, errors='coerce')
    
    # Convert time from nanoseconds (e.g., +4.9e+11ns) to seconds
    df_split['time'] = pd.to_numeric(df_split['time'].str.replace('ns', '').astype(float) * 1e-9, errors='coerce')

    return df_split

def plot_percentage_over_time(df):
    # Ensure the data is sorted by time for each drone
    df = df.sort_values(by=['drone_id', 'time']).reset_index(drop=True)

    plt.figure(figsize=(12, 7))
    
    # Use a distinct color palette
    unique_drones = df['drone_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(unique_drones)))
    
    # Loop through each unique drone ID and plot its percentage over time
    for i, drone_id in enumerate(unique_drones):
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        plt.plot(drone_data['time'], drone_data['percentage'], label=f'Drone {drone_id}', color=colors[i], alpha=0.8)
    
    # Add labels, legend, and title
    plt.xlabel('Time (s)')
    plt.ylabel('Battery Percentage (%)')
    plt.legend(title='Drone ID', frameon=True, fancybox=True, framealpha=0.9)
    plt.title('Battery Percentage Over Time')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()

    plt.savefig('plots/percentage_over_time.pdf')
    plt.show()

def interpolate_battery_usage(df, num_points=500):
    # Interpolating the battery usage data over a regular time interval for each drone
    interpolated_data = []
    
    for drone_id in df['drone_id'].unique():
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        
        if len(drone_data) < 2:
             interpolated_data.append(drone_data)
             continue

        time_new = np.linspace(drone_data['time'].min(), drone_data['time'].max(), num_points)
        interp_func = interp1d(drone_data['time'], drone_data['battery_usage'], kind='linear', fill_value='extrapolate')
        battery_usage_new = interp_func(time_new)

        # Maintain other columns as well
        interpolated_df = pd.DataFrame({
            'time': time_new,
            'battery_usage': battery_usage_new,
            'drone_id': drone_id,
            'x': np.interp(time_new, drone_data['time'], drone_data['x']),
            'y': np.interp(time_new, drone_data['time'], drone_data['y']),
            'z': np.interp(time_new, drone_data['time'], drone_data['z']),
            'current_draw': np.interp(time_new, drone_data['time'], drone_data['current_draw']),
            'aoi_start_x': np.interp(time_new, drone_data['time'], drone_data['aoi_start_x']),
            'aoi_end_x': np.interp(time_new, drone_data['time'], drone_data['aoi_end_x']),
            'aoi_start_y': np.interp(time_new, drone_data['time'], drone_data['aoi_start_y']),
            'aoi_end_y': np.interp(time_new, drone_data['time'], drone_data['aoi_end_y']),
            'aoi_start_z': np.interp(time_new, drone_data['time'], drone_data['aoi_start_z']),
            'aoi_end_z': np.interp(time_new, drone_data['time'], drone_data['aoi_end_z'])
        })

        interpolated_data.append(interpolated_df)
    
    df_interp = pd.concat(interpolated_data, ignore_index=True)
    
    return df_interp

def plot_battery_usage_phases(df):
    # Ensure the data is sorted by time for each drone
    df = df.sort_values(by=['drone_id', 'time']).reset_index(drop=True)

    # Remove any rows with NaN or infinite values in battery_usage or time
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=['battery_usage', 'time'])

    plt.figure(figsize=(12, 7))
    
    unique_drones = df['drone_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(unique_drones)))

    # Loop through each unique drone ID and plot its data
    for i, drone_id in enumerate(unique_drones):
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        
        # Plot battery usage over time for this drone with a unique color
        plt.plot(drone_data['time'], drone_data['battery_usage'], label=f'Drone {drone_id}', color=colors[i], alpha=0.8)
    
    # Adding labels, legend, and title
    plt.xlabel('Time (s)')
    plt.ylabel('Energy Consumed (Joules)')
    plt.title('Total Energy Consumption Over Time')
    plt.legend(title='Drone ID', frameon=True, fancybox=True, framealpha=0.9)
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()

    plt.savefig('plots/battery_usage_phases.pdf')
    plt.show()

def create_battery_usage_heatmap(df, grid_size=10):
    # Filter the DataFrame to include only data from the first drone (drone_id = 1)
    # Note: Assuming drone_id 1 exists. If not, pick the first available.
    target_id = 1 if 1 in df['drone_id'].unique() else df['drone_id'].unique()[0]
    df = df[df['drone_id'] == target_id].reset_index(drop=True)

    # Define the boundaries of the operational field
    x_min, x_max = df['x'].min(), df['x'].max()
    y_min, y_max = df['y'].min(), df['y'].max()

    # Create a 2D grid
    x_bins = np.arange(x_min, x_max, grid_size)
    y_bins = np.arange(y_min, y_max, grid_size)

    # Initialize a 2D array to store the battery usage sum for each cell
    heatmap = np.zeros((len(x_bins), len(y_bins)))

    # Loop through the drone's data
    for i in range(len(df) - 1):
        x1, x2 = df['x'][i], df['x'][i+1]
        y1, y2 = df['y'][i], df['y'][i+1]
        battery_usage = df['battery_usage'][i+1] - df['battery_usage'][i]
        
        # Identify the cells that the segment passes through
        x_indices = np.digitize([x1, x2], x_bins) - 1
        y_indices = np.digitize([y1, y2], y_bins) - 1

        # Sum the battery usage for the cells the segment passes through
        for xi in range(min(x_indices), max(x_indices) + 1):
            for yi in range(min(y_indices), max(y_indices) + 1):
                if 0 <= xi < heatmap.shape[0] and 0 <= yi < heatmap.shape[1]:
                    heatmap[xi, yi] += battery_usage / (abs(x2 - x1) + abs(y2 - y1) + 1e-9)  # Distribute usage across cells

    # Create the heatmap plot
    plt.figure(figsize=(10, 8))
    plt.imshow(heatmap.T, origin='lower', cmap='inferno', extent=[x_min, x_max, y_min, y_max], interpolation='gaussian')
    cbar = plt.colorbar(label='Energy Density (Joules/m)')
    plt.xlabel('X Coordinate [m]')
    plt.ylabel('Y Coordinate [m]')
    plt.title(f'Battery Usage Heatmap for Drone {target_id}')
    plt.grid(False) # Heatmaps usually look better without grids
    plt.tight_layout()
    plt.show()

def plot_drones_scenario(df):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Normalize ampere draw for color mapping (set from 0 to max ampere)
    norm = Normalize(vmin=0, vmax=df['current_draw'].max())  # Set minimum to 0
    cmap = cm.plasma  # Use a vibrant colormap

    unique_drones = df['drone_id'].unique()

    # Plot each drone's data
    for drone_id in unique_drones:
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)

        # We can plot segments, but for performance and look, let's try to plot continuous lines if possible.
        # However, to color by current draw, we need segments or a collection.
        # Let's stick to segments but optimize slightly.
        
        # Actually, let's just plot the path as a thin gray line first for continuity
        ax.plot(drone_data['x'], drone_data['y'], drone_data['z'], color='gray', alpha=0.3, linewidth=0.5)

        for i in range(len(drone_data) - 1):
            x1, x2 = drone_data['x'][i], drone_data['x'][i+1]
            y1, y2 = drone_data['y'][i], drone_data['y'][i+1]
            z1, z2 = drone_data['z'][i], drone_data['z'][i+1]
            
            # Determine the color based on the current draw (ampere) at this segment
            color = cmap(norm(drone_data['current_draw'][i]))

            # Plot the line with the color representing the current draw
            ax.plot([x1, x2], [y1, y2], [z1, z2], color=color, linewidth=2, alpha=0.8)

        # Markers for take-off and landing points
        ax.scatter([drone_data['x'].iloc[0]], [drone_data['y'].iloc[0]], [drone_data['z'].iloc[0]], 
                   color='green', marker='^', s=80, label='Takeoff' if drone_id == unique_drones[0] else "", edgecolors='black')
        ax.scatter([drone_data['x'].iloc[-1]], [drone_data['y'].iloc[-1]], [drone_data['z'].iloc[-1]], 
                   color='red', marker='v', s=80, label='Landing' if drone_id == unique_drones[0] else "", edgecolors='black')

    # Add a color bar to show the ampere draw mapping
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, pad=0.1, shrink=0.7)
    cbar.set_label('Current Draw (A)')

    # Axes labels
    ax.set_xlabel('X [m]', labelpad=30)
    ax.set_ylabel('Y [m]', labelpad=30)
    ax.set_zlabel('Z [m]', labelpad=30)
    ax.set_title('3D Drone Trajectories & Current Draw', pad=30)

    # Set limits with larger padding
    padding = 20
    ax.set_xlim(df['x'].min() - padding, df['x'].max() + padding)
    ax.set_ylim(df['y'].min() - padding, df['y'].max() + padding)
    ax.set_zlim(df['z'].min() - padding, df['z'].max() + padding)

    # Add a legend for takeoff and landing points
    ax.legend(loc='upper left')
    
    # Improve 3D view
    ax.view_init(elev=20, azim=-45)

    # Display the plot
    plt.tight_layout()
    plt.savefig('plots/drones_scenario_3d.pdf')
    plt.show()

def plot_ampere_draw(df):
    plt.figure(figsize=(12, 7))
    
    unique_drones = df['drone_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(unique_drones)))

    # Loop through each unique drone ID and plot its data
    for i, drone_id in enumerate(unique_drones):
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        
        # Plot current (ampere draw) over time for this drone with a unique color
        plt.plot(drone_data['time'], drone_data['current_draw'], label=f'Drone {drone_id}', color=colors[i], alpha=0.8)
    
    # Adding labels, legend, and title
    plt.xlabel('Time (s)')
    plt.ylabel('Current Draw (A)')
    plt.legend(title='Drone ID', frameon=True, fancybox=True, framealpha=0.9)
    plt.title('Instantaneous Current Draw Over Time')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()

    plt.savefig('plots/ampere_draw.pdf')
    plt.show()

def plot_ampere_breakdown_for_drone_01(df):
    # Define the states and their labels (0, 2, 3)
    states = [0, 2, 3]  # Ordered as required
    state_labels = ['Phase 1 (Takeoff/Transit)', 'IN AoI (Operation)', 'Phase 3 (Return/Land)']
    ampere_components = ['mobility_ampere', 'hardware_ampere', 'computing_ampere']
    component_labels = ['Mobility', 'Hardware', 'Computing']
    
    # Filter data for drone_id == 0
    df_drone0 = df[df['drone_id'] == 0]
    
    # Collect the last occurrence of each state
    state_entries = []
    valid_states = []
    valid_labels = []
    
    for idx, state in enumerate(states):
        if state in df_drone0['state'].values:
            entry = df_drone0[df_drone0['state'] == state].iloc[-1]  # Always pick the last occurrence
            state_entries.append(entry)
            valid_states.append(state)
            valid_labels.append(state_labels[idx])
    
    # Collect ampere data for plotting using column names
    state_data = [[entry['mobility_ampere'], entry['hardware_ampere'], entry['computing_ampere']] for entry in state_entries]
    
    # Convert to DataFrame for easier plotting, using custom labels as index
    state_df = pd.DataFrame(state_data, columns=component_labels, index=valid_labels)
    
    # Plot a bar graph
    ax = state_df.plot(kind='bar', stacked=True, figsize=(12, 8), colormap="viridis", width=0.6, edgecolor='black', alpha=0.9)
    
    # Add labels and title
    plt.xlabel("Drone Operational Phase", labelpad=15)
    plt.ylabel("Current Draw (A)", labelpad=15)
    plt.title("Ampere Contribution Breakdown for Drone ID 0", pad=20)
    plt.legend(title="Component", frameon=True, fancybox=True, framealpha=0.9)
    plt.xticks(rotation=0)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig('plots/ampere_breakdown_drone_0.pdf')
    plt.show()

if __name__ == "__main__":
    # Read data from the CSV file
    try:
        df_split = read_csv('results/results.csv')
        
        # Interpolate battery usage data for even distribution over time
        df_interp = interpolate_battery_usage(df_split)

        # Plot the battery usage over time, highlighting different phases
        plot_battery_usage_phases(df_interp)

        # Plot the drone scenario
        plot_drones_scenario(df_split)

        # Plot the current draw over time for each drone
        plot_ampere_draw(df_split)

        # Plot the percentage over time for each drone
        plot_percentage_over_time(df_split)

        # Plot the ampere breakdown by state
        plot_ampere_breakdown_for_drone_01(df_split)
        
    except FileNotFoundError:
        print("Error: 'results/results.csv' not found. Please run the simulation first.")
    except Exception as e:
        print(f"An error occurred: {e}")

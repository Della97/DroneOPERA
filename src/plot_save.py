import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
import pandas as pd
import numpy as np
from matplotlib import cm
from matplotlib.colors import Normalize
#from matplotlib.colors import TwoSlopeNorm
from scipy.interpolate import interp1d
from scipy.interpolate import make_interp_spline
import os

# Set global font sizes
plt.rcParams['font.size'] = 14          # Default font size for text
plt.rcParams['axes.labelsize'] = 16      # Font size for x, y, z axis labels
plt.rcParams['axes.titlesize'] = 16      # Font size for plot titles
plt.rcParams['xtick.labelsize'] = 12     # Font size for x-axis tick labels
plt.rcParams['ytick.labelsize'] = 12     # Font size for y-axis tick labels
plt.rcParams['legend.fontsize'] = 12     # Font size for legend text

# Create plots directory
os.makedirs('plots', exist_ok=True)

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

    plt.figure(figsize=(12, 6))
    
    # Set up color mapping for each drone_id
    colors = plt.cm.jet(np.linspace(0, 1, len(df['drone_id'].unique())))
    
    # Loop through each unique drone ID and plot its percentage over time
    for i, drone_id in enumerate(df['drone_id'].unique()):
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        plt.plot(drone_data['time'], drone_data['percentage'], label=f'Drone {drone_id}', color=colors[i])
    
    # Add labels, legend, and title
    plt.xlabel('Time (s)')
    plt.ylabel('Percentage')
    plt.legend(title='Drone ID')
    plt.title('Percentage Over Time for the Drone')

    plt.savefig('plots/percentage_over_time.png')
    plt.close()

def interpolate_battery_usage(df, num_points=500):
    # Interpolating the battery usage data over a regular time interval for each drone
    interpolated_data = []
    
    for drone_id in df['drone_id'].unique():
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        
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

    # Set up a color map to assign different colors to each drone_id
    colors = plt.cm.jet(np.linspace(0, 1, len(df['drone_id'].unique())))

    plt.figure(figsize=(12, 6))
    
    # Loop through each unique drone ID and plot its data
    for i, drone_id in enumerate(df['drone_id'].unique()):
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        
        # Plot battery usage over time for this drone with a unique color
        plt.plot(drone_data['time'], drone_data['battery_usage'], label=f'Drone {drone_id}', color=colors[i])
    
    # Adding labels, legend, and title
    plt.xlabel('Time (s)')
    plt.ylabel('Battery Usage (Joules)')
    plt.title('Battery Usage')

    plt.savefig('plots/battery_usage_phases.png')
    plt.close()

def plot_drones_scenario(df):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Normalize ampere draw for color mapping (set from 0 to max ampere)
    norm = Normalize(vmin=0, vmax=df['current_draw'].max())  # Set minimum to 0
    cmap = cm.viridis  # Use a colormap to represent the ampere draw

    # Plot each drone's data
    for drone_id in df['drone_id'].unique():
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)

        for i in range(len(drone_data) - 1):
            x1, x2 = drone_data['x'][i], drone_data['x'][i+1]
            y1, y2 = drone_data['y'][i], drone_data['y'][i+1]
            z1, z2 = drone_data['z'][i], drone_data['z'][i+1]
            
            # Determine the color based on the current draw (ampere) at this segment
            color = cmap(norm(drone_data['current_draw'][i]))

            # Plot the line with the color representing the current draw, set thinner line width
            ax.plot([x1, x2], [y1, y2], [z1, z2], color=color, linewidth=1)  # Set to 1 for a thinner line

        # Markers for take-off and landing points
        ax.scatter([drone_data['x'].iloc[0]], [drone_data['y'].iloc[0]], [drone_data['z'].iloc[0]], 
                   color='green', marker='^', s=100, label='Takeoff' if i == 0 else "")
        ax.scatter([drone_data['x'].iloc[-1]], [drone_data['y'].iloc[-1]], [drone_data['z'].iloc[-1]], 
                   color='red', marker='v', s=100, label='Landing' if i == 0 else "")

    # Add a color bar to show the ampere draw mapping
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Ampere Draw (A)')

    # Axes labels
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    # Set limits with larger padding
    padding = 20  # Increase padding as needed
    ax.set_xlim(df['x'].min() - padding, df['x'].max() + padding)
    ax.set_ylim(df['y'].min() - padding, df['y'].max() + padding)
    ax.set_zlim(df['z'].min() - padding, df['z'].max() + padding)

    # Add a legend for takeoff and landing points
    ax.legend()

    # Display the plot
    plt.savefig('plots/drones_scenario.png')
    plt.close()

def plot_ampere_draw(df):
    # Set up a color map to assign different colors to each drone_id
    colors = plt.cm.jet(np.linspace(0, 1, len(df['drone_id'].unique())))

    plt.figure(figsize=(12, 6))
    
    # Loop through each unique drone ID and plot its data
    for i, drone_id in enumerate(df['drone_id'].unique()):
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)
        
        # Plot current (ampere draw) over time for this drone with a unique color
        plt.plot(drone_data['time'], drone_data['current_draw'], label=f'Drone {drone_id}', color=colors[i])
    
    # Adding labels, legend, and title
    plt.xlabel('Time (s)')
    plt.ylabel('Current Draw (A)')
    plt.legend(title='Drone ID')
    plt.title('Current Draw Over Time for the Drone')

    plt.savefig('plots/ampere_draw.png')
    plt.close()

def plot_ampere_breakdown_for_drone_01(df):
    print(df.head())
    print(df.dtypes)

    # Define the states and their labels (0, 2, 3)
    states = [0, 2, 3]  # Ordered as required
    state_labels = ['Phase 1', 'IN AoI', 'State 3']
    ampere_components = ['mobility_ampere', 'hardware_ampere', 'computing_ampere']
    
    # Filter data for drone_id == 0
    df_drone0 = df[df['drone_id'] == 0]
    
    # Collect the last occurrence of each state
    state_entries = []
    for state in states:
        if state in df_drone0['state'].values:
            entry = df_drone0[df_drone0['state'] == state].iloc[-1]  # Always pick the last occurrence
            state_entries.append(entry)
    
    # Collect ampere data for plotting using column names
    state_data = [[entry['mobility_ampere'], entry['hardware_ampere'], entry['computing_ampere']] for entry in state_entries]
    
    # Convert to DataFrame for easier plotting, using custom labels as index
    state_df = pd.DataFrame(state_data, columns=ampere_components, index=state_labels)
    
    # Plot a bar graph
    state_df.plot(kind='bar', stacked=True, figsize=(10, 6), colormap="viridis")
    
    # Add labels and title
    print(state_df)
    for entry in state_entries:
        print(f"State {entry['state']}: mobility={entry['mobility_ampere']}, hardware={entry['hardware_ampere']}, computing={entry['computing_ampere']}")
    plt.xlabel("Drone State")
    plt.ylabel("Ampere (A)")
    plt.title("Ampere Contribution Breakdown for Drone ID 0 Across Selected States")
    plt.legend(title="Ampere Component")
    plt.savefig('plots/ampere_breakdown_drone_0.png')
    plt.close()


if __name__ == "__main__":
    # Read data from the CSV file
    df_split = read_csv('results/results.csv')
    print(df_split.shape)  # Check how many rows and columns are loaded
    print(df_split.head(10))  # Print first 10 rows to verify

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

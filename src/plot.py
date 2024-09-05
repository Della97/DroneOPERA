import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
import pandas as pd
import numpy as np
from matplotlib import cm
from matplotlib.colors import Normalize
from scipy.interpolate import interp1d

def read_csv(filename):
    # Read the CSV file and split the data manually by spaces
    df = pd.read_csv(filename, header=None)
    df_split = df[0].str.split(expand=True)

    # Rename the columns for easier access
    df_split.columns = [
        'drone_id', 'x', 'y', 'z', 'battery_usage', 'time',
        'aoi_start_x', 'aoi_end_x', 'aoi_start_y', 'aoi_end_y', 'aoi_start_z', 'aoi_end_z', 'current_draw'
    ]

    # Convert the relevant columns to numeric types
    df_split['drone_id'] = pd.to_numeric(df_split['drone_id'], errors='coerce')
    df_split['x'] = pd.to_numeric(df_split['x'], errors='coerce')
    df_split['y'] = pd.to_numeric(df_split['y'], errors='coerce')
    df_split['z'] = pd.to_numeric(df_split['z'], errors='coerce')
    df_split['battery_usage'] = pd.to_numeric(df_split['battery_usage'], errors='coerce')
    df_split['aoi_start_x'] = pd.to_numeric(df_split['aoi_start_x'], errors='coerce')
    df_split['aoi_end_x'] = pd.to_numeric(df_split['aoi_end_x'], errors='coerce')
    df_split['aoi_start_y'] = pd.to_numeric(df_split['aoi_start_y'], errors='coerce')
    df_split['aoi_end_y'] = pd.to_numeric(df_split['aoi_end_y'], errors='coerce')
    df_split['aoi_start_z'] = pd.to_numeric(df_split['aoi_start_z'], errors='coerce')
    df_split['aoi_end_z'] = pd.to_numeric(df_split['aoi_end_z'], errors='coerce')
    df_split['current_draw'] = pd.to_numeric(df_split['current_draw'], errors='coerce')
    
    # Convert time from nanoseconds (e.g., +4.9e+11ns) to seconds
    df_split['time'] = pd.to_numeric(df_split['time'].str.replace('ns', '').astype(float) * 1e-9, errors='coerce')

    return df_split

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

    plt.show()



def create_battery_usage_heatmap(df, grid_size=10):
    # Filter the DataFrame to include only data from the first drone (drone_id = 1)
    df = df[df['drone_id'] == 1].reset_index(drop=True)

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
                heatmap[xi, yi] += battery_usage / (abs(x2 - x1) + abs(y2 - y1))  # Distribute usage across cells

    # Create the heatmap plot
    plt.figure(figsize=(10, 8))
    plt.imshow(heatmap.T, origin='lower', cmap='hot', extent=[x_min, x_max, y_min, y_max])
    plt.colorbar(label='Total Battery Usage (Joules)')
    plt.xlabel('X Coordinate [m]')
    plt.ylabel('Y Coordinate [m]')
    plt.title('Battery Usage Heatmap for Drone 1')
    plt.show()

def plot_drones_scenario(df):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Normalize ampere draw for color mapping
    norm = Normalize(vmin=df['current_draw'].min(), vmax=df['current_draw'].max())
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

            # Plot the line with the color representing the current draw
            ax.plot([x1, x2], [y1, y2], [z1, z2], color=color, linewidth=2)

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

    # Set limits with padding
    ax.set_xlim(df['x'].min() - 10, df['x'].max() + 10)
    ax.set_ylim(df['y'].min() - 10, df['y'].max() + 10)
    ax.set_zlim(df['z'].min() - 10, df['z'].max() + 10)

    # Add a legend for takeoff and landing points
    ax.legend()

    # Display the plot
    plt.show()


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

            # Plot the line with the color representing the current draw
            ax.plot([x1, x2], [y1, y2], [z1, z2], color=color, linewidth=2)

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

    # Set limits with padding
    ax.set_xlim(df['x'].min() - 10, df['x'].max() + 10)
    ax.set_ylim(df['y'].min() - 10, df['y'].max() + 10)
    ax.set_zlim(df['z'].min() - 10, df['z'].max() + 10)

    # Add a legend for takeoff and landing points
    ax.legend()

    # Display the plot
    plt.show()





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
    plt.title('Current Draw Over Time for All Drones')

    plt.show()


if __name__ == "__main__":
    # Read data from the CSV file
    df_split = read_csv('results/results.csv')

    # Interpolate battery usage data for even distribution over time
    df_interp = interpolate_battery_usage(df_split)

    # Plot the battery usage over time, highlighting different phases
    plot_battery_usage_phases(df_interp)

    # Plot the drone scenario with highlighted AoI cubes
    plot_drones_scenario(df_split)

    # Plot the current draw over time for each drone
    plot_ampere_draw(df_split)

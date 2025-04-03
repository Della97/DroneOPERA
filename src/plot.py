import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
import pandas as pd
import numpy as np
from matplotlib import cm
from matplotlib.colors import Normalize
#from matplotlib.colors import TwoSlopeNorm
from scipy.interpolate import interp1d
from scipy.interpolate import make_interp_spline

# Set global font sizes
plt.rcParams['font.size'] = 14          # Default font size for text
plt.rcParams['axes.labelsize'] = 16      # Font size for x, y, z axis labels
plt.rcParams['axes.titlesize'] = 16      # Font size for plot titles
plt.rcParams['xtick.labelsize'] = 12     # Font size for x-axis tick labels
plt.rcParams['ytick.labelsize'] = 12     # Font size for y-axis tick labels
plt.rcParams['legend.fontsize'] = 12     # Font size for legend text

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

    plt.show()

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

def plot_drones_scenario1(df):
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

    # Use the 'coolwarm' colormap, where we can ensure blue-to-red behavior
    cmap = cm.coolwarm
    # Define a TwoSlopeNorm to start blue at 0, transition normally, and force red near 12
    norm = TwoSlopeNorm(vmin=0, vcenter=10, vmax=12)

    # Plot each drone's path
    for drone_id in df['drone_id'].unique():
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)

        for i in range(len(drone_data) - 1):
            x1, x2 = drone_data['x'][i], drone_data['x'][i+1]
            y1, y2 = drone_data['y'][i], drone_data['y'][i+1]
            z1, z2 = drone_data['z'][i], drone_data['z'][i+1]
            
            # Get color based on current draw with custom normalization
            color = cmap(norm(drone_data['current_draw'][i]))

            # Plot the segment with color based on current draw
            ax.plot([x1, x2], [y1, y2], [z1, z2], color=color, linewidth=2)

        # Mark the takeoff and landing points
        ax.scatter([drone_data['x'].iloc[0]], [drone_data['y'].iloc[0]], [drone_data['z'].iloc[0]], 
                   color='blue', marker='^', s=100, label='Takeoff' if i == 0 else "")
        ax.scatter([drone_data['x'].iloc[-1]], [drone_data['y'].iloc[-1]], [drone_data['z'].iloc[-1]], 
                   color='red', marker='v', s=100, label='Landing' if i == 0 else "")

    # Add a color bar with the custom colormap and normalization
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Ampere Draw (A)')

    # Axes labels and limits
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    
    padding = 20
    ax.set_xlim(df['x'].min() - padding, df['x'].max() + padding)
    ax.set_ylim(df['y'].min() - padding, df['y'].max() + padding)
    ax.set_zlim(df['z'].min() - padding, df['z'].max() + padding)

    # Display the legend for takeoff and landing points
    ax.legend()

    # Show the plot
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
    plt.title('Current Draw Over Time for the Drone')

    plt.show()

def plot_ampere_components_over_time(df):
    # Loop through each unique drone ID
    for drone_id in df['drone_id'].unique():
        # Filter the data for the current drone
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)

        # Plot each ampere component over time for this drone
        plt.figure(figsize=(12, 6))
        plt.plot(drone_data['time'], drone_data['mobility_ampere'], label='Mobility Ampere', color='blue')
        plt.plot(drone_data['time'], drone_data['hardware_ampere'], label='Hardware Ampere', color='green')
        plt.plot(drone_data['time'], drone_data['computing_ampere'], label='Computing Ampere', color='red')
        
        # Adding labels, legend, and title
        plt.xlabel('Time (s)')
        plt.ylabel('Ampere (A)')
        plt.title(f'Ampere Components Over Time for Drone {drone_id}')
        plt.legend()
        plt.show()

def plot_ampere_components_with_total(df):
    # Loop through each unique drone ID
    for drone_id in df['drone_id'].unique():
        # Filter the data for the current drone
        drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)

        # Calculate the total ampere as the sum of the components
        drone_data['total_ampere'] = (
            drone_data['mobility_ampere'] + 
            drone_data['hardware_ampere'] + 
            drone_data['computing_ampere']
        )

        # Set up the plot
        plt.figure(figsize=(12, 6))

        # Plot the total ampere as a line
        plt.plot(drone_data['time'], drone_data['total_ampere'], label='Total Ampere', color='black', linewidth=2)

        # Stack the individual components as an area plot
        plt.fill_between(drone_data['time'], 0, drone_data['mobility_ampere'], color='blue', alpha=0.6, label='Mobility Ampere')
        plt.fill_between(drone_data['time'], drone_data['mobility_ampere'], 
                         drone_data['mobility_ampere'] + drone_data['hardware_ampere'], color='green', alpha=0.6, label='Hardware Ampere')
        plt.fill_between(drone_data['time'], 
                         drone_data['mobility_ampere'] + drone_data['hardware_ampere'], 
                         drone_data['total_ampere'], color='red', alpha=0.6, label='Computing Ampere')

        # Adding labels, legend, and title
        plt.xlabel('Time (s)')
        plt.ylabel('Ampere (A)')
        plt.title(f'Total Ampere and Components Over Time for Drone {drone_id}')
        plt.legend()
        plt.show()

def plot_drone_id_1(df):
    # Filter the data to only include rows where drone_id is 1
    df = df[df['drone_id'] == 1].reset_index(drop=True)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Normalize ampere draw for color mapping (set from 0 to max ampere in filtered data)
    norm = Normalize(vmin=0, vmax=df['current_draw'].max())
    cmap = cm.viridis  # Use a colormap to represent the ampere draw

    # Plot the drone's data for drone_id 1
    for i in range(len(df) - 1):
        x1, x2 = df['x'][i], df['x'][i+1]
        y1, y2 = df['y'][i], df['y'][i+1]
        z1, z2 = df['z'][i], df['z'][i+1]
        
        # Determine the color based on the current draw (ampere) at this segment
        color = cmap(norm(df['current_draw'][i]))

        # Plot the line with the color representing the current draw, set thinner line width
        ax.plot([x1, x2], [y1, y2], [z1, z2], color=color, linewidth=1)

    # Markers for take-off and landing points
    ax.scatter([df['x'].iloc[0]], [df['y'].iloc[0]], [df['z'].iloc[0]], 
               color='green', marker='^', s=100, label='Takeoff')
    ax.scatter([df['x'].iloc[-1]], [df['y'].iloc[-1]], [df['z'].iloc[-1]], 
               color='red', marker='v', s=100, label='Landing')

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
    padding = 20  # Increase padding as needed
    ax.set_xlim(df['x'].min() - padding, df['x'].max() + padding)
    ax.set_ylim(df['y'].min() - padding, df['y'].max() + padding)
    ax.set_zlim(df['z'].min() - padding, df['z'].max() + padding)

    # Add a legend for takeoff and landing points
    ax.legend()

    # Display the plot
    plt.show()

def plot_ampere_breakdown_for_all_states(df):
    # Define the states and their labels, excluding state 3
    states = [0, 2, 1]  # Ordered to show state 1 last as per requirement
    state_labels = ['Phase 1', 'IN AoI', 'OUT AoI']
    ampere_components = ['mobility_ampere', 'hardware_ampere', 'computing_ampere']
    
    # Find a drone_id that has entries for the required states (0, 2, and 1)
    drone_ids_with_required_states = df.groupby('drone_id')['state'].nunique()
    selected_drone_id = drone_ids_with_required_states[drone_ids_with_required_states >= 3].index[0]
    
    # Collect the first occurrence of states 0 and 2 for this drone_id
    state_entries = []
    for state in [0, 2]:
        entry = df[(df['drone_id'] == selected_drone_id) & (df['state'] == state)].iloc[0]
        state_entries.append(entry)

    # Find the last occurrence of state 1 for this drone_id (Outside AoI)
    outside_aoi_entry = df[(df['drone_id'] == selected_drone_id) & (df['state'] == 1)].iloc[-1]
    state_entries.append(outside_aoi_entry)  # Add this entry as the last in the list

    # Collect ampere data for plotting
    state_data = [[entry['mobility_ampere'], entry['hardware_ampere'], entry['computing_ampere']] for entry in state_entries]

    # Convert to DataFrame for easier plotting, using custom labels as index
    state_df = pd.DataFrame(state_data, columns=ampere_components, index=state_labels)
    
    # Plot a bar graph
    state_df.plot(kind='bar', stacked=True, figsize=(10, 6), colormap="viridis")
    
    # Add labels and title
    plt.xlabel("Drone State")
    plt.ylabel("Ampere (A)")
    plt.title(f"Ampere Contribution Breakdown for Drone ID {selected_drone_id} Across Selected States")
    plt.legend(title="Ampere Component")
    plt.show()

def plot_ampere_breakdown_for_drone_0(df):
    # Define the states and their labels, excluding state 3
    states = [0, 2, 1]  # Ordered to show state 1 last as per requirement
    state_labels = ['Phase 1', 'IN AoI', 'OUT AoI']
    ampere_components = ['mobility_ampere', 'hardware_ampere', 'computing_ampere']
    
    # Filter data for drone_id == 0
    df = df[df['drone_id'] == 0]
    
    # Collect the first occurrence of states 0 and 2
    state_entries = []
    for state in [0, 2]:
        entry = df[df['state'] == state].iloc[0]
        state_entries.append(entry)
    
    # Find the last occurrence of state 1 (Outside AoI)
    outside_aoi_entry = df[df['state'] == 1].iloc[-1]
    state_entries.append(outside_aoi_entry)  # Add this entry as the last in the list

    # Collect ampere data for plotting
    state_data = [[entry['mobility_ampere'], entry['hardware_ampere'], entry['computing_ampere']] for entry in state_entries]
    
    # Convert to DataFrame for easier plotting, using custom labels as index
    state_df = pd.DataFrame(state_data, columns=ampere_components, index=state_labels)
    
    # Plot a bar graph
    state_df.plot(kind='bar', stacked=True, figsize=(10, 6), colormap="viridis")
    
    # Add labels and title
    plt.xlabel("Drone State")
    plt.ylabel("Ampere (A)")
    plt.title("Ampere Contribution Breakdown for Drone ID 0 Across Selected States")
    plt.legend(title="Ampere Component")
    plt.show()

def plot_single_drone_trajectory(df, drone_id):
    # Filter data for the specified drone ID
    drone_data = df[df['drone_id'] == drone_id].reset_index(drop=True)

    if drone_data.empty:
        print(f"No data found for drone ID {drone_id}.")
        return

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the drone's trajectory
    for i in range(len(drone_data) - 1):
        x1, x2 = drone_data['x'][i], drone_data['x'][i+1]
        y1, y2 = drone_data['y'][i], drone_data['y'][i+1]
        z1, z2 = drone_data['z'][i], drone_data['z'][i+1]
        
        # Plot the line segment for the drone's movement
        ax.plot([x1, x2], [y1, y2], [z1, z2], color='blue', linewidth=1)

    # Markers for takeoff and landing points
    ax.scatter([drone_data['x'].iloc[0]], [drone_data['y'].iloc[0]], [drone_data['z'].iloc[0]], 
               color='green', marker='^', s=100, label='Takeoff')
    ax.scatter([drone_data['x'].iloc[-1]], [drone_data['y'].iloc[-1]], [drone_data['z'].iloc[-1]], 
               color='red', marker='v', s=100, label='Landing')

    # Axes labels
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    # Set limits with padding
    ax.set_xlim(drone_data['x'].min() - 10, drone_data['x'].max() + 10)
    ax.set_ylim(drone_data['y'].min() - 10, drone_data['y'].max() + 10)
    ax.set_zlim(drone_data['z'].min() - 10, drone_data['z'].max() + 10)

    # Add a legend for takeoff and landing points
    ax.legend()

    # Display the plot
    plt.title(f"3D Trajectory for Drone {drone_id}")
    plt.show()


def plot_ampere_breakdown_for_drone_01(df):
    print(df.head())
    print(df.dtypes)

    # Define the states and their labels (0, 2, 3)
    states = [0, 2, 3]  # Ordered as required
    state_labels = ['Phase 1', 'IN AoI', 'State 3']
    ampere_components = ['mobility_ampere', 'hardware_ampere', 'computing_ampere']
    
    # Filter data for drone_id == 0
    df = df[df.iloc[:, 0] == 0]
    
    # Collect the last occurrence of each state
    state_entries = []
    for state in states:
        if state in df.iloc[:, 13].values:
            entry = df[df.iloc[:, 13] == state].iloc[-1]  # Always pick the last occurrence
            state_entries.append(entry)
    
    # Collect ampere data for plotting
    #state_data = [[entry.iloc[10], entry.iloc[11], entry.iloc[12]] for entry in state_entries]
    state_data = [[entry.values[10], entry.values[11], entry.values[12]] for entry in state_entries]
    
    # Convert to DataFrame for easier plotting, using custom labels as index
    state_df = pd.DataFrame(state_data, columns=ampere_components, index=state_labels)
    
    # Plot a bar graph
    state_df.plot(kind='bar', stacked=True, figsize=(10, 6), colormap="viridis")
    
    # Add labels and title
    print(state_df)
    for entry in state_entries:
        print(entry.values)  # Check extracted values
    plt.xlabel("Drone State")
    plt.ylabel("Ampere (A)")
    plt.title("Ampere Contribution Breakdown for Drone ID 0 Across Selected States")
    plt.legend(title="Ampere Component")
    plt.show()


if __name__ == "__main__":
    # Read data from the CSV file
    #df_split = read_csv('results/results.csv')
    df_split = pd.read_csv('results/results.csv')
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

    #plot_ampere_components_over_time(df_split)

    #plot_ampere_components_with_total(df_split)

    #plot_drone_id_1(df_split)

    # Plot the ampere breakdown by state
    plot_ampere_breakdown_for_drone_01(df_split)

    #plot_single_drone_trajectory(df_split, drone_id=1)

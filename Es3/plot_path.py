import pandas as pd
import matplotlib.pyplot as plt
import math
import os

# Names of the file .csv saved in the listener node (path_tracker_listener.cpp)
CSV_FILE_FLOOR = "vc_to_floor_path_projected.csv"
CSV_FILE_WRC = "vc_to_cs_path_projected.csv"

def plot_global_path(df_floor):
    """
    plot of VC and CS w.r.t floor_frame
    """
    if df_floor.empty:
        print("Error, data are empty")
        return

    # Extraction of the position w.r.t floor
    final_vc_x = df_floor['x'].iloc[-1]
    final_vc_y = df_floor['y'].iloc[-1]
    cs_x = df_floor['cs_x'].iloc[0]
    cs_y = df_floor['cs_y'].iloc[0]

    # Computation of final distance
    distance = math.sqrt((final_vc_x - cs_x)**2 + (final_vc_y - cs_y)**2)
    
    # --- Global path plot ---
    plt.figure(figsize=(10, 8))
    plt.plot(df_floor['x'], df_floor['y'], 
             label='VC Path and CS w.r.t Floor frame', 
             color='blue', linewidth=1.5, alpha=0.7)
    plt.scatter(cs_x, cs_y, 
                color='green', s=200, marker='s', zorder=5, 
                label='CS Position (reference)')
    plt.scatter(final_vc_x, final_vc_y, 
                color='red', s=150, zorder=5, 
                label='VC Final Position')
    
    plt.plot([final_vc_x, cs_x], [final_vc_y, cs_y], 
             'k--', linewidth=1, alpha=0.6, 
             label=f'Final Distance: {distance:.3f} m')

    plt.title(f'1. path of vc and cs w.r.t floor_frame (tag36h11:0)')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend(loc='best')
    plt.axis('equal')
    plt.grid(True)


def plot_relative_path(df_wrc):
    """
    plot of VC w.r.t CS 
    """
    if df_wrc.empty:
        print("Error: data of csv2 are empty ")
        return

    # the origin here is the c.s reference frame itself
    final_vc_x_rel = df_wrc['x'].iloc[-1]
    final_vc_y_rel = df_wrc['y'].iloc[-1]
    
    # --- Relative path plot ---
    plt.figure(figsize=(10, 8))
    plt.plot(df_wrc['x'], df_wrc['y'], 
             label='VC Path w.r.t CS frame', 
             color='purple', linewidth=1.5, alpha=0.7)
    plt.scatter(0, 0, 
                color='green', s=200, marker='s', zorder=5, 
                label='CS Position (reference)')
    plt.scatter(final_vc_x_rel, final_vc_y_rel, 
                color='red', s=150, zorder=5, 
                label='VC Final Position')
    
    # Final distance computation
    distance_check = math.sqrt(final_vc_x_rel**2 + final_vc_y_rel**2)

    plt.plot([final_vc_x_rel, 0], [final_vc_y_rel, 0], 
             'k--', linewidth=1, alpha=0.6, 
             label=f'Final Distance: {distance_check:.3f} m')


    plt.title(f'2. path of vc w.r.t cs_frame (tag36h11:1)')
    plt.xlabel('distance X from cs (meters)')
    plt.ylabel('sitance Y from CS (meters)')
    plt.legend(loc='best')
    plt.axis('equal')
    plt.grid(True)


def main_plotter():
    """ Load the files and call the plotter functions """
    
    # Load of the global path (w.r.t floor frame)
    if not os.path.exists(CSV_FILE_FLOOR):
        print(f"Error: global file csv '{CSV_FILE_FLOOR}' not founded")
        return
    df_floor = pd.read_csv(CSV_FILE_FLOOR)    
    # Load of the relative path (w.r.t cs_frame)
    if not os.path.exists(CSV_FILE_WRC):
        print(f"Error: relative file csv '{CSV_FILE_WRC}' not founded")
        return
    df_wrc = pd.read_csv(CSV_FILE_WRC)

    # Plot of the csv1 of the global path
    plot_global_path(df_floor)
    # Plot of the csv1 of the relative path
    plot_relative_path(df_wrc)
    plt.show() # show all the plot


if __name__ == '__main__':
    main_plotter()
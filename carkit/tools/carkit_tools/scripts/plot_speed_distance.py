#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def plot_speed_distance_relationships():
    # Create distance array from 0.5 to 2.0 with step 0.05
    distances = np.arange(0, 2.05, 0.05)
    
    # Define alpha values
    alphas = [0.5, 1.0, 1.5, 2.0]
    
    # Create the plot
    plt.figure(figsize=(10, 6))
    
    # Plot each line with different alpha
    for alpha in alphas:
        speeds = distances * alpha
        plt.plot(distances, speeds, label=f'α = {alpha}', linewidth=2)
    
    # Customize the plot
    plt.title('Speed vs Distance for Different α Values', fontsize=14)
    plt.xlabel('Distance (m)', fontsize=12)
    plt.ylabel('Speed (m/s)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=10)
    
    # Add a horizontal line at y=0 and a vertical line at x=0
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
    
    # Set axis limits
    plt.xlim(0, 2.1)
    plt.ylim(0, 4.5)
    
    # Show the plot
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    plot_speed_distance_relationships() 

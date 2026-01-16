import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

def plot_single_run(csv_file):
    """Plot H from a single CSV file"""
    df = pd.read_csv(csv_file)
    
    plt.figure(figsize=(8, 6))
    plt.plot(df['time'], df['H'], 'b-', linewidth=2)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('H', fontsize=12)
    plt.title('Aggregate Distortion over Time', fontsize=14)
    plt.grid(True, alpha=0.3)
    
    # Save plot
    plot_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Saved plot: {plot_file}")
    plt.close()

def plot_comparison(csv_files, labels=None):
    """Compare multiple runs"""
    plt.figure(figsize=(10, 6))
    
    for i, csv_file in enumerate(csv_files):
        df = pd.read_csv(csv_file)
        label = labels[i] if labels else os.path.basename(csv_file)
        plt.plot(df['time'], df['H'], linewidth=2, label=label)
    
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('H', fontsize=12)
    plt.title('Comparison of Multiple Runs', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.savefig('voronoi_data/comparison.png', dpi=300, bbox_inches='tight')
    print("Saved comparison plot: voronoi_data/comparison.png")
    plt.show()

def plot_latest():
    """Plot the most recent CSV file"""
    csv_files = glob.glob('voronoi_data/H_history_*.csv')
    if not csv_files:
        print("No CSV files found in voronoi_data/")
        return
    
    latest = max(csv_files, key=os.path.getctime)
    print(f"Plotting: {latest}")
    
    df = pd.read_csv(latest)
    
    plt.figure(figsize=(8, 6))
    plt.plot(df['time'], df['H'], 'b-', linewidth=2, label='Self-triggered')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('H', fontsize=12)
    plt.title('Aggregate Distortion over Time', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Print stats
    print(f"\nStatistics:")
    print(f"  Initial H: {df['H'].iloc[0]:.4f}")
    print(f"  Final H: {df['H'].iloc[-1]:.4f}")
    print(f"  Reduction: {df['H'].iloc[0] - df['H'].iloc[-1]:.4f}")
    print(f"  Duration: {df['time'].iloc[-1]:.1f}s")
    print(f"  Samples: {len(df)}")
    
    plt.savefig('voronoi_data/latest_plot.png', dpi=300, bbox_inches='tight')
    print("\nSaved plot: voronoi_data/latest_plot.png")
    plt.show()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # Plot specific file(s)
        if sys.argv[1] == 'compare':
            files = sys.argv[2:]
            plot_comparison(files)
        else:
            for f in sys.argv[1:]:
                plot_single_run(f)
    else:
        # Plot latest
        plot_latest()
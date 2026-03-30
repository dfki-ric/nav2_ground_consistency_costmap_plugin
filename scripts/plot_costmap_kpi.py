#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import sys
from datetime import datetime

# Read CSV file
csv_file = "/tmp/costmap_kpi_ground_consistency.csv"

try:
    df = pd.read_csv(csv_file)
except FileNotFoundError:
    print(f"Error: {csv_file} not found")
    sys.exit(1)

if df.empty or len(df) < 2:
    print("CSV file is empty or has insufficient data")
    sys.exit(1)

# Convert numeric columns from string to float/int
numeric_cols = ['total_cycle_ms', 'cells_updated', 'cells_decayed', 
                 'total_ground_cells', 'total_nonground_cells', 'memory_usage_mb',
                 'ground_points', 'nonground_points']
for col in numeric_cols:
    df[col] = pd.to_numeric(df[col], errors='coerce')

# Parse timestamp
df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce')

# Remove rows with NaT timestamps
df = df.dropna(subset=['timestamp']).reset_index(drop=True)

if df.empty:
    print("No valid data rows found")
    sys.exit(1)

# Compute time relative to start in seconds
t0 = df['timestamp'].iloc[0]
df['elapsed_s'] = (df['timestamp'] - t0).dt.total_seconds()

# Create figure with subplots
fig, axes = plt.subplots(3, 2, figsize=(15, 13))
fig.suptitle('Costmap Layer KPI Metrics Over Time', fontsize=16, fontweight='bold')

# Format x-axis helper
def format_xaxis(ax):
    ax.grid(True, alpha=0.3)

# Plot 1: Total Cycle Latency (filter out zero-latency cycles)
df_latency = df[df['total_cycle_ms'] > 0]
axes[0, 0].plot(df_latency['elapsed_s'], df_latency['total_cycle_ms'], 'b-', linewidth=1.0)
axes[0, 0].set_ylabel('Latency (ms)', fontweight='bold')
axes[0, 0].set_title('Total Cycle Time (updateBounds + updateCosts)')
format_xaxis(axes[0, 0])

# Plot 2: Cells Updated vs Decayed (filter out zero-decay cycles)
axes[0, 1].plot(df['elapsed_s'], df['cells_updated'], 'g-', label='Updated', linewidth=1.5)
df_decay = df[df['cells_decayed'] > 0]
axes[0, 1].plot(df_decay['elapsed_s'], df_decay['cells_decayed'], 'r-', label='Decayed', linewidth=1.5)
axes[0, 1].set_ylabel('Cell Count', fontweight='bold')
axes[0, 1].set_title('Cells Updated vs Decayed')
axes[0, 1].legend()
format_xaxis(axes[0, 1])

# Plot 3: Total Ground Cells
axes[1, 0].plot(df['elapsed_s'], df['total_ground_cells'], 'brown', linewidth=1.5)
axes[1, 0].set_ylabel('Cell Count', fontweight='bold')
axes[1, 0].set_title('Total Ground Score Map Size')
format_xaxis(axes[1, 0])

# Plot 4: Total Nonground Cells
axes[1, 1].plot(df['elapsed_s'], df['total_nonground_cells'], 'purple', linewidth=1.5)
axes[1, 1].set_ylabel('Cell Count', fontweight='bold')
axes[1, 1].set_title('Total Nonground Score Map Size')
format_xaxis(axes[1, 1])

# Plot 5: Memory Usage
axes[2, 0].plot(df['elapsed_s'], df['memory_usage_mb'], 'orange', linewidth=1.5)
axes[2, 0].set_ylabel('Memory (MB)', fontweight='bold')
axes[2, 0].set_xlabel('Time (s)', fontweight='bold')
axes[2, 0].set_title('Memory Usage')
format_xaxis(axes[2, 0])

# Plot 6: Points Processed
axes[2, 1].plot(df['elapsed_s'], df['ground_points'], 'brown', label='Ground', linewidth=1.5)
axes[2, 1].plot(df['elapsed_s'], df['nonground_points'], 'purple', label='Nonground', linewidth=1.5)
axes[2, 1].set_ylabel('Point Count', fontweight='bold')
axes[2, 1].set_xlabel('Time (s)', fontweight='bold')
axes[2, 1].set_title('Points Processed per Update')
axes[2, 1].legend()
format_xaxis(axes[2, 1])

plt.tight_layout(h_pad=3.0, rect=[0, 0.03, 1, 0.96])
plt.savefig('/tmp/costmap_kpi_analysis.png', dpi=150, bbox_inches='tight')
print("✓ Saved plot to: /tmp/costmap_kpi_analysis.png")

# Print statistics
print("\n" + "="*60)
print("KPI STATISTICS")
print("="*60)
print(f"Total updates: {len(df)}")
print(f"Duration: {(df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]).total_seconds():.1f}s")
print(f"\nTotal Cycle Time (ms):")
print(f"  Mean: {df['total_cycle_ms'].mean():.2f}")
print(f"  Min:  {df['total_cycle_ms'].min():.2f}")
print(f"  Max:  {df['total_cycle_ms'].max():.2f}")
print(f"  P95:  {df['total_cycle_ms'].quantile(0.95):.2f}")
print(f"\nCells Updated per cycle:")
print(f"  Mean: {df['cells_updated'].mean():.0f}")
print(f"  Max:  {df['cells_updated'].max():.0f}")
print(f"\nCells Decayed per cycle:")
print(f"  Mean: {df['cells_decayed'].mean():.0f}")
print(f"  Max:  {df['cells_decayed'].max():.0f}")
print(f"\nMemory Usage (MB):")
print(f"  Mean: {df['memory_usage_mb'].mean():.3f}")
print(f"  Max:  {df['memory_usage_mb'].max():.3f}")
print(f"\nGround Score Map Size:")
print(f"  Mean: {df['total_ground_cells'].mean():.0f}")
print(f"  Max:  {df['total_ground_cells'].max():.0f}")
print(f"\nNonground Score Map Size:")
print(f"  Mean: {df['total_nonground_cells'].mean():.0f}")
print(f"  Max:  {df['total_nonground_cells'].max():.0f}")
print("="*60)

plt.show()

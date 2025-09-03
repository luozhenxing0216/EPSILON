#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SSC Map Algorithm Flow Visualization
展示SSC算法的完整工作流程和数据结构关系
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.patches import FancyBboxPatch
import seaborn as sns

plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

def create_algorithm_flow_diagram():
    """创建SSC算法流程图"""
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('SSC (Spatial-Temporal Corridor) Algorithm Architecture', 
                 fontsize=16, fontweight='bold')
    
    # 1. 数据结构概览
    ax1.set_title('1. Core Data Structures', fontweight='bold')
    ax1.set_xlim(0, 10)
    ax1.set_ylim(0, 10)
    
    # 3D Grid
    grid_box = FancyBboxPatch((1, 7), 8, 2, boxstyle="round,pad=0.1", 
                              facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax1.add_patch(grid_box)
    ax1.text(5, 8, '3D Grid Map\n[s, d, t] = [1000, 100, 81]\nResolution: [0.25m, 0.2m, 0.1s]', 
             ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Config
    config_box = FancyBboxPatch((1, 4.5), 3.5, 2, boxstyle="round,pad=0.1",
                                facecolor='lightgreen', edgecolor='green', linewidth=2)
    ax1.add_patch(config_box)
    ax1.text(2.75, 5.5, 'Config\n• Map Size\n• Kinematics\n• Inflation', 
             ha='center', va='center', fontsize=9)
    
    # Corridor
    corridor_box = FancyBboxPatch((5.5, 4.5), 3.5, 2, boxstyle="round,pad=0.1",
                                  facecolor='lightyellow', edgecolor='orange', linewidth=2)
    ax1.add_patch(corridor_box)
    ax1.text(7.25, 5.5, 'Driving Corridor\n• Cubes\n• Seeds\n• Validity', 
             ha='center', va='center', fontsize=9)
    
    # Trajectory
    traj_box = FancyBboxPatch((1, 2), 8, 1.5, boxstyle="round,pad=0.1",
                              facecolor='lightcoral', edgecolor='red', linewidth=2)
    ax1.add_patch(traj_box)
    ax1.text(5, 2.75, 'Input Trajectory\nFrenet States: [s, d, t] + [vs, vd] + [as, ad]', 
             ha='center', va='center', fontsize=10)
    
    # Arrows
    ax1.arrow(5, 6.8, 0, -0.5, head_width=0.2, head_length=0.1, fc='black', ec='black')
    ax1.arrow(2.75, 4.3, 0, -0.5, head_width=0.2, head_length=0.1, fc='black', ec='black')
    ax1.arrow(7.25, 4.3, 0, -0.5, head_width=0.2, head_length=0.1, fc='black', ec='black')
    
    ax1.set_aspect('equal')
    ax1.axis('off')
    
    # 2. 算法流程
    ax2.set_title('2. Algorithm Pipeline', fontweight='bold')
    ax2.set_xlim(0, 10)
    ax2.set_ylim(0, 10)
    
    steps = [
        ('Initialize\n3D Grid', 9, 'lightblue'),
        ('Fill Static\nObstacles', 7.5, 'lightcoral'),
        ('Fill Dynamic\nObstacles', 6, 'lightcoral'),
        ('Extract\nSeeds', 4.5, 'lightgreen'),
        ('Create Initial\nCubes', 3, 'lightyellow'),
        ('Inflate\nCubes', 1.5, 'lightpink')
    ]
    
    for i, (step, y, color) in enumerate(steps):
        box = FancyBboxPatch((2, y-0.4), 6, 0.8, boxstyle="round,pad=0.1",
                             facecolor=color, edgecolor='black', linewidth=1)
        ax2.add_patch(box)
        ax2.text(5, y, step, ha='center', va='center', fontsize=10, fontweight='bold')
        
        if i < len(steps) - 1:
            ax2.arrow(5, y-0.5, 0, -0.8, head_width=0.3, head_length=0.2, 
                     fc='darkblue', ec='darkblue', linewidth=2)
    
    ax2.set_aspect('equal')
    ax2.axis('off')
    
    # 3. 3D空间示意
    ax3.set_title('3. 3D Space-Time Grid', fontweight='bold')
    
    # 创建3D效果的2D图
    s_range = np.linspace(0, 10, 11)
    d_range = np.linspace(0, 5, 6)
    t_layers = [0, 2, 4]
    
    colors = ['lightblue', 'lightgreen', 'lightyellow']
    
    for i, (t, color) in enumerate(zip(t_layers, colors)):
        for s in s_range[::2]:
            for d in d_range[::2]:
                x_offset = i * 0.3
                y_offset = i * 0.3
                square = patches.Rectangle((s + x_offset, d + y_offset), 0.8, 0.8,
                                         facecolor=color, edgecolor='black', 
                                         linewidth=0.5, alpha=0.7)
                ax3.add_patch(square)
    
    # 添加轴标签
    ax3.text(5, -1, 's (longitudinal)', ha='center', fontsize=12, fontweight='bold')
    ax3.text(-1, 2.5, 'd (lateral)', va='center', rotation=90, fontsize=12, fontweight='bold')
    ax3.text(8, 8, 't (time)', ha='center', fontsize=12, fontweight='bold', 
             bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    
    # 障碍物示例
    obstacle = patches.Rectangle((3.5, 2), 1, 1, facecolor='red', alpha=0.8)
    ax3.add_patch(obstacle)
    ax3.text(4, 1.5, 'Obstacle', ha='center', fontsize=10, color='red', fontweight='bold')
    
    # 走廊示例
    corridor = patches.Rectangle((6, 1.5), 2, 2, facecolor='none', 
                               edgecolor='blue', linewidth=3)
    ax3.add_patch(corridor)
    ax3.text(7, 0.8, 'Corridor\nCube', ha='center', fontsize=10, 
             color='blue', fontweight='bold')
    
    ax3.set_xlim(-1.5, 12)
    ax3.set_ylim(-1.5, 9)
    ax3.set_aspect('equal')
    ax3.axis('off')
    
    # 4. 立方体膨胀过程
    ax4.set_title('4. Cube Inflation Process', fontweight='bold')
    
    # 初始种子
    seed1 = plt.Circle((2, 5), 0.2, color='red', zorder=5)
    seed2 = plt.Circle((6, 3), 0.2, color='red', zorder=5)
    ax4.add_patch(seed1)
    ax4.add_patch(seed2)
    
    # 初始立方体
    initial_cube = patches.Rectangle((1.5, 2.5), 5, 3, facecolor='none',
                                   edgecolor='orange', linewidth=2, linestyle='--')
    ax4.add_patch(initial_cube)
    
    # 膨胀后立方体
    inflated_cube = patches.Rectangle((0.5, 1.5), 7, 5, facecolor='lightblue',
                                    edgecolor='blue', linewidth=3, alpha=0.3)
    ax4.add_patch(inflated_cube)
    
    # 膨胀方向箭头
    directions = [
        (4, 6.8, 0, 0.7, '+Z (time)'),
        (8, 4, 0.7, 0, '+X (s)'),
        (0.2, 4, -0.7, 0, '-X (s)'),
        (4, 1.2, 0, -0.7, '-Y (d)'),
        (4, 7.2, 0, 0.7, '+Y (d)')
    ]
    
    for x, y, dx, dy, label in directions:
        ax4.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, 
                 fc='green', ec='green', linewidth=2)
        if dx > 0:
            ax4.text(x + dx + 0.3, y, label, fontsize=8, ha='left')
        elif dx < 0:
            ax4.text(x + dx - 0.3, y, label, fontsize=8, ha='right')
        elif dy > 0:
            ax4.text(x, y + dy + 0.3, label, fontsize=8, ha='center')
        else:
            ax4.text(x, y + dy - 0.3, label, fontsize=8, ha='center')
    
    # 图例
    ax4.plot([], [], 'ro', markersize=8, label='Seeds')
    ax4.plot([], [], color='orange', linewidth=2, linestyle='--', label='Initial Cube')
    ax4.plot([], [], color='blue', linewidth=3, alpha=0.5, label='Inflated Cube')
    ax4.legend(loc='upper right')
    
    ax4.set_xlim(-2, 10)
    ax4.set_ylim(0, 8)
    ax4.set_aspect('equal')
    ax4.axis('off')
    
    plt.tight_layout()
    plt.savefig('/home/lzx/Learn/EPSILON_Program/src/EPSILON/test/util/ssc_planner/ssc_algorithm_flow.png', 
                dpi=300, bbox_inches='tight')
    plt.show()
    
    print("✅ SSC Algorithm Flow Diagram created!")

def create_data_flow_diagram():
    """创建数据流图"""
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    fig.suptitle('SSC Algorithm Data Flow & Function Call Hierarchy', 
                 fontsize=16, fontweight='bold')
    
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 12)
    
    # 输入数据
    inputs = [
        ('Vehicle Trajectory\nvec_E<FsVehicle>', 2, 11, 'lightblue'),
        ('Static Obstacles\nvec_E<Vec2f>', 6, 11, 'lightcoral'),
        ('Config Parameters', 10, 11, 'lightgreen')
    ]
    
    for text, x, y, color in inputs:
        box = FancyBboxPatch((x-1, y-0.5), 2, 1, boxstyle="round,pad=0.1",
                             facecolor=color, edgecolor='black', linewidth=1)
        ax.add_patch(box)
        ax.text(x, y, text, ha='center', va='center', fontsize=9, fontweight='bold')
    
    # 主要函数
    functions = [
        ('ConstructSscMap', 7, 9.5, 'gold'),
        ('FillStaticPart', 3, 8, 'lightcoral'),
        ('FillDynamicPart', 11, 8, 'lightblue'),
        ('ConstructCorridorUsingInitialTrajectory', 7, 6.5, 'orange'),
        ('GetInitialCubeUsingSeed', 3, 5, 'lightyellow'),
        ('InflateCubeIn3dGrid', 7, 5, 'lightpink'),
        ('CheckIfCubeIsFree', 11, 5, 'lightsteelblue'),
        ('InflateCubeOnXPosAxis', 2, 3.5, 'lightgray'),
        ('InflateCubeOnYPosAxis', 5, 3.5, 'lightgray'),
        ('InflateCubeOnZPosAxis', 8, 3.5, 'lightgray'),
        ('CheckIfPlaneIsFreeOnXAxis', 11, 3.5, 'lightgray')
    ]
    
    for text, x, y, color in functions:
        width = len(text) * 0.1 + 1
        box = FancyBboxPatch((x-width/2, y-0.3), width, 0.6, boxstyle="round,pad=0.1",
                             facecolor=color, edgecolor='black', linewidth=1)
        ax.add_patch(box)
        ax.text(x, y, text, ha='center', va='center', fontsize=8, fontweight='bold')
    
    # 输出数据
    outputs = [
        ('3D Grid Map\np_3d_grid_', 3, 1.5, 'lightblue'),
        ('Driving Corridors\ndriving_corridor_vec_', 7, 1.5, 'lightyellow'),
        ('Final Metric Cubes\nfinal_corridor_vec_', 11, 1.5, 'lightgreen')
    ]
    
    for text, x, y, color in outputs:
        box = FancyBboxPatch((x-1.2, y-0.4), 2.4, 0.8, boxstyle="round,pad=0.1",
                             facecolor=color, edgecolor='black', linewidth=2)
        ax.add_patch(box)
        ax.text(x, y, text, ha='center', va='center', fontsize=9, fontweight='bold')
    
    # 连接线
    connections = [
        # 输入到主函数
        ((2, 10.5), (7, 10)),
        ((6, 10.5), (7, 10)),
        ((10, 10.5), (7, 10)),
        
        # 主函数内部调用
        ((7, 9), (3, 8.5)),
        ((7, 9), (11, 8.5)),
        ((7, 9), (7, 7)),
        
        # 走廊构建调用
        ((7, 6), (3, 5.5)),
        ((7, 6), (7, 5.5)),
        ((7, 6), (11, 5.5)),
        
        # 膨胀函数调用
        ((7, 4.5), (2, 4)),
        ((7, 4.5), (5, 4)),
        ((7, 4.5), (8, 4)),
        
        # 检查函数调用
        ((11, 4.5), (11, 4)),
        
        # 输出
        ((3, 7.5), (3, 2)),
        ((7, 4.5), (7, 2)),
        ((11, 4.5), (11, 2))
    ]
    
    for (x1, y1), (x2, y2) in connections:
        ax.arrow(x1, y1, x2-x1, y2-y1, head_width=0.1, head_length=0.1,
                fc='darkblue', ec='darkblue', linewidth=1.5, alpha=0.7)
    
    # 添加图例说明
    legend_elements = [
        ('Input Data', 'lightblue'),
        ('Core Functions', 'gold'), 
        ('Helper Functions', 'lightgray'),
        ('Output Data', 'lightgreen')
    ]
    
    for i, (label, color) in enumerate(legend_elements):
        box = FancyBboxPatch((0.5, 10-i*0.8), 1.5, 0.5, boxstyle="round,pad=0.1",
                             facecolor=color, edgecolor='black', linewidth=1)
        ax.add_patch(box)
        ax.text(1.25, 10.25-i*0.8, label, ha='center', va='center', fontsize=8)
    
    ax.set_aspect('equal')
    ax.axis('off')
    
    plt.tight_layout()
    plt.savefig('/home/lzx/Learn/EPSILON_Program/src/EPSILON/test/util/ssc_planner/ssc_data_flow.png', 
                dpi=300, bbox_inches='tight')
    plt.show()
    
    print("✅ SSC Data Flow Diagram created!")

def create_performance_analysis():
    """创建性能分析图表"""
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('SSC Algorithm Performance Analysis', fontsize=16, fontweight='bold')
    
    # 1. 时间复杂度分析
    ax1.set_title('Time Complexity Analysis', fontweight='bold')
    
    grid_sizes = [50, 100, 200, 500, 1000]
    grid_time = [size**3 * 0.001 for size in grid_sizes]  # O(n³) for grid operations
    inflation_time = [size**2 * 0.01 for size in grid_sizes]  # O(n²) for cube inflation
    total_time = [g + i for g, i in zip(grid_time, inflation_time)]
    
    ax1.plot(grid_sizes, grid_time, 'o-', label='Grid Operations O(n³)', linewidth=2)
    ax1.plot(grid_sizes, inflation_time, 's-', label='Cube Inflation O(n²)', linewidth=2)
    ax1.plot(grid_sizes, total_time, '^-', label='Total Time', linewidth=2)
    
    ax1.set_xlabel('Grid Size (per dimension)')
    ax1.set_ylabel('Time (seconds)')
    ax1.set_yscale('log')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. 内存使用分析
    ax2.set_title('Memory Usage Analysis', fontweight='bold')
    
    memory_3d = [size**3 * 8 / 1024**2 for size in grid_sizes]  # 8 bytes per grid cell, MB
    memory_corridors = [size * 10 / 1024 for size in grid_sizes]  # Linear growth, KB
    
    ax2.bar([s-10 for s in grid_sizes], memory_3d, width=20, 
           label='3D Grid Memory (MB)', alpha=0.7)
    ax2.bar([s+10 for s in grid_sizes], memory_corridors, width=20,
           label='Corridor Memory (KB)', alpha=0.7)
    
    ax2.set_xlabel('Grid Size (per dimension)')
    ax2.set_ylabel('Memory Usage')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. 膨胀步数效果
    ax3.set_title('Inflation Steps vs Corridor Quality', fontweight='bold')
    
    inflation_steps = [1, 5, 10, 20, 50]
    corridor_volume = [step * 100 for step in inflation_steps]  # 假设线性关系
    computation_time = [step * 0.1 for step in inflation_steps]
    
    ax3_twin = ax3.twinx()
    
    line1 = ax3.plot(inflation_steps, corridor_volume, 'bo-', linewidth=2, label='Corridor Volume')
    line2 = ax3_twin.plot(inflation_steps, computation_time, 'ro-', linewidth=2, label='Computation Time')
    
    ax3.set_xlabel('Inflation Steps')
    ax3.set_ylabel('Corridor Volume', color='blue')
    ax3_twin.set_ylabel('Computation Time (s)', color='red')
    
    # 合并图例
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc='center right')
    
    ax3.grid(True, alpha=0.3)
    
    # 4. 算法配置参数影响
    ax4.set_title('Configuration Parameter Impact', fontweight='bold')
    
    params = ['Grid\nResolution', 'Max Time\nGrids', 'Inflation\nSteps', 
              'Kinematic\nLimits', 'Safety\nMargin']
    impact_safety = [85, 70, 90, 95, 100]
    impact_efficiency = [60, 80, 40, 85, 70]
    impact_quality = [90, 75, 85, 80, 85]
    
    x = np.arange(len(params))
    width = 0.25
    
    ax4.bar(x - width, impact_safety, width, label='Safety Impact', alpha=0.8)
    ax4.bar(x, impact_efficiency, width, label='Efficiency Impact', alpha=0.8)
    ax4.bar(x + width, impact_quality, width, label='Quality Impact', alpha=0.8)
    
    ax4.set_xlabel('Configuration Parameters')
    ax4.set_ylabel('Impact Score (0-100)')
    ax4.set_xticks(x)
    ax4.set_xticklabels(params)
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/lzx/Learn/EPSILON_Program/src/EPSILON/test/util/ssc_planner/ssc_performance_analysis.png', 
                dpi=300, bbox_inches='tight')
    plt.show()
    
    print("✅ SSC Performance Analysis created!")

if __name__ == "__main__":
    print("Creating SSC Algorithm Visualization Suite...")
    
    # 创建算法流程图
    create_algorithm_flow_diagram()
    
    # 创建数据流图  
    create_data_flow_diagram()
    
    # 创建性能分析
    create_performance_analysis()
    
    print("\n🎯 All SSC visualization diagrams created successfully!")
    print("\nFiles generated:")
    print("- ssc_algorithm_flow.png: Algorithm pipeline and 3D space visualization")
    print("- ssc_data_flow.png: Function call hierarchy and data flow")
    print("- ssc_performance_analysis.png: Performance metrics and parameter impact")

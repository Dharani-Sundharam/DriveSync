#!/usr/bin/env python3
"""
System Flow Diagram Generator
============================

Generates visual flow diagrams for the robot control system architecture.
Use this to create diagrams for your PowerPoint presentation.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, ConnectionPatch
import numpy as np

def create_system_architecture_diagram():
    """Create the main system architecture diagram"""
    
    fig, ax = plt.subplots(1, 1, figsize=(16, 12))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    # Define colors
    colors = {
        'ui': '#3b82f6',        # Blue
        'control': '#10b981',   # Green  
        'algorithm': '#f59e0b', # Orange
        'hardware': '#8b5cf6',  # Purple
        'physical': '#ef4444'   # Red
    }
    
    # Layer 1: User Interface
    ui_box = FancyBboxPatch((0.5, 8.5), 9, 1.2, 
                           boxstyle="round,pad=0.1", 
                           facecolor=colors['ui'], alpha=0.3,
                           edgecolor=colors['ui'], linewidth=2)
    ax.add_patch(ui_box)
    ax.text(5, 9.1, 'USER INTERFACE LAYER', ha='center', va='center', 
            fontsize=14, fontweight='bold', color=colors['ui'])
    
    # UI Components
    ui_components = [
        ('Main GUI\n(Auto Resolution)', 1.5, 8.8),
        ('LIDAR Overlay\n(480x320)', 5, 8.8),
        ('Control Panel\n(Manual/Auto)', 8.5, 8.8)
    ]
    
    for comp, x, y in ui_components:
        comp_box = FancyBboxPatch((x-0.7, y-0.25), 1.4, 0.5,
                                 boxstyle="round,pad=0.05",
                                 facecolor='white', alpha=0.8,
                                 edgecolor=colors['ui'])
        ax.add_patch(comp_box)
        ax.text(x, y, comp, ha='center', va='center', fontsize=9)
    
    # Layer 2: Control & Coordination
    control_box = FancyBboxPatch((0.5, 6.8), 9, 1.2,
                                boxstyle="round,pad=0.1",
                                facecolor=colors['control'], alpha=0.3,
                                edgecolor=colors['control'], linewidth=2)
    ax.add_patch(control_box)
    ax.text(5, 7.4, 'CONTROL & COORDINATION LAYER', ha='center', va='center',
            fontsize=14, fontweight='bold', color=colors['control'])
    
    # Main Controller
    main_controller = FancyBboxPatch((3, 7.0), 4, 0.6,
                                    boxstyle="round,pad=0.05",
                                    facecolor='white', alpha=0.9,
                                    edgecolor=colors['control'], linewidth=2)
    ax.add_patch(main_controller)
    ax.text(5, 7.3, 'PathfindingRobotController\n(Main Orchestrator)', 
            ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Layer 3: Algorithm & Processing
    algo_box = FancyBboxPatch((0.5, 4.5), 9, 1.8,
                             boxstyle="round,pad=0.1",
                             facecolor=colors['algorithm'], alpha=0.3,
                             edgecolor=colors['algorithm'], linewidth=2)
    ax.add_patch(algo_box)
    ax.text(5, 6.0, 'ALGORITHM & PROCESSING LAYER', ha='center', va='center',
            fontsize=14, fontweight='bold', color=colors['algorithm'])
    
    # Algorithm components
    algo_components = [
        ('Map Environment\n• Road Network\n• Boundaries', 1.8, 5.5),
        ('Pathfinding\n• A* Algorithm\n• RRT Algorithm', 3.6, 5.5),
        ('Navigation\n• PID Control\n• Path Following', 5.4, 5.5),
        ('LIDAR Mapping\n• Scan Processing\n• Occupancy Grid', 7.2, 5.5),
        ('Collision Avoid\n• YOLO Detection\n• Safety States', 1.8, 4.8),
        ('Robot Kinematics\n• Differential Drive\n• Encoder Calc', 3.6, 4.8),
        ('Data Fusion\n• Sensor Data\n• State Fusion', 5.4, 4.8),
        ('Performance\n• Thread Pool\n• Optimization', 7.2, 4.8)
    ]
    
    for comp, x, y in algo_components:
        comp_box = FancyBboxPatch((x-0.7, y-0.25), 1.4, 0.5,
                                 boxstyle="round,pad=0.05",
                                 facecolor='white', alpha=0.8,
                                 edgecolor=colors['algorithm'])
        ax.add_patch(comp_box)
        ax.text(x, y, comp, ha='center', va='center', fontsize=8)
    
    # Layer 4: Hardware Abstraction
    hw_abs_box = FancyBboxPatch((0.5, 2.8), 9, 1.2,
                               boxstyle="round,pad=0.1",
                               facecolor=colors['hardware'], alpha=0.3,
                               edgecolor=colors['hardware'], linewidth=2)
    ax.add_patch(hw_abs_box)
    ax.text(5, 3.4, 'HARDWARE ABSTRACTION LAYER', ha='center', va='center',
            fontsize=14, fontweight='bold', color=colors['hardware'])
    
    hw_abs_components = [
        ('Robot Controller\n• Serial Comm\n• Encoder Data', 2, 3.2),
        ('LIDAR Interface\n• YDLidar SDK\n• Scan Data', 4, 3.2),
        ('Camera Interface\n• OpenCV\n• YOLO Model', 6, 3.2),
        ('Serial Comm\n• Thread Safe\n• Queue Based', 8, 3.2)
    ]
    
    for comp, x, y in hw_abs_components:
        comp_box = FancyBboxPatch((x-0.8, y-0.25), 1.6, 0.5,
                                 boxstyle="round,pad=0.05",
                                 facecolor='white', alpha=0.8,
                                 edgecolor=colors['hardware'])
        ax.add_patch(comp_box)
        ax.text(x, y, comp, ha='center', va='center', fontsize=8)
    
    # Layer 5: Hardware
    hw_box = FancyBboxPatch((0.5, 0.5), 9, 1.8,
                           boxstyle="round,pad=0.1",
                           facecolor=colors['physical'], alpha=0.3,
                           edgecolor=colors['physical'], linewidth=2)
    ax.add_patch(hw_box)
    ax.text(5, 2.0, 'HARDWARE LAYER', ha='center', va='center',
            fontsize=14, fontweight='bold', color=colors['physical'])
    
    hw_components = [
        ('Raspberry Pi 4\n• Main Computer\n• Linux OS', 2, 1.7),
        ('YDLIDAR X2\n• 360° Laser\n• 8m Range', 4, 1.7),
        ('USB Camera\n• Video Stream\n• Object Detect', 6, 1.7),
        ('Arduino Uno\n• Motor Driver\n• Encoder Read', 8, 1.7),
        ('Differential Drive\n• 210mm Length\n• 70mm Wheels', 2.5, 1.0),
        ('Encoders\n• 4993/4966\n• ticks/rev', 4.5, 1.0),
        ('L298N Driver\n• PWM Control\n• Direction', 6.5, 1.0),
        ('Power System\n• 12V Motors\n• 5V Electronics', 8, 1.0)
    ]
    
    for comp, x, y in hw_components:
        comp_box = FancyBboxPatch((x-0.7, y-0.25), 1.4, 0.5,
                                 boxstyle="round,pad=0.05",
                                 facecolor='white', alpha=0.8,
                                 edgecolor=colors['physical'])
        ax.add_patch(comp_box)
        ax.text(x, y, comp, ha='center', va='center', fontsize=8)
    
    # Add arrows between layers
    arrow_props = dict(arrowstyle='->', connectionstyle='arc3', lw=2, color='gray')
    
    # UI to Control
    ax.annotate('', xy=(5, 6.8), xytext=(5, 8.5), arrowprops=arrow_props)
    
    # Control to Algorithm
    ax.annotate('', xy=(5, 6.3), xytext=(5, 6.8), arrowprops=arrow_props)
    
    # Algorithm to Hardware Abstraction
    ax.annotate('', xy=(5, 4.0), xytext=(5, 4.5), arrowprops=arrow_props)
    
    # Hardware Abstraction to Hardware
    ax.annotate('', xy=(5, 2.8), xytext=(5, 2.3), arrowprops=arrow_props)
    
    plt.title('Autonomous Robot Navigation System - Architecture', 
              fontsize=18, fontweight='bold', pad=20)
    
    # Add legend
    legend_elements = [
        mpatches.Patch(color=colors['ui'], alpha=0.3, label='User Interface'),
        mpatches.Patch(color=colors['control'], alpha=0.3, label='Control & Coordination'),
        mpatches.Patch(color=colors['algorithm'], alpha=0.3, label='Algorithm & Processing'),
        mpatches.Patch(color=colors['hardware'], alpha=0.3, label='Hardware Abstraction'),
        mpatches.Patch(color=colors['physical'], alpha=0.3, label='Physical Hardware')
    ]
    ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))
    
    plt.tight_layout()
    plt.savefig('/home/dharani/Desktop/DriveSync/robot_control/docs/system_architecture.png', 
                dpi=300, bbox_inches='tight')
    plt.show()

def create_data_flow_diagram():
    """Create the data flow diagram"""
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # Input sources
    inputs = [
        ('User Input', 1, 7),
        ('LIDAR Sensor', 3, 7),
        ('Camera Sensor', 5, 7),
        ('Encoder Feedback', 7, 7)
    ]
    
    for name, x, y in inputs:
        input_box = FancyBboxPatch((x-0.6, y-0.3), 1.2, 0.6,
                                  boxstyle="round,pad=0.05",
                                  facecolor='lightblue', alpha=0.8)
        ax.add_patch(input_box)
        ax.text(x, y, name, ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Data fusion
    fusion_box = FancyBboxPatch((2, 5.2), 6, 1,
                               boxstyle="round,pad=0.1",
                               facecolor='lightgreen', alpha=0.8)
    ax.add_patch(fusion_box)
    ax.text(5, 5.7, 'SENSOR DATA FUSION & PROCESSING', ha='center', va='center',
            fontsize=12, fontweight='bold')
    
    # Processing components
    processing = [
        ('User Events\n& Commands', 2.5, 5.4),
        ('LIDAR Scans\n& Mapping', 3.8, 5.4),
        ('Object Detection\n& Safety', 5.2, 5.4),
        ('Position\n& Odometry', 6.5, 5.4)
    ]
    
    for comp, x, y in processing:
        ax.text(x, y, comp, ha='center', va='center', fontsize=9)
    
    # Decision making
    decision_box = FancyBboxPatch((1, 3.2), 8, 1,
                                 boxstyle="round,pad=0.1",
                                 facecolor='orange', alpha=0.8)
    ax.add_patch(decision_box)
    ax.text(5, 3.7, 'DECISION MAKING & CONTROL', ha='center', va='center',
            fontsize=12, fontweight='bold')
    
    decisions = [
        ('Path\nPlanning', 2, 3.4),
        ('Navigation\nControl', 3.5, 3.4),
        ('Safety\nAssessment', 5, 3.4),
        ('Mode\nManagement', 6.5, 3.4),
        ('Motor\nCommands', 8, 3.4)
    ]
    
    for comp, x, y in decisions:
        ax.text(x, y, comp, ha='center', va='center', fontsize=9)
    
    # Outputs
    outputs = [
        ('Motor Control', 2, 1.5),
        ('Display Updates', 4, 1.5),
        ('Status Indicators', 6, 1.5),
        ('Data Logging', 8, 1.5)
    ]
    
    for name, x, y in outputs:
        output_box = FancyBboxPatch((x-0.6, y-0.3), 1.2, 0.6,
                                   boxstyle="round,pad=0.05",
                                   facecolor='lightcoral', alpha=0.8)
        ax.add_patch(output_box)
        ax.text(x, y, name, ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Add arrows
    arrow_props = dict(arrowstyle='->', lw=2, color='darkblue')
    
    # Input to fusion
    for _, x, y in inputs:
        ax.annotate('', xy=(5, 6.2), xytext=(x, y-0.3), arrowprops=arrow_props)
    
    # Fusion to decision
    ax.annotate('', xy=(5, 4.2), xytext=(5, 5.2), arrowprops=arrow_props)
    
    # Decision to outputs
    for _, x, y in outputs:
        ax.annotate('', xy=(x, y+0.3), xytext=(5, 3.2), arrowprops=arrow_props)
    
    plt.title('System Data Flow Diagram', fontsize=16, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig('/home/dharani/Desktop/DriveSync/robot_control/docs/data_flow.png', 
                dpi=300, bbox_inches='tight')
    plt.show()

def create_control_loop_diagram():
    """Create the main control loop diagram"""
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # Control loop steps
    steps = [
        ('Sensor Data\nCollection', 2, 6.5, 'lightblue'),
        ('Data Processing\n& Analysis', 8, 6.5, 'lightgreen'),
        ('Decision Making\n& Planning', 8, 2, 'orange'),
        ('Control Execution\n& Output', 2, 2, 'lightcoral')
    ]
    
    for i, (name, x, y, color) in enumerate(steps):
        step_box = FancyBboxPatch((x-1, y-0.8), 2, 1.6,
                                 boxstyle="round,pad=0.1",
                                 facecolor=color, alpha=0.8,
                                 edgecolor='black', linewidth=2)
        ax.add_patch(step_box)
        ax.text(x, y+0.3, f'Step {i+1}', ha='center', va='center', 
                fontsize=12, fontweight='bold')
        ax.text(x, y-0.3, name, ha='center', va='center', fontsize=10)
    
    # Add detailed components for each step
    details = [
        (['• LIDAR Scanning', '• Camera Monitoring', '• Encoder Reading', '• User Input'], 2, 5.2),
        (['• LIDAR Mapping', '• Object Detection', '• Position Calc', '• Safety Check'], 8, 5.2),
        (['• Path Planning', '• Navigation Control', '• Safety Override', '• Mode Management'], 8, 3.3),
        (['• Motor Commands', '• Display Updates', '• Status Comm', '• Data Logging'], 2, 3.3)
    ]
    
    for detail_list, x, y in details:
        for i, detail in enumerate(detail_list):
            ax.text(x, y - i*0.2, detail, ha='center', va='center', fontsize=8)
    
    # Add arrows to show flow
    arrow_props = dict(arrowstyle='->', lw=3, color='darkblue')
    
    # Step 1 to Step 2
    ax.annotate('', xy=(7, 6.5), xytext=(3, 6.5), arrowprops=arrow_props)
    
    # Step 2 to Step 3
    ax.annotate('', xy=(8, 2.8), xytext=(8, 5.7), arrowprops=arrow_props)
    
    # Step 3 to Step 4
    ax.annotate('', xy=(3, 2), xytext=(7, 2), arrowprops=arrow_props)
    
    # Step 4 to Step 1 (feedback loop)
    ax.annotate('', xy=(2, 5.7), xytext=(2, 2.8), arrowprops=arrow_props)
    
    # Add timing annotation
    ax.text(5, 7.5, '20 Hz Control Loop (50ms cycle time)', ha='center', va='center',
            fontsize=14, fontweight='bold', color='red')
    
    plt.title('Main System Control Loop', fontsize=16, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig('/home/dharani/Desktop/DriveSync/robot_control/docs/control_loop.png', 
                dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    print("🎨 Generating system architecture diagrams...")
    
    try:
        print("📊 Creating system architecture diagram...")
        create_system_architecture_diagram()
        
        print("🔄 Creating data flow diagram...")
        create_data_flow_diagram()
        
        print("⚙️  Creating control loop diagram...")
        create_control_loop_diagram()
        
        print("✅ All diagrams generated successfully!")
        print("📁 Saved to: /home/dharani/Desktop/DriveSync/robot_control/docs/")
        
    except Exception as e:
        print(f"❌ Error generating diagrams: {e}")
        print("Make sure matplotlib is installed: pip install matplotlib")

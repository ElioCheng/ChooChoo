#!/usr/bin/env python3
"""
Calculate deceleration values for train kinematic model defaults.

This script reads the model_defaults.c file and calculates deceleration values
using the formula: deceleration = velocity / (stop_distance * 2 / velocity)

The data structure is: {velocity, acceleration, deceleration, stop_distance, stop_time, 0, 0, 0}
"""

import re
import sys
from typing import List, Tuple, Dict

# Constants from kinematics.h
KINEMATIC_VELOCITY_SCALE_FACTOR = 100000000  # 10^8
KINEMATIC_ACCEL_SCALE_FACTOR = 100000000     # 10^8
KINEMATIC_TOTAL_SPEED_LEVELS = 28

def parse_train_data(file_content: str) -> Dict[int, List[List[int]]]:
    """
    Parse the model_defaults.c file to extract train data.
    
    Returns:
        Dictionary mapping train_id to list of speed parameters
    """
    trains = {}
    
    # Find all train speed array declarations
    pattern = r'train_(\d+)_speeds\[KINEMATIC_TOTAL_SPEED_LEVELS\]\s*=\s*\{'
    matches = re.finditer(pattern, file_content, re.DOTALL)
    
    for match in matches:
        train_id = int(match.group(1))
        start_pos = match.end()
        
        # Find the closing brace and semicolon
        brace_count = 1
        pos = start_pos
        while pos < len(file_content) and brace_count > 0:
            if file_content[pos] == '{':
                brace_count += 1
            elif file_content[pos] == '}':
                brace_count -= 1
            pos += 1
        
        if brace_count == 0:
            # Extract the array content
            array_content = file_content[start_pos:pos-1]
            
            # Parse the array data
            rows = []
            
            # Find all {num1,num2,num3,num4,num5,num6,num7,num8} patterns
            row_pattern = r'\{([^}]+)\}'
            row_matches = re.findall(row_pattern, array_content)
            
            for row_match in row_matches:
                # Parse the comma-separated numbers
                numbers_str = row_match.strip()
                numbers = [int(x.strip()) for x in numbers_str.split(',') if x.strip()]
                if len(numbers) >= 5:  # We need at least velocity, accel, decel, stop_dist, stop_time
                    # Pad to 8 elements if needed
                    while len(numbers) < 8:
                        numbers.append(0)
                    rows.append(numbers[:8])  # Take only first 8 elements
            
            if len(rows) == KINEMATIC_TOTAL_SPEED_LEVELS:
                trains[train_id] = rows
            else:
                print(f"Warning: Train {train_id} has {len(rows)} rows, expected {KINEMATIC_TOTAL_SPEED_LEVELS}")
    
    return trains

def calculate_deceleration(velocity: int, stop_distance: int) -> int:
    """
    Calculate deceleration using the formula: deceleration = velocity / (stop_distance * 2 / velocity)
    
    This simplifies to: deceleration = velocity^2 / (stop_distance * 2)
    
    Args:
        velocity: Velocity in scaled units (mm/tick * SCALE_FACTOR)
        stop_distance: Stop distance in mm
        
    Returns:
        Deceleration in scaled units (mm/tick² * SCALE_FACTOR)
    """
    if stop_distance == 0 or velocity == 0:
        return 0
    
    # Calculate: deceleration = velocity^2 / (stop_distance * 2)
    # Since velocity is scaled by VELOCITY_SCALE_FACTOR, we need to handle scaling properly
    # To get 8 decimal precision like velocity, we need to scale the result appropriately
    
    # First, convert velocity to real units for calculation
    velocity_real = velocity / KINEMATIC_VELOCITY_SCALE_FACTOR
    
    # Calculate deceleration in real units
    deceleration_real = (velocity_real * velocity_real) / (stop_distance * 2)
    
    # Scale back to fixed-point representation with 8 decimal precision
    deceleration = int(deceleration_real * KINEMATIC_ACCEL_SCALE_FACTOR)
    
    return deceleration

def update_train_deceleration(train_data: List[List[int]]) -> List[List[int]]:
    """
    Update deceleration values for all speed levels in a train.
    
    Args:
        train_data: List of speed parameter rows
        
    Returns:
        Updated train data with new deceleration values
    """
    updated_data = []
    
    for i, row in enumerate(train_data):
        velocity = row[0]
        acceleration = row[1]
        deceleration = row[2]  # Current deceleration (will be updated)
        stop_distance = row[3]
        stop_time = row[4]
        
        # Calculate new deceleration
        new_deceleration = calculate_deceleration(velocity, stop_distance)
        
        # Create updated row
        updated_row = [velocity, acceleration, new_deceleration, stop_distance, stop_time] + row[5:]
        updated_data.append(updated_row)
        
        # Print calculation details for non-zero values
        if velocity > 0 and stop_distance > 0:
            velocity_mm_per_tick = velocity / KINEMATIC_VELOCITY_SCALE_FACTOR
            decel_mm_per_tick2 = new_deceleration / KINEMATIC_ACCEL_SCALE_FACTOR
            print(f"  Speed {i}: velocity={velocity_mm_per_tick:.8f} mm/tick, "
                  f"stop_dist={stop_distance} mm, decel={decel_mm_per_tick2:.8f} mm/tick²")
    
    return updated_data

def format_c_array(train_id: int, train_data: List[List[int]]) -> str:
    """
    Format train data as a C array declaration.
    
    Args:
        train_id: Train ID number
        train_data: List of speed parameter rows
        
    Returns:
        Formatted C array string
    """
    lines = [f"static kinematic_speed_params_t train_{train_id}_speeds[KINEMATIC_TOTAL_SPEED_LEVELS] = {{"]
    
    for row in train_data:
        # Format as {velocity,acceleration,deceleration,stop_distance,stop_time,0,0,0}
        formatted_row = "{" + ",".join(str(x) for x in row) + "}"
        lines.append(f"    {formatted_row},")
    
    lines.append("};")
    return "\n".join(lines)

def main():
    """Main function to process the model defaults file."""
    if len(sys.argv) != 2:
        print("Usage: python calculate_deceleration.py <model_defaults.c>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    try:
        with open(input_file, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found")
        sys.exit(1)
    
    print("Parsing train data from model_defaults.c...")
    trains = parse_train_data(content)
    
    if not trains:
        print("No train data found in the file")
        sys.exit(1)
    
    print(f"Found {len(trains)} trains: {sorted(trains.keys())}")
    
    # Process each train
    for train_id in sorted(trains.keys()):
        print(f"\n=== Train {train_id} ===")
        
        original_data = trains[train_id]
        updated_data = update_train_deceleration(original_data)
        
        # Generate updated C array
        c_array = format_c_array(train_id, updated_data)
        
        # Save to file
        output_file = f"train_{train_id}_updated.c"
        with open(output_file, 'w') as f:
            f.write(f"// Updated deceleration values for train {train_id}\n")
            f.write(f"// Generated by calculate_deceleration.py\n\n")
            f.write(c_array)
            f.write("\n")
        
        print(f"Updated data saved to {output_file}")
    
    print(f"\nProcessing complete! Updated files generated for {len(trains)} trains.")

if __name__ == "__main__":
    main() 
import re
import random
import sys

def jitter_value(val_str):
    try:
        val = float(val_str)
        # Add random jitter between -5m and 5m
        jitter = random.uniform(-1, 1)
        return f"{val + jitter:.4f}"
    except ValueError:
        return val_str

def process_file(file_path):
    print(f"Processing {file_path}...")
    # Read entire file
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # Pattern for static set: $node_(N) set [XYZ]_ VAL
    static_pattern = re.compile(r'(\$node_\(\d+\) set [XYZ]_ )([\d\.-]+)')
    # Pattern for dynamic setdest: ... setdest X Y SPEED"
    dest_pattern = re.compile(r'(.* setdest )([\d\.-]+) ([\d\.-]+)( .*)')
    
    new_lines = []
    modified_count = 0
    
    for line in lines:
        # Process static set X_/Y_/Z_
        match_static = static_pattern.match(line)
        if match_static:
            prefix = match_static.group(1)
            val = match_static.group(2)
            new_line = f"{prefix}{jitter_value(val)}\n"
            new_lines.append(new_line)
            modified_count += 1
            continue

        # Process setdest
        match_dest = dest_pattern.match(line)
        if match_dest:
            prefix = match_dest.group(1)
            x = match_dest.group(2)
            y = match_dest.group(3)
            suffix = match_dest.group(4)
            new_line = f"{prefix}{jitter_value(x)} {jitter_value(y)}{suffix}\n"
            new_lines.append(new_line)
            modified_count += 1
            continue

        new_lines.append(line)
        
    # Write back
    with open(file_path, 'w') as f:
        f.writelines(new_lines)
    print(f"Done. Modified {modified_count} lines.")

if __name__ == "__main__":
    process_file("/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/grid_network_fed/mobility.tcl")

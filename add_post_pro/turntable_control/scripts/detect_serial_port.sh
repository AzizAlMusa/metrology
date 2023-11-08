#!/bin/bash

# Use the 'ls' command to list available serial ports (adjust as needed)
available_ports=$(ls /dev/ttyUSB* 2>/dev/null)

# Check if any ports were found
if [ -n "$available_ports" ]; then
    # Select the first available port (you can modify this logic as needed)
    selected_port=$(echo "$available_ports" | head -n 1 | tr -d '\n')
    
    # Change the permissions of the selected port
    sudo chmod a+rw "$selected_port"
    
    echo -n "$selected_port"  # Use -n to prevent adding a newline character
else
    echo -n "No available serial ports found."  # Use -n to prevent adding a newline character
fi

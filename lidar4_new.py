import struct
import serial

import time
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import math

class LidarDataCollector:
    def __init__(self):
        self.angles = []        # Empty list to store direction measurements
        self.distances = []     # Empty list to store distance measurements
        self.timestamps = []
        self.execution_time = []    
        self.start_time = time.time() 
        
         # Record when we start collecting data
        
    def add_points(self, angles, distances):
        current_time = time.time() - self.start_time
        self.angles.extend(angles)
        self.distances.extend(distances)
        self.timestamps.extend([current_time] * len(angles))
        self.execution_time.append(current_time)
        
    def plot_data(self):
        if not self.distances:
            print("No data to plot!")
            return
            
        # Convert to numpy arrays for easier manipulation
        angles = np.array(self.angles)
        distances = np.array(self.distances)
        timestamps = np.array(self.timestamps)
        
        # Convert polar to cartesian coordinates
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        
        # Create color array based on timestamps for visualization
        colors = timestamps - min(timestamps)
        
        # Create the plot
        plt.figure(figsize=(12, 12))
        scatter = plt.scatter(x, y, c=colors, s=2, cmap='viridis', alpha=0.6)
        plt.colorbar(scatter, label='Time (seconds)')
        plt.grid(True)
        plt.axis('equal')
        plt.title(f'LiDAR Data Points Over Time\nTotal Points: {len(self.distances)}')
        plt.xlabel('X (mm)')
        plt.ylabel('Y (mm)')
        
        # Set axis limits
        # limit = max(abs(np.max(x)), abs(np.max(y)), 5000) 
        limit = max(abs(np.max(x)), abs(np.max(y)),300)
        plt.xlim(-300, 300)
        plt.ylim(-300, 300)
        
        # Add collection duration
        duration = max(timestamps) - min(timestamps)
        plt.figtext(0.02, 0.02, f"Collection Duration: {duration:.1f} seconds", fontsize=10)
        
        plt.show()

def process_packet(packet, collector):
    try:
        # Split the packet into its parts
        header, length, speed, start_angle, data_bytes, end_angle, timestamp, crc = struct.unpack(
            "<B B H H 36s H H H",
            packet
        )
        
        angles = []      # List for storing directions
        distances = []   # List for storing distances
        
        # Look at each of the 12 measurements in the packet
        for i in range(12):
            offset = i * 3  # Each measurement is 3 bytes apart
            
            # Get the three parts of each measurement
            distance_l, distance_h, confidence = struct.unpack("<BBB", data_bytes[offset:offset + 3])
            
            # Combine the two distance parts into one number
            distance = (distance_h << 8) | distance_l
            
            # Calculate the exact angle for this measurement
            angle = start_angle + (i * (end_angle - start_angle) / 12.0)
            
            # Convert angle to radians (what computers prefer)
            angle_rad = math.radians(angle / 100.0)
            
            print(f"Confidence: {confidence}, Distance: {distance}")
            
            # Only keep measurements that are:
            # - High quality (confidence > 200)
            # - Valid distance (> 0)
            # - Not too far away (< 8000mm or 8 meters)
            if confidence > 210 and distance > 0 and distance < 8000:
                angles.append(angle_rad)      # Save the direction
                distances.append(distance)     # Save the distance
        
        # If we found any good measurements, save them
        if angles and distances:
            collector.add_points(angles, distances)
            
    except struct.error as e:
        print(f"Error unpacking data: {e}")  # Report any data reading errors
    except Exception as e:
        print(f"Unexpected error processing packet: {e}")  # Report any other errors

def collect_data(duration_seconds=120):
    """Collect LiDAR data for specified duration"""
    collector = LidarDataCollector()
    port = "COM3"
    
    try:
        # Setup serial connection to LiDAR
        ser = serial.Serial(port, baudrate=115200, timeout=1)
        print(f"Connected to {port}")
        
        # Clear any old data in buffer
        ser.reset_input_buffer()
        
        print(f"\nCollecting data for {duration_seconds} seconds...")
        start_time = time.time()
        packet_buffer = bytearray()
        
        while (time.time() - start_time) < duration_seconds:
            elapsed = time.time() - start_time
            print(f"\rProgress: {elapsed:.1f}/{duration_seconds}s - Points: {len(collector.distances)}", 
                  end="", flush=True)
            
            # Read one byte at a time
            byte_data = ser.read(1)
            if not byte_data:
                continue
                
            packet_buffer.extend(byte_data)
            
            if len(packet_buffer) >= 48:  # Complete packet is 48 bytes
                if packet_buffer[0] == 0x54:  # Check for packet header
                    packet = packet_buffer[:48]  # Get the packet
                    process_packet(packet, collector)  # Process it
                    packet_buffer = packet_buffer[48:]  # Remove processed packet
                else:
                    packet_buffer = packet_buffer[1:]  # Remove first byte if invalid
        
        print(f"\nData collection complete! Collected {len(collector.distances)} points")
        
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
            print("Serial port closed")
    
    return collector

if __name__ == "__main__":
    # Collect data for 30 seconds
    print("Starting data collection...")
    collector = collect_data(duration_seconds=30)
    
    # Save data to CSV file
    filename = f"lidar_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with open(filename, 'w') as f:
        f.write("Time(s),Angle(rad),Distance(mm)\n")
        for t, a, d in zip(collector.timestamps, collector.angles, collector.distances):
            f.write(f"{t:.3f},{a},{d}\n")
    print(f"Data saved to {filename}")
    
    # Plot the data
    collector.plot_data()
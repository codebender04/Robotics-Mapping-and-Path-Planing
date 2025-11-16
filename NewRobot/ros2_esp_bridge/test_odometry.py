#!/usr/bin/env python3
"""
Quick test script to verify odometry calculations
"""
import math

# Robot parameters (same as in the bridge)
wheel_radius = 0.035  # meters
wheelbase = 0.24      # meters  
ticks_per_rev = 231   # encoder ticks per wheel revolution

def test_odometry():
    print("=== Odometry Calculation Test ===")
    print(f"Wheel radius: {wheel_radius}m")
    print(f"Wheelbase: {wheelbase}m") 
    print(f"Ticks per rev: {ticks_per_rev}")
    
    meters_per_tick = (2.0 * math.pi * wheel_radius) / ticks_per_rev
    print(f"Distance per tick: {meters_per_tick*1000:.3f}mm")
    
    # Test 1: One wheel revolution
    print(f"\n--- Test 1: One wheel revolution ---")
    one_rev_distance = 2.0 * math.pi * wheel_radius
    print(f"One wheel revolution = {one_rev_distance*1000:.1f}mm")
    print(f"Should take {ticks_per_rev} ticks")
    
    # Test 2: 90 degree turn
    print(f"\n--- Test 2: 90 degree turn in place ---")
    quarter_turn_distance = (math.pi * wheelbase) / 4  # quarter circle
    ticks_for_90_turn = quarter_turn_distance / meters_per_tick
    print(f"90° turn: each wheel travels {quarter_turn_distance*1000:.1f}mm")
    print(f"Should take ~{ticks_for_90_turn:.0f} ticks per wheel")
    
    # Test 3: Rectangle corner (1m straight + 90° turn)
    print(f"\n--- Test 3: Rectangle corner (1m + 90°) ---")
    straight_ticks = 1.0 / meters_per_tick  # 1 meter
    total_ticks = straight_ticks + ticks_for_90_turn
    print(f"1m straight: {straight_ticks:.0f} ticks")
    print(f"90° turn: {ticks_for_90_turn:.0f} ticks")
    print(f"Total per corner: {total_ticks:.0f} ticks")

if __name__ == '__main__':
    test_odometry()
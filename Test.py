import math

def inches_to_mm(inches):
    return inches * 25.4

def mm_to_inches(mm):
    return mm / 25.4

def diameter_to_circumference(diameter):
    return diameter * math.pi

while True:
    diameter = input("Enter the diameter of the circle in inches: ")
    diameter = float(diameter)
    circumference = diameter_to_circumference(diameter)
    circumference = inches_to_mm(circumference)
    tooth_count = circumference / 5
    print(tooth_count)
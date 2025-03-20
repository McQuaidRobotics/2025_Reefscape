import math
from wpimath.geometry import Rotation2d, Translation2d

ANGLE = Rotation2d.fromDegrees(-35)
MAGNITUDE = 5.0 * 0.0254

print(Translation2d(MAGNITUDE, ANGLE))
from wpimath.controller import SimpleMotorFeedforwardRadians


# def inches_to_mm(inches):
#     return inches * 25.4

# def mm_to_inches(mm):
#     return mm / 25.4

# def diameter_to_circumference(diameter):
#     return diameter * math.pi

# while True:
#     diameter = input("Enter the diameter of the circle in inches: ")
#     diameter = float(diameter)
#     circumference = diameter_to_circumference(diameter)
#     circumference = inches_to_mm(circumference)
#     tooth_count = circumference / 5
#     print(tooth_count)

mff = SimpleMotorFeedforwardRadians(0.0, 2.606 / math.tau, 0.095481 / math.tau)

print(mff.kV)
print(mff.kA)
print(mff.maxAchievableAcceleration(12.0, 0.0))

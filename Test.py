import math

FX = 913.6 # mm
FY = 912.9 # mm
CX = 641.63 # px
CY = 363.79 # px

def horizontal_fov(fx, width):
    return 2 * math.atan2(width, 2 * fx) * 180 / math.pi

def vertical_fov(fy, height):
    return 2 * math.atan2(height, 2 * fy) * 180 / math.pi

# FOV_Diagonal   = 2 * atan2(sqrt(W^2 + H^2)/2, f) * 180 / pi

def diagonal_fov(fx, fy, width, height):
    return 2 * math.atan2(math.sqrt(width**2 + height**2) / 2, fx) * 180 / math.pi

h_fov = horizontal_fov(FX, 1280)
v_fov = vertical_fov(FY, 800)

print("Horizontal FOV: {:.2f} degrees".format(h_fov))
print("Vertical FOV: {:.2f} degrees".format(v_fov))
print("Diagonal FOV: {:.2f} degrees".format(diagonal_fov(FX, FY, 1280, 800)))
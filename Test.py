def inches_to_meters(inches):
    return inches * 0.0254

def meters_to_inches(meters):
    return meters / 0.0254

def calc_cg_height(elevator_height: float) -> float:
    MAX_HEIGHT = 80.75
    MIN_HEIGHT = 12.75
    MAX_CG = 16.75
    MIN_CG = 7.25

    t = (elevator_height - MIN_HEIGHT) / (MAX_HEIGHT - MIN_HEIGHT)
    return inches_to_meters(MIN_CG + t * (MAX_CG - MIN_CG))

WHEEL_BASE_WIDTH = 0.52705
CG_HEIGHT = calc_cg_height(80)

print(CG_HEIGHT)
print(((9.81 * (WHEEL_BASE_WIDTH / 2.0)) / (CG_HEIGHT)) * 0.75)
import math
from regex import R
import wpimath.geometry as geo
from wpimath.geometry import Translation2d, Rotation2d, Pose2d, Transform2d
import enum

def inchesToMeters(inches: float) -> float:
    return inches * 0.0254

def rotateAround(this: Translation2d, other: Translation2d, rot: Rotation2d) -> Translation2d:
    return Translation2d(
        (this.X() - other.X()) * rot.cos() - (this.Y() - other.Y()) * rot.sin() + other.X(),
        (this.X() - other.X()) * rot.sin() + (this.Y() - other.Y()) * rot.cos() + other.Y())

REEF_CENTER: Translation2d = Translation2d(inchesToMeters(176.746), inchesToMeters(158.501))
FACE_OFFSET: Translation2d = Translation2d(inchesToMeters(32.75), 0)
BRANCH_OFFSET: float = inchesToMeters(6.5)
ROBOT_WIDTH: float = inchesToMeters(32.5)

class Side(enum.Enum):
    CLOSE_LEFT = 0
    CLOSE_MID = 1
    CLOSE_RIGHT = 2
    FAR_LEFT = 3
    FAR_MID = 4
    FAR_RIGHT = 5

    def angle(self) -> Rotation2d:
        if self == Side.CLOSE_LEFT:
            return Rotation2d.fromDegrees(120)
        elif self == Side.CLOSE_MID:
            return Rotation2d.fromDegrees(180)
        elif self == Side.CLOSE_RIGHT:
            return Rotation2d.fromDegrees(-120)
        elif self == Side.FAR_LEFT:
            return Rotation2d.fromDegrees(60)
        elif self == Side.FAR_MID:
            return Rotation2d.fromDegrees(0)
        else:
            return Rotation2d.fromDegrees(-60)

    def namePascal(self) -> str:
        if self == Side.CLOSE_LEFT:
            return "CloseLeft"
        elif self == Side.CLOSE_MID:
            return "CloseMid"
        elif self == Side.CLOSE_RIGHT:
            return "CloseRight"
        elif self == Side.FAR_LEFT:
            return "FarLeft"
        elif self == Side.FAR_MID:
            return "FarMid"
        else:
            return "FarRight"

    def face(self) -> Pose2d:
        center_pose = Pose2d(REEF_CENTER, Rotation2d())
        return center_pose + Transform2d(FACE_OFFSET.rotateBy(self.angle()), self.angle())

    def score_pose(self, dist_from_face: float, y_offset: float) -> Pose2d:
        t = Side.FAR_MID.face().translation() + Translation2d(dist_from_face, y_offset)
        t = rotateAround(t, REEF_CENTER, self.face().rotation())
        return Pose2d(t, self.face().rotation().rotateBy(Rotation2d(math.pi)))

    def scoreLeft(self, dist_from_face: float) -> Pose2d:
        return self.score_pose(dist_from_face, -BRANCH_OFFSET)

    def scoreRight(self, dist_from_face: float) -> Pose2d:
        return self.score_pose(dist_from_face, BRANCH_OFFSET)

    def scoreMid(self, dist_from_face: float) -> Pose2d:
        return self.score_pose(dist_from_face, 0)

def poseToJson(pose: Pose2d) -> dict[str, dict[str, float | str]]:
    return {
        "x": {
            "exp": f"{pose.X()} m",
            "val": pose.X()
        },
        "y": {
            "exp": f"{pose.Y()} m",
            "val": pose.Y()
        },
        "heading": {
            "exp": f"{pose.rotation().radians()} rad",
            "val": pose.rotation().radians()
        }
    }

import json
chor: dict
with open("./src/deploy/choreo/reefscape.chor", "r") as f:
    chor = json.load(f)
    pose_store = chor["variables"]["poses"]
    dist_from_face = (ROBOT_WIDTH / 2) + inchesToMeters(5.0)
    for side in Side:
        pose_store[side.namePascal() + "_L"] = poseToJson(side.scoreLeft(dist_from_face))
        pose_store[side.namePascal() + "_M"] = poseToJson(side.scoreMid(dist_from_face))
        pose_store[side.namePascal() + "_R"] = poseToJson(side.scoreRight(dist_from_face))

with open("./src/deploy/choreo/reefscape.chor", "w") as f:
    json.dump(chor, f, indent=4)

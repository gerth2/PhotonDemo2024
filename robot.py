from wpilib import Timer, SmartDashboard, TimedRobot, Servo
from wpimath.geometry import Transform3d, Pose3d, Translation3d, Rotation3d
from wpimath.units import inchesToMeters
import math
from photonlibpy.photonCamera import PhotonCamera, setVersionCheckEnabled #VisionLEDMode


CAM_MOUNT_POSE = Pose3d(
    Translation3d(
        inchesToMeters(0.0), #TODO update these
        inchesToMeters(-10.0),
        inchesToMeters(1.0),
    ), 
    Rotation3d.fromDegrees(
        0.0,30.0,0.0 #TODO update these
    )
)

POINTER_GIMBAL_MOUNT_POSE = Pose3d(
    Translation3d(
        inchesToMeters(0.0), #TODO update these
        inchesToMeters(10.0),
        inchesToMeters(1.0),
    ), 
    Rotation3d.fromDegrees(
        0.0,0.0,0.0 
    )
)

class MyRobot(TimedRobot):

    def robotInit(self):
        self.xAxisServo = Servo(0)
        self.yAxisServo = Servo(1)
        self.xAxisAngleCmd = 0.0
        self.yAxisAngleCmd = 0.0

        setVersionCheckEnabled(False)
        self.cam = PhotonCamera("DEMO")


    def robotPeriodic(self) -> None:
        SmartDashboard.putNumber("X Angle Cmd", self.xAxisAngleCmd)
        SmartDashboard.putNumber("Y Angle Cmd", self.yAxisAngleCmd)

        self.xAxisServo.setAngle(self.xAxisAngleCmd)
        self.yAxisServo.setAngle(self.yAxisAngleCmd)


    def teleopPeriodic(self):
        self.xAxisAngleCmd = 90.0
        self.yAxisAngleCmd = 90.0

    def autonomousPeriodic(self) -> None:
        self.xAxisAngleCmd = 30.0 * math.sin(2 * math.pi * Timer.getFPGATimestamp() * 0.25) + 90.0
        self.yAxisAngleCmd = 30.0 * math.cos(2 * math.pi * Timer.getFPGATimestamp() * 0.25) + 90.0
        res = self.cam.getLatestResult()

        for target in res.getTargets():
            # Transform both poses to on-field poses
            tgtID = target.getFiducialId()
            if (tgtID % 2) == 0:
                # Demo - only track even targets, ignore odd ones
                tgtPose = target.getBestCameraToTarget()
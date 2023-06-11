package bhs.devilbotz;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final double loopPeriodSecs = 0.02;
    public static final boolean tuningMode = false;
    private static final RobotType robot = RobotType.ROBOT_2024SIM;

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.ROBOT_2024SIM) { // Invalid robot selected
                return RobotType.ROBOT_2024S;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_2024S:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case ROBOT_2024SIM:
                return Mode.SIM;

            default:
                return Mode.REAL;
        }
    }

    public enum RobotType {
        ROBOT_2024S, ROBOT_2024SIM
    }

    public enum Mode {
        REAL, REPLAY, SIM
    }

    public static class VisionConstants {
        public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.5, 0.0, 0.5),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final String cameraName = "YOUR CAMERA NAME";
    }
}

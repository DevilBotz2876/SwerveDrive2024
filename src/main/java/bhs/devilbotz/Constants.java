package bhs.devilbotz;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  // How often to run a loop of the robot
  // Default is 20ms (do not change unless you know what you're doing
  public static final double loopPeriodSecs = 0.02;
  // Tuning mode toggle
  public static final boolean tuningMode = false;
  // Which robot to use?
  // Options:
  // - ROBOT_2024S
  // - ROBOT_2024SIM
  private static final RobotType robot = RobotType.ROBOT_2024S;

  // Get the current robot
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

  // Get the current mode
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

  // Robot Type Enum
  public enum RobotType {
    ROBOT_2024S,
    ROBOT_2024SIM
  }

  // Robot Type Mode
  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  // Vision Constants
  public static class VisionConstants {
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(0),
                Units.degreesToRadians(
                    0))); // Cam mounted facing forward, half a meter forward of center, half a
    // meter up
    // from center.
    // TODO: Get camera name
    public static final String cameraName = "YOUR CAMERA NAME";
  }
}

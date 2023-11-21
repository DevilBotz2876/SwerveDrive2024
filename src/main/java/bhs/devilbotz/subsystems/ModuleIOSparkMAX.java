package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.utils.SparkMAXBurnManager;
import bhs.devilbotz.utils.SparkMaxDerivedVelocityController;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Hardware interface for a swerve module with two Spark MAXs as drive and steer motor controllers
 */
public class ModuleIOSparkMAX implements ModuleIO {
  // 2 Controllers for 2 NEOs
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  // Encoder Initialization
  private final SparkMaxDerivedVelocityController driveDerivedVelocityController;
  private final RelativeEncoder driveDefaultEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANCoder turnAbsoluteEncoder;

  private final Rotation2d absoluteEncoderOffset;

  // Encoder resolution
  private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  // Booleans for setup
  private final boolean isTurnMotorInverted = true;
  private final double turnAbsoluteEncoderTicksPerRevolution = 4096.0;

  /**
   * Initialize the Module
   *
   * @param index which module to initialize
   */
  public ModuleIOSparkMAX(int index) {
    // Change the hardware depending on the type of robot
    switch (Constants.getRobot()) {
      case ROBOT_2024S:
        // Depending on the module, initialize the hardware
        switch (index) {
          case 0:
            driveSparkMax = new CANSparkMax(10, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(14, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(18);

            turnAbsoluteEncoder.configMagnetOffset(51);
            absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(0));

            break;
          case 1:
            driveSparkMax = new CANSparkMax(11, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(15, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(19);

            turnAbsoluteEncoder.configMagnetOffset(113);
            absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(0));
            break;
          case 2:
            driveSparkMax = new CANSparkMax(12, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(16, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(20);

            turnAbsoluteEncoder.configMagnetOffset(85);
            absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(0));
            break;
          case 3:
            driveSparkMax = new CANSparkMax(13, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(17, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(21);

            turnAbsoluteEncoder.configMagnetOffset(293);
            absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(0));
            break;
          default:
            throw new RuntimeException("Invalid module index for ModuleIOSparkMAX");
        }
        turnAbsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        break;
      default:
        throw new RuntimeException("Invalid robot for ModuleIOSparkMAX");
    }

    // Burn the Spark MAXs depending on their status
    if (SparkMAXBurnManager.shouldBurn()) {
      driveSparkMax.restoreFactoryDefaults();
      turnSparkMax.restoreFactoryDefaults();
    }

    // Invert the turn spark
    turnSparkMax.setInverted(isTurnMotorInverted);

    driveSparkMax.setInverted(true);

    // Set the current limit to 30 amps
    driveSparkMax.setSmartCurrentLimit(18);
    turnSparkMax.setSmartCurrentLimit(18);
    // Enable voltage compensation
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    // Assign the encoders
    driveDerivedVelocityController = new SparkMaxDerivedVelocityController(driveSparkMax);
    driveDefaultEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnRelativeEncoder.setPosition(0.0);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    // Burn the spark maxes if necessary
    if (SparkMAXBurnManager.shouldBurn()) {
      driveSparkMax.burnFlash();
      turnSparkMax.burnFlash();
    }
  }

  /**
   * Update the AK hardware inputs
   *
   * @param inputs the inputs to update
   */
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveDerivedVelocityController.getPosition())
            / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveDerivedVelocityController.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveVelocityFilteredRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveDefaultEncoder.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveAppliedVolts =
        driveSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();
    inputs.driveTempCelcius = driveSparkMax.getMotorTemperature();

    inputs.turnAbsolutePositionRad =
        Units.degreesToRadians(turnAbsoluteEncoder.getAbsolutePosition()) - absoluteEncoderOffset.getRadians();

    inputs.turnPositionRad =
        Units.rotationsToRadians(turnRelativeEncoder.getPosition()) / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / turnAfterEncoderReduction;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    inputs.turnTempCelcius = turnSparkMax.getMotorTemperature();
  }

  /**
   * Set the drive voltage of the Spark MAX
   *
   * @param volts The voltage to run the Spark Max at
   */
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  /**
   * Set the turn voltage of the Spark MAX
   *
   * @param volts The voltage to run the Spark Max at
   */
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  /**
   * Set the brake mode of the drive NEO
   *
   * @param enable whether to enable the brake mode or not
   */
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  /**
   * Set the brake mode of the turn NEO
   *
   * @param enable whether to enable the brake mode or not
   */
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}

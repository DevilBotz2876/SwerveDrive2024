package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Objects;

public class ModuleIOFalcon implements ModuleIO {
    private final WPI_TalonFX driveFalcon;
    private final WPI_TalonFX turnFalcon;

    private final CANCoder turnAbsoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;

    private final double driveGearRatio = Mk4SwerveModuleHelper.GearRatio.L3.getConfiguration().getDriveReduction();
    private final double turnGearRatio = Mk4SwerveModuleHelper.GearRatio.L3.getConfiguration().getSteerReduction();

    private final TalonFXSensorCollection driveDefaultEncoder;
    private final TalonFXSensorCollection turnRelativeEncoder;

    private final double driveEncoderTicksPerRevolution = 2048.0;
    private final double turnEncoderTicksPerRevolution = 2048.0;
    private final double turnAbsoluteEncoderTicksPerRevolution = 4096.0;


    public ModuleIOFalcon(int index) {
        if (Objects.requireNonNull(Constants.getRobot()) == Constants.RobotType.ROBOT_2024S) {
            switch (index) {
                case 0:
                    driveFalcon = new WPI_TalonFX(11);
                    turnFalcon = new WPI_TalonFX(12);
                    turnAbsoluteEncoder = new CANCoder(13);
                    absoluteEncoderOffset = new Rotation2d(0.0);
                    break;
                case 1:
                    driveFalcon = new WPI_TalonFX(14);
                    turnFalcon = new WPI_TalonFX(15);
                    turnAbsoluteEncoder = new CANCoder(16);
                    absoluteEncoderOffset = new Rotation2d(0.0);
                    break;
                case 2:
                    driveFalcon = new WPI_TalonFX(17);
                    turnFalcon = new WPI_TalonFX(18);
                    turnAbsoluteEncoder = new CANCoder(19);
                    absoluteEncoderOffset = new Rotation2d(0.0);
                    break;
                case 3:
                    driveFalcon = new WPI_TalonFX(20);
                    turnFalcon = new WPI_TalonFX(21);
                    turnAbsoluteEncoder = new CANCoder(22);
                    absoluteEncoderOffset = new Rotation2d(0.0);
                    break;
                default:
                    throw new RuntimeException(
                            "Invalid module index for ModuleIOFalcon");
            }
        } else {
            throw new RuntimeException("Invalid robot for ModuleIOFalcon");
        }

        driveFalcon.configFactoryDefault();
        turnFalcon.configFactoryDefault();

        turnFalcon.setInverted(true);

        // Set current limit to 30 amps for drive and 30 amps for turn
        driveFalcon.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true, 30, 30, 0.1));
        turnFalcon.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true, 30, 30, 0.1));

        // Set voltage compensation to 12 volts
        driveFalcon.configVoltageCompSaturation(12.0);
        driveFalcon.enableVoltageCompensation(true);
        turnFalcon.configVoltageCompSaturation(12.0);
        turnFalcon.enableVoltageCompensation(true);

        driveDefaultEncoder = driveFalcon.getSensorCollection();
        turnRelativeEncoder = turnFalcon.getSensorCollection();

        resetEncoders();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = Units.rotationsToRadians(
                driveDefaultEncoder.getIntegratedSensorPosition() / driveEncoderTicksPerRevolution);
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                driveDefaultEncoder.getIntegratedSensorVelocity() / driveEncoderTicksPerRevolution * 60.0);
        inputs.drivePositionMeters = inputs.drivePositionRad * driveGearRatio * Mk4SwerveModuleHelper.GearRatio.L2.getConfiguration().getWheelDiameter();

        inputs.driveVolts = driveFalcon.getMotorOutputVoltage();
        inputs.driveCurrentAmps = driveFalcon.getStatorCurrent();
        inputs.driveTempCelcius = driveFalcon.getTemperature();

        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(
                turnAbsoluteEncoder.getAbsolutePosition() / turnAbsoluteEncoderTicksPerRevolution);
        inputs.turnPositionRad = Units.rotationsToRadians(
                turnRelativeEncoder.getIntegratedSensorPosition() / turnEncoderTicksPerRevolution);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                turnRelativeEncoder.getIntegratedSensorVelocity() / turnEncoderTicksPerRevolution * 60.0);

        inputs.turnVolts = turnFalcon.getMotorOutputVoltage();
        inputs.turnCurrentAmps = turnFalcon.getStatorCurrent();
        inputs.turnTempCelcius = turnFalcon.getTemperature();
    }

    public void setDriveVoltage(double volts) {
        driveFalcon.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        turnFalcon.setVoltage(volts);
    }

    public void setDriveBrakeMode(boolean enable) {
        driveFalcon.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnFalcon.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void resetEncoders() {
        driveDefaultEncoder.setIntegratedSensorPosition(0, 0);
        turnRelativeEncoder.setIntegratedSensorPosition(0, 0);
    }
}
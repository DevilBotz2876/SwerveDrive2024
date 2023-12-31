package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Hardware interface for a swerve module with two simulated motors as drive and steer motors
 */
public class ModuleIOSim implements ModuleIO {
    // L2 swerve module is 6.75:1 drive overall ratio
    private final FlywheelSim driveSim =
            new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    // Swerve module turn ratio is 150/7:1 overall
    private final FlywheelSim turnSim =
            new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

    // Module states
    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * Update the AK hardware inputs
     *
     * @param inputs the inputs to update
     */
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);

        double angleDiffRad =
                turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelcius = 0.0;

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnPositionRad = turnRelativePositionRad;
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        inputs.turnTempCelcius = 0.0;
    }

    /**
     * Set the voltage of the drive sim motor
     *
     * @param volts The voltage to set the sim motor to
     */
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    /**
     * Set the voltage of the turn sim motor
     *
     * @param volts The voltage to set the sim motor to
     */
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}
package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ModuleIOSim implements ModuleIO {
    // L2 swerve module is 6.75:1 drive overall ratio
    private final FlywheelSim driveSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    // Swerve module turn ratio is 150/7:1 overall
    private final FlywheelSim turnSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

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
        inputs.drivePositionMeters = inputs.drivePositionRad * (Mk4SwerveModuleHelper.GearRatio.L2.getConfiguration().getWheelDiameter() / 2.0);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelcius = 0.0;

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnPositionRad = turnRelativePositionRad;
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        inputs.turnTempCelcius = 0.0;
    }

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}
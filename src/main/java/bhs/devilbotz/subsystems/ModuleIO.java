package bhs.devilbotz.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Run the drive motor at the specified voltage.
     */
    default void setDriveVoltage(double volts) {
    }

    /**
     * Run the turn motor at the specified voltage.
     */
    default void setTurnVoltage(double volts) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrakeMode(boolean enable) {
    }

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setTurnBrakeMode(boolean enable) {
    }

    @AutoLog
    class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double drivePositionMeters = 0.0;
        public double driveVelocityRadPerSec = 0.0;

        public double driveVelocityFilteredRadPerSec = 0.0;

        public double driveVolts = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;

        public double turnVolts = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelcius = 0.0;
    }
}
package bhs.devilbotz.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    default void updateInputs(GyroIOInputs inputs) {
    }

    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public double pitchRad = 0.0;
        public double velocityPitchRadPerSec = 0.0;
        public double yawRad = 0.0;
        public double velocityYawRadPerSec = 0.0;
        public double rollRad = 0.0;
        public double velocityRollRadPerSec = 0.0;
    }
}
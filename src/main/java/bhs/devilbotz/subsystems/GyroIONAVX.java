package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import java.util.Objects;

public class GyroIONAVX implements GyroIO {
    private final AHRS navx;

    public GyroIONAVX() {
        if (Objects.requireNonNull(Constants.getRobot()) == Constants.RobotType.ROBOT_2024S) {
            navx = new AHRS(SPI.Port.kMXP);
        } else {
            throw new RuntimeException("Invalid robot for NAVX");
        }
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.pitchRad = navx.getRawGyroX();
        inputs.velocityPitchRadPerSec = navx.getVelocityX();
        inputs.yawRad = navx.getRawGyroY();
        inputs.velocityYawRadPerSec = navx.getVelocityY();
        inputs.rollRad = navx.getRawGyroZ();
        inputs.velocityRollRadPerSec = navx.getVelocityZ();

    }
}

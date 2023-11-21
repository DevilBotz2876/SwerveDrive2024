package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import java.util.Objects;

/**
 * Hardware interface for the NAVX 3 axis gyro
 */
public class GyroIONAVX implements GyroIO {
    private final AHRS navx;

    /**
     * Constructor to initialize the NAVX
     */
    public GyroIONAVX() {
        if (Objects.requireNonNull(Constants.getRobot()) == Constants.RobotType.ROBOT_2024S) {
            navx = new AHRS(SPI.Port.kMXP);
        } else {
            throw new RuntimeException("Invalid robot for NAVX");
        }
    }

    /**
     * Update the AK hardware inputs
     *
     * @param inputs the inputs to update
     */
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.pitchRad = Units.degreesToRadians(-navx.getPitch());
        inputs.velocityPitchRadPerSec = navx.getVelocityX();
        inputs.yawRad = Units.degreesToRadians(-navx.getYaw());
        inputs.velocityYawRadPerSec = navx.getVelocityY();
        inputs.rollRad = Units.degreesToRadians(-navx.getRoll());
        inputs.velocityRollRadPerSec = navx.getVelocityZ();

    }
}

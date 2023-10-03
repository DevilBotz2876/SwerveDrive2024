package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.Drive;
import bhs.devilbotz.utils.GeomUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * Loop-able command responsible for Driving the chassis with Joysticks
 * This subsystem is responsible for:
 * <ul>
 * <li>Handling the inputs from the controller and translating it to drive inputs
 * <li>Controlling and executing the joystick driving mode
 * <li>Applying dead-bands, curves, and conversion to velocity
 * </ul>
 *
 * @author Parker Meyers
 * @category Drive
 * @category Swerve
 * @category Control
 * @see Drive
 */
public class DriveWithJoysticks extends CommandBase {
    private static final double deadband = 0.1;
    private final Drive drive;
    private final Supplier<Double> leftXSupplier;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> rightYSupplier;
    private final Supplier<Boolean> robotRelativeOverride;
    private final Supplier<JoystickMode> modeSupplier;
    private final Supplier<Double> linearSpeedLimitSupplier;
    private final Supplier<Double> angularSpeedLimitSupplier;
    private final Supplier<Double> autoDriveSupplier;

    /**
     * DriveWithJoysticks constructor
     *
     * @param drive                     The {@link Drive} subsystem, used to send final velocity
     * @param leftXSupplier             The left X value from the controller
     * @param leftYSupplier             The left Y value from the controller
     * @param rightYSupplier            The right Y value from the controller
     * @param robotRelativeOverride     Toggle if the robot is field relative or robot relative
     * @param modeSupplier              Joystick mode from the chooser
     * @param linearSpeedLimitSupplier  Linear speed limit
     * @param angularSpeedLimitSupplier Angular speed limit
     * @param autoDriveSupplier         Supplier for auto driving
     */
    public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier,
                              Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier,
                              Supplier<Boolean> robotRelativeOverride,
                              Supplier<JoystickMode> modeSupplier,
                              Supplier<Double> linearSpeedLimitSupplier,
                              Supplier<Double> angularSpeedLimitSupplier,
                              Supplier<Double> autoDriveSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightYSupplier = rightYSupplier;
        this.robotRelativeOverride = robotRelativeOverride;
        this.modeSupplier = modeSupplier;
        this.linearSpeedLimitSupplier = linearSpeedLimitSupplier;
        this.angularSpeedLimitSupplier = angularSpeedLimitSupplier;
        this.autoDriveSupplier = autoDriveSupplier;
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Main loop, gets the values from the joystick, applies dead-bands and squaring,
     * gets the velocity and executes them.
     */
    @Override
    public void execute() {
        // Get values from double suppliers
        double leftX = leftXSupplier.get();
        double leftY = leftYSupplier.get();
        double rightY = rightYSupplier.get();

        // Get direction and magnitude of linear axes
        double linearMagnitude = Math.hypot(leftX, leftY);
        Rotation2d linearDirection = new Rotation2d(leftX, leftY);

        // Apply deadband
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
        rightY = MathUtil.applyDeadband(rightY, deadband);

        // Apply squaring
        linearMagnitude =
                Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
        rightY = Math.copySign(rightY * rightY, rightY);

        // Apply speed limits
        linearMagnitude *= linearSpeedLimitSupplier.get();
        rightY *= angularSpeedLimitSupplier.get();

        // Calculate new linear components
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(
                                GeomUtil.transformFromTranslation(linearMagnitude, 0.0))
                        .getTranslation();
        if (modeSupplier.get() == JoystickMode.TANK) {
            linearVelocity = new Translation2d(linearVelocity.getX(), 0.0);
        }

        // Convert to meters per second
        ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                rightY * drive.getMaxAngularSpeedRadPerSec());

        // Convert from field relative
        if (!robotRelativeOverride.get()
                && modeSupplier.get() == JoystickMode.STANDARD) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                    drive.getRotation());
        }

        // Apply auto drive
        // ChassisSpeeds autoDriveSpeeds =
        // AutoDriveSoftWithSpline.calculate(drive.getPose(),
        // autoDriveSupplier.get() * drive.getMaxLinearSpeedMetersPerSec(),
        // autoDriveSupplier.get() > 0.08);
        // speeds = new ChassisSpeeds(
        // speeds.vxMetersPerSecond + autoDriveSpeeds.vxMetersPerSecond,
        // speeds.vyMetersPerSecond + autoDriveSpeeds.vyMetersPerSecond,
        // speeds.omegaRadiansPerSecond + autoDriveSpeeds.omegaRadiansPerSecond);

        // Send to drive
        drive.runVelocity(speeds);
    }

    /**
     * Called once the command ends or is interrupted.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    /**
     * Returns true when the command should end.
     *
     * @return if the command is finished
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * The available modes for the joystick
     */
    public enum JoystickMode {
        STANDARD, TANK
    }
}
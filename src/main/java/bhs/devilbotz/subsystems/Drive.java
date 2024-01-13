package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.utils.GeomUtil;
import bhs.devilbotz.utils.LoggedTunableNumber;
import bhs.devilbotz.utils.PhotonCameraWrapper;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.Arrays;
import java.util.Optional;

/**
 * Drive subsystem for the DevilBotz 2024 swerve drive base.
 * This subsystem is responsible for:
 * - Handling the swerve module inputs and outputs logic
 * - Performing the swerve module kinematics
 * - Performing the swerve module PID control
 * - Predicting the robot pose using the swerve module positions, gyro, and vision
 *
 * @author Parker Meyers
 * @category Drive
 * @category Swerve
 * @category Pose
 * @category Vision
 * @category Kinematics
 * @see ModuleIO
 * @see GyroIO
 * @see PhotonCameraWrapper
 * @see SwerveDriveKinematics
 * @see SwerveDrivePoseEstimator
 * @see PIDController
 * @see SimpleMotorFeedforward
 */
public class Drive extends SubsystemBase {
    // What value to switch the robot to coast while under, needs to be lower then 0.05 to switch to coast while disabling.
    private static final double maxCoastVelocityMetersPerSec = 0.05;

    // Define the IO & Inputs
    // Gyro
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    // Swerve Modules (4)
    // In the order of: FL, FR, BL, BR
    private final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputsAutoLogged[] moduleInputs =
            new ModuleIOInputsAutoLogged[]{
                    new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
                    new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()};

    // Define limiters
    private final double maxLinearSpeed;
    private final double maxAngularSpeed;

    // Swerve Module Kinematics
    private final double wheelRadius;
    private final double trackWidthX;
    private final double trackWidthY;

    // Define the tunable numbers for the drive PID
    private final LoggedTunableNumber driveKp =
            new LoggedTunableNumber("Drive/DriveKp");
    private final LoggedTunableNumber driveKd =
            new LoggedTunableNumber("Drive/DriveKd");
    private final LoggedTunableNumber driveKs =
            new LoggedTunableNumber("Drive/DriveKs");
    private final LoggedTunableNumber driveKv =
            new LoggedTunableNumber("Drive/DriveKv");

    // Define the tunable numbers for the turn PID
    private final LoggedTunableNumber turnKp =
            new LoggedTunableNumber("Drive/TurnKp");
    private final LoggedTunableNumber turnKd =
            new LoggedTunableNumber("Drive/TurnKd");

    // Define the kinematics/PID
    private final SwerveDriveKinematics kinematics;
    private final PIDController[] driveFeedback = new PIDController[4];
    private final PIDController[] turnFeedback = new PIDController[4];
    private final double[] lastModulePositionsRad = new double[]{0.0, 0.0, 0.0, 0.0};
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PhotonCameraWrapper pcw;
    private SimpleMotorFeedforward driveFeedforward;

    // Define the pose estimators/odometry/vision
    private Pose2d odometryPose = new Pose2d();
    private Translation2d fieldVelocity = new Translation2d();
    private Pose2d visionEstimate = new Pose2d();

    // Define module states
    private double lastGyroPosRad = 0.0;
    private boolean brakeMode = false;
    private DriveMode driveMode = DriveMode.NORMAL;
    private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();
    private double characterizationVoltage = 0.0;

    /**
     * Constructor for the Drive subsystem.
     *
     * @param gyroIO     The gyro IO object
     * @param flModuleIO The front left module IO object
     * @param frModuleIO The front right module IO object
     * @param blModuleIO The back left module IO object
     * @param brModuleIO The back right module IO object
     * @param pcw        The PhotonCameraWrapper object
     * @category Constructor
     * @category Drive
     * @category Swerve
     * @see GyroIO
     * @see ModuleIO
     * @see PhotonCameraWrapper
     */
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
                 ModuleIO blModuleIO, ModuleIO brModuleIO, PhotonCameraWrapper pcw) {
        this.pcw = pcw;
        this.gyroIO = gyroIO;
        moduleIOs[0] = flModuleIO;
        moduleIOs[1] = frModuleIO;
        moduleIOs[2] = blModuleIO;
        moduleIOs[3] = brModuleIO;

        // Set the kinematics, max speeds, and PID values based on the robot
        switch (Constants.getRobot()) {
            case ROBOT_2024S:
                // NEO: 14.5 free speed
                // Falcon: 16.3 free speed
                maxLinearSpeed = Units.feetToMeters(14.5);
                wheelRadius = Units.inchesToMeters(2.0);
                trackWidthX = Units.inchesToMeters(25.0);
                trackWidthY = Units.inchesToMeters(25.0);

                // TODO: Tune the PID
                driveKp.initDefault(0.015);
                driveKd.initDefault(0.0009);
                // TODO: Tune the FF with the FFChar class
                driveKs.initDefault(0.18475);
                driveKv.initDefault(0.13439);

                // TODO: Tune the PID
                turnKp.initDefault(7.75);
                turnKd.initDefault(0.08);

                break;
            case ROBOT_2024SIM:
                maxLinearSpeed = Units.feetToMeters(14.5);
                wheelRadius = Units.inchesToMeters(2.0);
                trackWidthX = 0.65;
                trackWidthY = 0.65;

                driveKp.initDefault(0.9);
                driveKd.initDefault(0.0);
                driveKs.initDefault(0.116970);
                driveKv.initDefault(0.133240);

                turnKp.initDefault(23.0);
                turnKd.initDefault(0.0);
                break;
            default:
                // If the robot is not defined, set everything to 0
                maxLinearSpeed = 0.0;
                wheelRadius = 0.0;
                trackWidthX = 0.0;
                trackWidthY = 0.0;

                driveKp.initDefault(0.0);
                driveKd.initDefault(0.0);
                driveKs.initDefault(0.0);
                driveKv.initDefault(0.0);

                turnKp.initDefault(0.0);
                turnKd.initDefault(0.0);
                break;
        }

        // How much to trust the kinematics model
        // stateStdDevs - Standard deviations of the pose estimate (x position in meters, y position in meters, and
        // heading in radians). Increase these numbers to trust your state estimate less.
        // TODO: Tune
        Vector<N3> modelStatesDeviation = VecBuilder.fill(0.1, 0.1, 0.1);
        // How much to trust the vision
        // visionMeasurementStdDevs - Standard deviations of the vision pose measurement (x position in meters,
        // y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less
        // TODO: Tune
        Vector<N3> visionMeasurementDeviation = VecBuilder.fill(1.5, 1.5, 1.5);

        // Create the kinematics, pose estimator, and feedforward
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, odometryPose.getRotation(), getModulePositions(),
                new Pose2d(), modelStatesDeviation, visionMeasurementDeviation);

        // Set the initial vision estimate to the current pose
        visionEstimate = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

        // Create the PID controllers for each module
        for (int i = 0; i < 4; i++) {
            driveFeedback[i] = new PIDController(driveKp.get(), 0.0, driveKd.get(),
                    Constants.loopPeriodSecs);
            turnFeedback[i] = new PIDController(turnKp.get(), 0.0, turnKd.get(),
                    Constants.loopPeriodSecs);
            turnFeedback[i].enableContinuousInput(-Math.PI, Math.PI);
        }

        // Calculate max angular speed
        maxAngularSpeed = maxLinearSpeed / Arrays.stream(getModuleTranslations())
                .map(Translation2d::getNorm).max(Double::compare).get();
    }

    @Override
    public void periodic() {
        // Update the gyroIO with the current gyro inputs & log them
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        // Update each moduleIO with the current module inputs & log them
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + i,
                    moduleInputs[i]);
        }

        // Update objects based on TunableNumbers
        if (driveKp.hasChanged() || driveKd.hasChanged() || driveKs.hasChanged()
                || driveKv.hasChanged() || turnKp.hasChanged() || turnKd.hasChanged()) {
            // Update the PID controllers & feedforward
            driveFeedforward =
                    new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
            System.out.println("Drive Feedforward Updated");
            for (int i = 0; i < 4; i++) {
                driveFeedback[i].setP(driveKp.get());
                driveFeedback[i].setD(driveKd.get());
                turnFeedback[i].setP(turnKp.get());
                turnFeedback[i].setD(turnKd.get());

                System.out.println("Module " + i + " PID Values Updated");
                System.out.println("Drive Kp: " + driveKp.get());
                System.out.println("Drive Kd: " + driveKd.get());
                System.out.println("Drive Ks: " + driveKs.get());
                System.out.println("Drive Kv: " + driveKv.get());
                System.out.println("Turn Kp: " + turnKp.get());
                System.out.println("Turn Kd: " + turnKd.get());
            }
        }

        // Update angle measurements
        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] =
                    new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
        }

        // Set drive mode and safety for disabled mode
        if (DriverStation.isDisabled()) {
            // Disable output while disabled
            for (int i = 0; i < 4; i++) {
                moduleIOs[i].setTurnVoltage(0.0);
                moduleIOs[i].setDriveVoltage(0.0);
            }
        } else {
            switch (driveMode) {
                case NORMAL:
                    // In normal mode, run the controllers for turning and driving based on the current
                    // setpoint
                    SwerveModuleState[] setpointStates =
                            kinematics.toSwerveModuleStates(closedLoopSetpoint);
                    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,
                            maxLinearSpeed);

                    // If stationary, go to last state
                    boolean isStationary =
                            Math.abs(closedLoopSetpoint.vxMetersPerSecond) < 1e-3
                                    && Math.abs(closedLoopSetpoint.vyMetersPerSecond) < 1e-3
                                    && Math.abs(closedLoopSetpoint.omegaRadiansPerSecond) < 1e-3;

                    SwerveModuleState[] setpointStatesOptimized =
                            new SwerveModuleState[]{null, null, null, null};
                    for (int i = 0; i < 4; i++) {
                        // Run turn controller
                        setpointStatesOptimized[i] =
                                SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
                        if (isStationary) {
                            moduleIOs[i].setTurnVoltage(0.0);
                        } else {
                            moduleIOs[i].setTurnVoltage(
                                    turnFeedback[i].calculate(turnPositions[i].getRadians(),
                                            setpointStatesOptimized[i].angle.getRadians()));
                        }

                        // Update velocity based on turn error
                        setpointStatesOptimized[i].speedMetersPerSecond *=
                                Math.cos(turnFeedback[i].getPositionError());

                        // Run drive controller
                        double velocityRadPerSec =
                                setpointStatesOptimized[i].speedMetersPerSecond / wheelRadius;
                        moduleIOs[i].setDriveVoltage(
                                driveFeedforward.calculate(velocityRadPerSec) + driveFeedback[i]
                                        .calculate(moduleInputs[i].driveVelocityRadPerSec,
                                                velocityRadPerSec));

                        // Log individual setpoints
                        Logger.getInstance().recordOutput(
                                "SwerveSetpointValues/Drive/" + i,
                                velocityRadPerSec);
                        Logger.getInstance().recordOutput(
                                "SwerveSetpointValues/Turn/" + i,
                                setpointStatesOptimized[i].angle.getRadians());
                    }

                    // Log all module setpoints
                    Logger.getInstance().recordOutput("SwerveModuleStates/Setpoints",
                            setpointStates);
                    Logger.getInstance().recordOutput(
                            "SwerveModuleStates/SetpointsOptimized", setpointStatesOptimized);
                    break;

                case CHARACTERIZATION:
                    // In characterization mode, drive at the specified voltage (and turn to zero degrees)
                    for (int i = 0; i < 4; i++) {
                        moduleIOs[i].setTurnVoltage(
                                turnFeedback[i].calculate(turnPositions[i].getRadians(), 0.0));
                        moduleIOs[i].setDriveVoltage(characterizationVoltage);
                    }
                    break;

                case X:
                    for (int i = 0; i < 4; i++) {
                        Rotation2d targetRotation =
                                GeomUtil.direction(getModuleTranslations()[i]);
                        Rotation2d currentRotation = turnPositions[i];
                        if (Math.abs(
                                targetRotation.minus(currentRotation).getDegrees()) > 90.0) {
                            targetRotation =
                                    targetRotation.minus(Rotation2d.fromDegrees(180.0));
                        }
                        moduleIOs[i].setTurnVoltage(turnFeedback[i].calculate(
                                currentRotation.getRadians(), targetRotation.getRadians()));
                        moduleIOs[i].setDriveVoltage(0.0);
                    }
                    break;
            }
        }

        // Update odometry
        // Calculate the change in position of each module
        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                    (moduleInputs[i].drivePositionRad - lastModulePositionsRad[i])
                            * wheelRadius,
                    turnPositions[i]);
            lastModulePositionsRad[i] = moduleInputs[i].drivePositionRad;
        }

        // Convert these measured changes to chassis speeds
        ChassisSpeeds chassisStateDiff =
                kinematics.toChassisSpeeds(measuredStatesDiff);
        // Update the pose depending on if the gyro is connected
        if (gyroInputs.connected) {
            // Use gyro for angular change when connected
            odometryPose =
                    odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
                            chassisStateDiff.vyMetersPerSecond,
                            gyroInputs.yawRad - lastGyroPosRad));

            // Since we already updated the odometry pose, we can just use the current pose for the odometry class
            poseEstimator.update(odometryPose.getRotation(), getModulePositions());
        } else {
            // Fall back to using angular velocity (disconnected or sim)
            odometryPose =
                    odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
                            chassisStateDiff.vyMetersPerSecond,
                            chassisStateDiff.omegaRadiansPerSecond));
            // Since we already updated the odometry pose, we can just use the current pose for the odometry class
            poseEstimator.update(odometryPose.getRotation(), getModulePositions());
        }
        lastGyroPosRad = gyroInputs.yawRad;

        // Update field velocity
        SwerveModuleState[] measuredStates =
                new SwerveModuleState[]{null, null, null, null};
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                    moduleInputs[i].driveVelocityRadPerSec * wheelRadius,
                    turnPositions[i]);
        }
        // Convert these measured states to chassis speeds
        ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
        fieldVelocity = new Translation2d(chassisState.vxMetersPerSecond,
                chassisState.vyMetersPerSecond).rotateBy(getRotation());

        // Update Vision
        Optional<EstimatedRobotPose> result =
                pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

        // If an april tag is detected, update the pose estimator
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            poseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            visionEstimate = camPose.estimatedPose.toPose2d();
        } else {
            // if there is no vision, set the vision estimate pose off the field
            visionEstimate = new Pose2d(-20000.0, -20000.0, new Rotation2d(0));
        }

        // Log measured states
        Logger.getInstance().recordOutput("SwerveModuleStates/Measured", measuredStates);

        // Log odometry pose
        Logger.getInstance().recordOutput("Odometry/Encoders/Robot", odometryPose);

        // Log raw vision pose
        Logger.getInstance().recordOutput("Odometry/Vision/Robot", visionEstimate);

        // Log Fused Pose
        Logger.getInstance().recordOutput("Odometry/FusedOdometry/Robot", poseEstimator.getEstimatedPosition());

        // Smartly Enable/disable brake mode
        if (DriverStation.isEnabled()) {
            if (!brakeMode) {
                brakeMode = true;
                for (int i = 0; i < 4; i++) {
                    moduleIOs[i].setTurnBrakeMode(true);
                    moduleIOs[i].setDriveBrakeMode(true);
                }
            }
        } else {
            boolean stillMoving = false;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(moduleInputs[i].driveVelocityRadPerSec
                        * wheelRadius) > maxCoastVelocityMetersPerSec) {
                    stillMoving = true;
                    break;
                }
            }

            if (brakeMode && !stillMoving) {
                brakeMode = false;
                for (int i = 0; i < 4; i++) {
                    moduleIOs[i].setTurnBrakeMode(false);
                    moduleIOs[i].setDriveBrakeMode(false);
                }
            }
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        driveMode = DriveMode.NORMAL;
        closedLoopSetpoint = speeds;
    }

    /**
     * Stops the drive.
     *
     * <p> This is equivalent to calling {@link #runVelocity(ChassisSpeeds)} with
     * no velocity.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Sets the drive mode to X
     */
    public void goToX() {
        driveMode = DriveMode.X;
    }

    /**
     * @return the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed;
    }

    /**
     * @return the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return maxAngularSpeed;
    }

    /**
     * @return the current odometry pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the current odometry pose.
     *
     * @param pose the post to set the odometry to
     */
    public void setPose(Pose2d pose) {
        odometryPose = pose;
        poseEstimator.resetPosition(Rotation2d.fromRadians(gyroInputs.yawRad), getModulePositions(), pose);
    }

    /**
     * @return the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return odometryPose.getRotation();
    }

    /**
     * @return the current field velocity
     */
    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    /**
     * @return an array of module translations.
     */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
                new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
                new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
                new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)};
    }

    /**
     * @return the current module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                    moduleInputs[i].drivePositionRad * wheelRadius,
                    new Rotation2d(moduleInputs[i].turnAbsolutePositionRad));
        }

        return modulePositions;
    }

    /**
     * Runs forwards at the commanded voltage.
     *
     * @param volts the voltage to run characterization at
     */
    public void runCharacterizationVolts(double volts) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = volts;
    }

    /**
     * @return the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < 4; i++) {
            driveVelocityAverage += moduleInputs[i].driveVelocityRadPerSec;
        }
        return driveVelocityAverage / 4.0;
    }

    /**
     * The 3 drive modes
     */
    private enum DriveMode {
        NORMAL, X, CHARACTERIZATION
    }

    /** Set module angle */
    public void setModuleAngle(int module, double angle) {
        moduleIOs[module].setTurnVoltage(turnFeedback[module].calculate(
                moduleInputs[module].turnAbsolutePositionRad, angle));
    }

    public ModuleIOInputsAutoLogged getModuleInputs(int module) {
        return moduleInputs[module];
    }

    public GyroIOInputsAutoLogged getGyroInputs() {
        return gyroInputs;
    }
}
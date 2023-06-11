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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.Arrays;
import java.util.Optional;

public class Drive extends SubsystemBase {
    private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
    // switch to coast when disabling

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs =
            new GyroIOInputsAutoLogged();
    private final ModuleIO[] moduleIOs = new ModuleIO[4]; // FL, FR, BL, BR
    private final ModuleIOInputsAutoLogged[] moduleInputs =
            new ModuleIOInputsAutoLogged[]{new ModuleIOInputsAutoLogged(),
                    new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
                    new ModuleIOInputsAutoLogged()};

    private final double maxLinearSpeed;
    private final double maxAngularSpeed;
    private final double wheelRadius;
    private final double trackWidthX;
    private final double trackWidthY;

    private final LoggedTunableNumber driveKp =
            new LoggedTunableNumber("Drive/DriveKp");
    private final LoggedTunableNumber driveKd =
            new LoggedTunableNumber("Drive/DriveKd");
    private final LoggedTunableNumber driveKs =
            new LoggedTunableNumber("Drive/DriveKs");
    private final LoggedTunableNumber driveKv =
            new LoggedTunableNumber("Drive/DriveKv");

    private final LoggedTunableNumber turnKp =
            new LoggedTunableNumber("Drive/TurnKp");
    private final LoggedTunableNumber turnKd =
            new LoggedTunableNumber("Drive/TurnKd");

    private final SwerveDriveKinematics kinematics;
    private final PIDController[] driveFeedback = new PIDController[4];
    private final PIDController[] turnFeedback = new PIDController[4];
    private final double[] lastModulePositionsRad = new double[]{0.0, 0.0, 0.0, 0.0};
    private final PhotonCameraWrapper pcw;
    private SimpleMotorFeedforward driveFeedforward;
    private Pose2d odometryPose = new Pose2d();
    private final SwerveDrivePoseEstimator poseEstimator;
    private Translation2d fieldVelocity = new Translation2d();
    private double lastGyroPosRad = 0.0;
    private boolean brakeMode = false;
    private DriveMode driveMode = DriveMode.NORMAL;
    private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();
    private double characterizationVoltage = 0.0;
    private Pose2d visionEstimate = new Pose2d();

    /**
     * Creates a new Drive.
     */
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
                 ModuleIO blModuleIO, ModuleIO brModuleIO, PhotonCameraWrapper pcw) {
        this.pcw = pcw;
        this.gyroIO = gyroIO;
        moduleIOs[0] = flModuleIO;
        moduleIOs[1] = frModuleIO;
        moduleIOs[2] = blModuleIO;
        moduleIOs[3] = brModuleIO;

        switch (Constants.getRobot()) {
            case ROBOT_2024S:
                maxLinearSpeed = Units.feetToMeters(16.3);
                wheelRadius = Units.inchesToMeters(2.0);
                trackWidthX = Units.inchesToMeters(25.0);
                trackWidthY = Units.inchesToMeters(25.0);

                driveKp.initDefault(0.1);
                driveKd.initDefault(0.0);
                driveKs.initDefault(0.12349);
                driveKv.initDefault(0.13477);

                turnKp.initDefault(10.0);
                turnKd.initDefault(0.0);

                break;
            case ROBOT_2024SIM:
                maxLinearSpeed = Units.feetToMeters(16.3);
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

        Vector<N3> modelStatesDeviation = VecBuilder.fill(0.1, 0.1, 0.1);
        Vector<N3> visionMeasurementDeviation = VecBuilder.fill(0.9, 0.9, 0.9);

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, odometryPose.getRotation(), getModulePositions(),
                new Pose2d(), modelStatesDeviation, visionMeasurementDeviation);

        visionEstimate = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

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
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + i,
                    moduleInputs[i]);
        }

        // Update objects based on TunableNumbers
        if (driveKp.hasChanged() || driveKd.hasChanged() || driveKs.hasChanged()
                || driveKv.hasChanged() || turnKp.hasChanged() || turnKd.hasChanged()) {
            driveFeedforward =
                    new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
            for (int i = 0; i < 4; i++) {
                driveFeedback[i].setP(driveKp.get());
                driveFeedback[i].setD(driveKd.get());
                turnFeedback[i].setP(turnKp.get());
                turnFeedback[i].setD(turnKd.get());
            }
        }

        // Update angle measurements
        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] =
                    new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
        }

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
        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                    (moduleInputs[i].drivePositionRad - lastModulePositionsRad[i])
                            * wheelRadius,
                    turnPositions[i]);
            lastModulePositionsRad[i] = moduleInputs[i].drivePositionRad;
        }

        ChassisSpeeds chassisStateDiff =
                kinematics.toChassisSpeeds(measuredStatesDiff);
        if (gyroInputs.connected) { // Use gyro for angular change when connected
            odometryPose =
                    odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
                            chassisStateDiff.vyMetersPerSecond,
                            gyroInputs.yawRad - lastGyroPosRad));

            // Since we already updated the odometry pose, we can just use the current pose for the odometry class
            poseEstimator.update(odometryPose.getRotation(), getModulePositions());
        } else { // Fall back to using angular velocity (disconnected or sim)
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
        ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
        fieldVelocity = new Translation2d(chassisState.vxMetersPerSecond,
                chassisState.vyMetersPerSecond).rotateBy(getRotation());

        // Update Vision
        Optional<EstimatedRobotPose> result =
                pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

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
        Logger.getInstance().recordOutput("SwerveModuleStates/Measured",
                measuredStates);

        // Log odometry pose
        Logger.getInstance().recordOutput("Odometry/Encoders/Robot", odometryPose);

        // Log raw vision pose
        Logger.getInstance().recordOutput("Odometry/Vision/Robot", visionEstimate);

        // Log Pose Estimator Pose
        Logger.getInstance().recordOutput("Odometry/FusedOdometry/Robot", poseEstimator.getEstimatedPosition());

        // Enable/disable brake mode
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
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void goToX() {
        driveMode = DriveMode.X;
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed;
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return maxAngularSpeed;
    }

    /**
     * Returns the current odometry pose.
     */
    public Pose2d getPose() {
        return odometryPose;
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        odometryPose = pose;
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return odometryPose.getRotation();
    }

    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    /**
     * Returns an array of module translations.
     */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
                new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
                new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
                new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)};
    }

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
     */
    public void runCharacterizationVolts(double volts) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = volts;
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < 4; i++) {
            driveVelocityAverage += moduleInputs[i].driveVelocityRadPerSec;
        }
        return driveVelocityAverage / 4.0;
    }

    private enum DriveMode {
        NORMAL, X, CHARACTERIZATION
    }
}
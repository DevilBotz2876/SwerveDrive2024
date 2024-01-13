// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.commands.*;
import bhs.devilbotz.commands.DriveWithJoysticks.JoystickMode;
import bhs.devilbotz.subsystems.*;
import bhs.devilbotz.utils.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static final Pose2d autoDriveTarget =
            new Pose2d(4.0, 2.0, new Rotation2d());
    // Choosers
    private final LoggedDashboardChooser<AutoRoutine> autoChooser =
            new LoggedDashboardChooser<>("Auto Routine");
    private final LoggedDashboardChooser<JoystickMode> joystickModeChooser =
            new LoggedDashboardChooser<>("Joystick Mode");
    private final LoggedDashboardChooser<Double> demoLinearSpeedLimitChooser =
            new LoggedDashboardChooser<>("Linear Speed Limit");
    private final LoggedDashboardChooser<Double> demoAngularSpeedLimitChooser =
            new LoggedDashboardChooser<>("Angular Speed Limit");
    // Wrappers
    private final PhotonCameraWrapper pcw = new PhotonCameraWrapper();
    // OI objects
    private final XboxController driverController = new XboxController(0);
    // Subsystems
    private Drive drive;
    private boolean isFieldRelative = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Check if flash should be burned
        SparkMAXBurnManager.update();

        // Instantiate active subsystems
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_2024S:
                    drive = new Drive(new GyroIONAVX(), new ModuleIOSparkMAX(0),
                            new ModuleIOSparkMAX(1), new ModuleIOSparkMAX(2),
                            new ModuleIOSparkMAX(3), pcw);
                    break;
                case ROBOT_2024SIM:
                    drive = new Drive(new GyroIO() {
                    }, new ModuleIOSim(),
                            new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), pcw);
                    break;
                default:
                    break;
            }
        }

        // Instantiate missing subsystems
        drive = drive != null ? drive
                : new Drive(new GyroIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        },
                new ModuleIO() {
                }, new ModuleIO() {
        }, pcw);

        // Set up auto routines
        autoChooser.addDefaultOption("Do Nothing",
                new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));

        autoChooser.addOption("Drive Characterization",
                new AutoRoutine(AutoPosition.ORIGIN,
                        new FeedForwardCharacterization(drive, true,
                                new FeedForwardCharacterization.FeedForwardCharacterizationData("drive"),
                                drive::runCharacterizationVolts,
                                drive::getCharacterizationVelocity)));


        // Set up choosers
        joystickModeChooser.addDefaultOption("Standard", JoystickMode.STANDARD);
        joystickModeChooser.addOption("Tank", JoystickMode.TANK);
        demoLinearSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
        demoLinearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        demoLinearSpeedLimitChooser.addDefaultOption("Medium-Fast Speed (50%)", 0.5);
        demoLinearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        demoLinearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
        demoAngularSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
        demoAngularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        demoAngularSpeedLimitChooser.addDefaultOption("Medium-Fast Speed (50%)", 0.5);
        demoAngularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        demoAngularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

        // Alert if in tuning mode
        if (Constants.tuningMode) {
            new Alert("Tuning mode active, expect decreased network performance.",
                    Alert.AlertType.INFO).set(true);
        }

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driving controls
        new Trigger(driverController::getStartButton)
                .or(new Trigger(driverController::getBackButton))
                .onTrue(new InstantCommand(() -> {
                    isFieldRelative = !isFieldRelative;
                    SmartDashboard.putBoolean("Field Relative", isFieldRelative);
                }).ignoringDisable(true));
        SmartDashboard.putBoolean("Field Relative", isFieldRelative);

        drive.setDefaultCommand(new DriveWithJoysticks(drive,
                () -> -driverController.getLeftY(), () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(), () -> !isFieldRelative,
                joystickModeChooser::get,
                demoLinearSpeedLimitChooser::get,
                demoAngularSpeedLimitChooser::get,
                null,
                () -> driverController.getRightTriggerAxis() > 0.8 && driverController.getLeftTriggerAxis() > 0.8));


        // Reset gyro command
        Command resetGyroCommand = new InstantCommand(() -> {
            drive.setPose(autoDriveTarget);
        }, drive).ignoringDisable(true);
        Command rumbleCommand = new StartEndCommand(
                () -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1),
                () -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        }.withTimeout(0.2);

        new Trigger(driverController::getRightBumper)
                .and(new Trigger(driverController::getLeftBumper))
                .onTrue(resetGyroCommand).onTrue(rumbleCommand);

        // TODO: same here
        // Run FeedForwardCharacterization command
        new Trigger(driverController::getXButton)
                .onTrue(new FeedForwardCharacterization(drive, true,
                        new FeedForwardCharacterization.FeedForwardCharacterizationData("drive"),
                        drive::runCharacterizationVolts,
                        drive::getCharacterizationVelocity));

        // axis 2 is the left bumper, when it is over 80% pressed, it will run the command
        // axis 3 is the right bumper, when it is over 80% pressed, it will run the command

        // Auto drive controls
        // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        // .whileTrue(new AutoDriveHard(drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        AutoRoutine routine = autoChooser.get();
        drive.setPose(routine.position.getPose());
        return routine.command;
    }

    public void rumble(double value) {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, value);
        driverController.setRumble(GenericHID.RumbleType.kRightRumble, value);
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, value);
    }

    public enum AutoPosition {
        ORIGIN, TARMAC_A, TARMAC_B, TARMAC_C, TARMAC_D, FENDER_A, FENDER_A_REVERSED, FENDER_B, FENDER_B_REVERSED;

        public Pose2d getPose() {
            switch (this) {
                case ORIGIN:
                    return new Pose2d();
                case TARMAC_A:
                    return FieldConstants.referenceA
                            .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.7));
                case TARMAC_B:
                    return FieldConstants.referenceB
                            .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.2));
                case TARMAC_C:
                    return FieldConstants.referenceC
                            .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1));
                case TARMAC_D:
                    return FieldConstants.referenceD
                            .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.7));
                case FENDER_A:
                    return FieldConstants.fenderA
                            .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
                case FENDER_A_REVERSED:
                    return FieldConstants.fenderA.transformBy(new Transform2d(
                            new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
                case FENDER_B:
                    return FieldConstants.fenderB
                            .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
                case FENDER_B_REVERSED:
                    return FieldConstants.fenderB.transformBy(new Transform2d(
                            new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
                default:
                    return new Pose2d();
            }
        }
    }

    private static class AutoRoutine {
        public final AutoPosition position;
        public final Command command;

        public AutoRoutine(AutoPosition position, Command command) {
            this.position = position;
            this.command = command;
        }
    }
}
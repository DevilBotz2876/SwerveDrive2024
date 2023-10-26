// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.utils.Alert;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private final Alert logReceiverQueueAlert =
            new Alert("Logging queue exceeded capacity, data will NOT be logged.",
                    Alert.AlertType.ERROR);
    private RobotContainer robotContainer;
    private Command autoCommand;
    private double autoStart;
    private boolean autoMessagePrinted;

    /**
     * Robot constructor, called by WPILib
     */
    public Robot() {
        super(Constants.loopPeriodSecs);
    }

    /**
     * Called once when the robot is first powered on
     */
    @Override
    public void robotInit() {
        Logger logger = Logger.getInstance();
        setUseTiming(Constants.getMode() != Constants.Mode.REPLAY);
        logger.recordMetadata("Robot", Constants.getRobot().toString());
        logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
        logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.getMode()) {

            case REAL:
                // USB drive is mounted at /media/sda1
                // TODO: Make sure this is the correct path and make it configurable
                logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
                logger.addDataReceiver(new NT4Publisher());
                LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kCTRE);
                break;

            case SIM:
                logger.addDataReceiver(new WPILOGWriter(""));
                logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                String path = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(path));
                logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
                break;
        }
        logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /**
     * Called periodically while the robot is powered on
     */
    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();

        // Log scheduled commands
        Logger.getInstance().recordOutput("ActiveCommands/Scheduler",
                NetworkTableInstance.getDefault()
                        .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
                        .getStringArray(new String[]{}));

        // Check logging fault
        logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

        // Print auto duration
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.printf("*** Auto finished in %.2f secs ***%n",
                            Timer.getFPGATimestamp() - autoStart);
                } else {
                    System.out
                            .printf("*** Auto cancelled in %.2f secs ***%n",
                                    Timer.getFPGATimestamp() - autoStart);
                }
                autoMessagePrinted = true;
            }
        }

        Threads.setCurrentThreadPriority(true, 10);
    }

    /**
     * Called once when the robot is disabled
     */
    @Override
    public void disabledInit() {
        robotContainer.rumble(1);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                robotContainer.rumble(0);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        }).start();
    }

    /**
     * Called periodically while the robot is disabled
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * Called once when autonomous mode is first enabled
     */
    @Override
    public void autonomousInit() {
        autoStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
        autoCommand = robotContainer.getAutonomousCommand();
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    /**
     * Called periodically while autonomous mode is enabled
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Called once when teleop mode is first enabled
     */
    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    /**
     * Called periodically while teleop mode is enabled
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * Called once when test mode is first enabled
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Called periodically while test mode is enabled
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * Called once when simulation mode is first enabled
     */
    @Override
    public void simulationInit() {
    }

    /**
     * Called periodically while simulation mode is enabled
     */
    @Override
    public void simulationPeriodic() {
    }
}

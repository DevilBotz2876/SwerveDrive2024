package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WheelPIDCalibration extends CommandBase {
    // This command will set all modules to 180 degrees and then 0 degrees every 1 second
    // This will allow us to tune the PID values for the modules
    // Use wpilib timers

    private final Drive drive;
    private Timer timer = new Timer();

    public WheelPIDCalibration(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setModuleAngle(0, 0);
        drive.setModuleAngle(1, 0);
        drive.setModuleAngle(2, 0);
        drive.setModuleAngle(3, 0);

        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() < 1) {
            drive.setModuleAngle(0, Units.degreesToRadians(180));
            drive.setModuleAngle(1, Units.degreesToRadians(180));
            drive.setModuleAngle(2, Units.degreesToRadians(180));
            drive.setModuleAngle(3, Units.degreesToRadians(180));
        } else if (timer.get() < 2) {
            drive.setModuleAngle(0, 0);
            drive.setModuleAngle(1, 0);
            drive.setModuleAngle(2, 0);
            drive.setModuleAngle(3, 0);
        } else {
            timer.reset();
        }

    }

    @Override
    public void end(boolean interrupted) {
        drive.setModuleAngle(0, 0);
        drive.setModuleAngle(1, 0);
        drive.setModuleAngle(2, 0);
        drive.setModuleAngle(3, 0);
    }
}

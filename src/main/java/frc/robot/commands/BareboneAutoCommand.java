package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class BareboneAutoCommand extends Command {
    DriveSubsystem driveSubsystem;

    public BareboneAutoCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       driveSubsystem.drive(1, -0.1);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class TeleopDriveCommand extends Command {
    private DriveSubsystem driveSubsystem;

    private Joystick controller;

    public TeleopDriveCommand(DriveSubsystem driveSubsystem, Joystick controller) {

        this.controller = controller;

        this.driveSubsystem = driveSubsystem;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(controller.getY(), controller.getY());
    }
}

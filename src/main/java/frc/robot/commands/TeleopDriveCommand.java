package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class TeleopDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;

    private final XboxController controller;

    public TeleopDriveCommand(DriveSubsystem driveSubsystem, XboxController controller) {

        this.controller = controller;

        this.driveSubsystem = driveSubsystem;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(controller.getLeftY(), controller.getLeftX());
    }
}

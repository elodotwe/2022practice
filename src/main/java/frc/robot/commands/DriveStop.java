package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveStop extends CommandBase {
    Drivetrain drivetrain;

    public DriveStop(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

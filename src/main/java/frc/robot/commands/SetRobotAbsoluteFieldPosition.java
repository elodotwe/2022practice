package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetRobotAbsoluteFieldPosition extends CommandBase {
    private Pose2d position;
    private Drivetrain drivetrain;

    public SetRobotAbsoluteFieldPosition(Drivetrain drivetrain, Pose2d position) {
        this.drivetrain = drivetrain;
        this.position = position;
    }

    @Override
    public void initialize() {
        drivetrain.resetOdometry(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

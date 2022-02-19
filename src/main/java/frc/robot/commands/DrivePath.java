package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class DrivePath extends SequentialCommandGroup {
    public final double kMaxSpeedMetersPerSecond = 3;
    public final double kMaxAccelerationMetersPerSecondSquared = 3;
    public final double kRamseteB = 2;
    public final double kRamseteZeta = 0.7;

    public DrivePath(Drivetrain drivetrain, String pathFileName) {
        Trajectory trajectory;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathFileName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathFileName, ex.getStackTrace());
            return;
        }
        

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        trajectory,
                        drivetrain.odometry::getPoseMeters,
                        new RamseteController(kRamseteB, kRamseteZeta),
                        drivetrain.motorFeedforward,
                        drivetrain.kinematics,
                        drivetrain::getWheelSpeeds,
                        new PIDController(drivetrain.kPDriveVel, 0, 0),
                        new PIDController(drivetrain.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        drivetrain::tankDriveVolts,
                        drivetrain);

        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        Command ramsete = ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));

        // There's probably a better way to do this but I don't know how--
        // I want to wrap RamseteCommand in a nicer-to-use command, essentially
        // wrapping a command in a command.
        // I thought of delegating all of the Command functions to the Ramsete command contained within here,
        // but that's a lot more messy than simply having a Sequential group with only one command lol
        addCommands(ramsete);
    }
}

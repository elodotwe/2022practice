package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class DrivePath extends SequentialCommandGroup {
    public final double kRamseteB = 2;
    public final double kRamseteZeta = 0.7;

    public DrivePath(Drivetrain drivetrain, String pathFileName) {
        this(drivetrain, pathFileName, true);
    }

    public DrivePath(Drivetrain drivetrain, String pathFileName, boolean resetOdometry) {
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

        if (resetOdometry) {
            addCommands(new SetRobotAbsoluteFieldPosition(drivetrain, trajectory.getInitialPose()));
        }

        addCommands(ramseteCommand, new DriveStop(drivetrain));
    }
}

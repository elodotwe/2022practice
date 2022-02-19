package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class DrivePath extends SequentialCommandGroup {
    public final double kMaxSpeedMetersPerSecond = 3;
    public final double kMaxAccelerationMetersPerSecondSquared = 3;
    public final double kRamseteB = 2;
    public final double kRamseteZeta = 0.7;

    public DrivePath(Drivetrain drivetrain) {
        // Create a voltage constraint to ensure we don't accelerate too fast

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                drivetrain.motorFeedforward,
                drivetrain.kinematics,
                10);

        // Create config for trajectory

        TrajectoryConfig config = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(drivetrain.kinematics)
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.

        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(3, 3), new Translation2d(5, 1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(6, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
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
        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

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

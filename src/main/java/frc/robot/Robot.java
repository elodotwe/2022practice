package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Teleop;
import frc.robot.simulation.Simulation;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private Joysticks joysticks;
  private Drivetrain drivetrain;

  private Teleop teleop;

  private Simulation simulation;

  private AutonChooser autonChooser;
  private Command autonCommand = null;

  @Override
  public void robotInit() {
    joysticks = new Joysticks();
    drivetrain = new Drivetrain();
    teleop = new Teleop(drivetrain, joysticks.getDriveForewardPower(), joysticks.getDriveRotationPower());
    autonChooser = new AutonChooser(drivetrain);
  }

  @Override
  public void teleopInit() {
    teleop.schedule();
  }

  @Override
  public void teleopExit() {
    teleop.cancel();
  }

  @Override
  public void autonomousInit() {
    autonCommand = autonChooser.getAutonCommand();
    if (autonCommand != null) {
      autonCommand.schedule();
    } else {
      System.out.println("WARNING: no autonomous because AutonChooser returned null");
    }
  }

  @Override
  public void autonomousExit() {
    if (autonCommand != null) {
      autonCommand.cancel();
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivetrain.periodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationInit() {
      simulation = new Simulation(drivetrain);
  }

  @Override
  public void simulationPeriodic() {
      simulation.simulationPeriodic();
  }
}

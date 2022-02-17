package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Teleop;
import frc.robot.simulation.Simulation;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private Joysticks joysticks = new Joysticks();
  private Drivetrain drivetrain = new Drivetrain();

  private Teleop teleop = new Teleop(drivetrain, joysticks.getDriveForewardPower(), joysticks.getDriveRotationPower());

  private Simulation simulation;

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
  }

  @Override
  public void autonomousExit() {
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

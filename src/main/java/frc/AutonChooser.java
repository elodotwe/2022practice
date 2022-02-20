package frc;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePath;
import frc.robot.subsystems.Drivetrain;

public class AutonChooser {
    private class AutonChoice {
        final String name;
        final Command command;

        AutonChoice(String name, Command command) {
            this.name = name;
            this.command = command;
        }
    }

    private SendableChooser<Command> autonChooser = new SendableChooser<>();

    public AutonChooser(Drivetrain drivetrain) {
        AutonChoice[] choices = {
            new AutonChoice("unnamed path", new DrivePath(drivetrain, "paths/Unnamed.wpilib.json")),
            new AutonChoice("long shot", new DrivePath(drivetrain, "paths/collect_near.wpilib.json").andThen(new DrivePath(drivetrain, "paths/reverse_to_long_shot.wpilib.json", false), new DrivePath(drivetrain, "paths/long_shot.wpilib.json", false)))
        };

        boolean first = true;
        for (AutonChoice autonChoice : choices) {
            if (first) {
                first = false;
                autonChooser.setDefaultOption(autonChoice.name, autonChoice.command);
            } else {
                autonChooser.addOption(autonChoice.name, autonChoice.command);
            }
        }

        SmartDashboard.putData(autonChooser);
    }

    public Command getAutonCommand() {
        return autonChooser.getSelected();
    }
}

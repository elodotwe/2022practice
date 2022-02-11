package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Joysticks {
    private Joystick chassisJoystick = new Joystick(0);

    public double getDriveForewardPower() {
        return chassisJoystick.getY();
    }

    public double getDriveRotationPower() {
        return chassisJoystick.getX();
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Joysticks {
    private Joystick chassisJoystick = new Joystick(0);

    public double getDriveForewardPower() {
        // Left stick Y on logitech joypad, inverted because we want 1.0 to be forward
        return -chassisJoystick.getRawAxis(1);
    }

    public double getDriveRotationPower() {
        // Right stick X on logitech joypad
        return chassisJoystick.getRawAxis(4);
    }
}

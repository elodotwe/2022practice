package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;

public class Joysticks {
    private Joystick chassisJoystick = new Joystick(0);

    public Supplier<Double> getDriveForewardPower() {
        // Left stick Y on logitech joypad, inverted because we want 1.0 to be forward
        return () -> -chassisJoystick.getRawAxis(1);
    }

    public Supplier<Double> getDriveRotationPower() {
        // Right stick X on logitech joypad
        return () -> chassisJoystick.getRawAxis(4);
    }
}

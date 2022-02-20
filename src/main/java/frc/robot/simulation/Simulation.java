package frc.robot.simulation;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.subsystems.Drivetrain;

public class Simulation {
    private DifferentialDrivetrainSim drivetrainSim;
    private Drivetrain drivetrain;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private ADXRS450_GyroSim gyroSim;

    // Do not remove this variable. We need to retain the return value from `registerResetCallback()`, otherwise the callbacks don't happen.
    private CallbackStore callbackStore;

    private void initializeDrivetrainSim() {
        drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kEightInch, null);
    }

    public Simulation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        initializeDrivetrainSim();
        leftEncoderSim = new EncoderSim(drivetrain.leftEncoder);
        rightEncoderSim = new EncoderSim(drivetrain.rightEncoder);
        gyroSim = new ADXRS450_GyroSim(drivetrain.gyro);

        callbackStore = leftEncoderSim.registerResetCallback(new NotifyCallback() {
            @Override
            public void callback(String name, HALValue value) {
                // We reset the encoders only at the beginning of an autonomous routine-- when we know what our starting position should be.
                // Toss out all knowledge of our simulated physics; if we're resetting the encoder we want to reset our physics too.
                initializeDrivetrainSim();
                // Calling simulationPeriodic manually means the simulated encoders and gyro all get updated immediately-- so odometry doesn't
                // get a chance to pick up any leftover crap from prior sessions.
                simulationPeriodic();
            }
        }, false);
    }

    public void simulationPeriodic() {
        drivetrainSim.setInputs(drivetrain.leftMotor.get() * RobotController.getInputVoltage(), drivetrain.rightMotor.get() * RobotController.getInputVoltage());
        drivetrainSim.update(0.02);

        leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());

        gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees());
    }
}

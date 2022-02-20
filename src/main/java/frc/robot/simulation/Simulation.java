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
                initializeDrivetrainSim();
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

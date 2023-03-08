package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 *
 */
public class Arm extends SubsystemBase {

    private CANSparkMax armMotorController;

    public Arm() {
        armMotorController = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}


package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Hand extends SubsystemBase {

    private Servo handServo;

    public Hand() {
        handServo = new Servo(Constants.HAND_SERVO_ID);
        addChild("HandServo", handServo);
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


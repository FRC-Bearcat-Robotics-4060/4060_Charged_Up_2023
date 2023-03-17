package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushed);
    private CANSparkMax rollerMotor = new CANSparkMax(Constants.ROLLER_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushed);

    private RelativeEncoder wristEncoder;
    private SparkMaxPIDController wristPIDController;
 
    // add abstract methods
    public WristSubsystem() {

        wristMotor.restoreFactoryDefaults();
        wristMotor.setInverted(false);
        wristMotor.setSmartCurrentLimit(10);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // wristMotor.setOpenLoopRampRate(1.0);
        // wristMotor.setClosedLoopRampRate(1.0);
        // Set closed loop to position mode
        wristEncoder = wristMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        wristEncoder.setPositionConversionFactor(360.0);

        wristPIDController = wristMotor.getPIDController();
        wristPIDController.setFeedbackDevice(wristEncoder);
        wristPIDController.setOutputRange(-0.35, 0.35);
        wristPIDController.setP(Constants.WRIST_P);
        wristPIDController.setI(Constants.WRIST_I);
        wristPIDController.setD(Constants.WRIST_D);

        wristMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.WRIST_DEPLOY_POS - 10f);
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.WRIST_PARK_POS + 10f);
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Set encoder position to zero. Might be needed if we reboot.
        wristEncoder.setPosition(0.0);

        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setInverted(false);
        rollerMotor.setSmartCurrentLimit(10);
        rollerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void zeroPosition() {
        wristEncoder.setPosition(0.0);
    }

    public void runRollerIn(double speed) {
        rollerMotor.set(speed);
    }

    public void runRollerOut(double speed) {
        rollerMotor.set(-speed);
    }

    public void feedIn() {
        rollerMotor.set(Constants.ROLLER_IN_SPEED);
    }

    public void feedOut() {
        rollerMotor.set(Constants.ROLLER_OUT_SPEED);
    }

    public void stopRoller() {
        rollerMotor.set(0.0);
    }

    public void setWristPosition(double position) {
        wristPIDController.setReference(position, ControlType.kPosition);
    }

    // Add function that sets wrist position to 150 degrees
    public void deploy() {
        setWristPosition(Constants.WRIST_DEPLOY_POS);
    }

    // Add function called park that sets wrist position to 10 degrees
    public void park() {
        setWristPosition(Constants.WRIST_PARK_POS);
    }

    public void level1() {
        setWristPosition(Constants.WRIST_LEVEL1_POS);
    }

    public void level2() {
        setWristPosition(Constants.WRIST_LEVEL2_POS);
    }

    public void level3() {
        setWristPosition(Constants.WRIST_LEVEL3_POS);
    }

}


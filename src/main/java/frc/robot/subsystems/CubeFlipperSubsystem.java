// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import static frc.robot.Constants.*;

public class CubeFlipperSubsystem extends SubsystemBase {

    Servo flipperServo = new Servo(CUBE_FLIPPER_SERVO_CHANNEL);

    public CubeFlipperSubsystem() {
        park();
    }

    public void park() {
        flipperServo.set(CUBE_FLIPPER_SERVO_PARK_POS);
    }

    public void eject() {
        flipperServo.set(CUBE_FLIPPER_SERVO_EJECT_POS);
    }
}

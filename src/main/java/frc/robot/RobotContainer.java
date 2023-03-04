// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private final Joystick m_controller = new Joystick(0);
    private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);

    
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // public final Hand m_hand = new Hand();
    // public final Wrist m_wrist = new Wrist();
    // public final Arm m_arm = new Arm();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
                () -> poseEstimator.getCurrentPose().getRotation(),
                () -> modifyAxis(-m_controller.getRawAxis(1))
                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(m_controller.getRawAxis(0))
                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyTwistAxis(m_controller.getTwist())
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        // Configure the button bindings
        configureButtonBindings();

        m_chooser.setDefaultOption("None", new InstantCommand());
        m_chooser.addOption("Drive Forward 2.0m",
                new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator,
                        poseEstimator.getCurrentPose().transformBy(
                                new Transform2d(new Translation2d(2., 0.), new Rotation2d()))));
        m_chooser.addOption("Drive Back 0.75m",
                new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator,
                        poseEstimator.getCurrentPose().transformBy(
                                new Transform2d(new Translation2d(-0.75, 0.), new Rotation2d()))));
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        //https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        // TODO: Do we have a subsystem this should require?
        new JoystickButton(m_controller, 3)
            .onTrue(Commands.runOnce(m_drivetrainSubsystem::zeroGyroscope));

        // new JoystickButton(m_controller, 4)
        //     .onTrue(new PrintPositionCommand(poseEstimator));

        // Borrowed from https://github.com/STMARobotics/frc-7028-2023/blob/main/src/main/java/frc/robot/RobotContainer.java
        // Drive to cone node to the left of tag 1, then just shoot

        // new JoystickButton(m_controller, 11).whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem,
        //         poseEstimator, poseEstimator.getCurrentPose().transformBy(
        //                 new Transform2d(new Translation2d(1., 0.), new Rotation2d()))));
        // new JoystickButton(m_controller, 5)
        //     .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, 
        //         new Pose2d(14.15, 1.07, Rotation2d.fromDegrees(-5.97))));

        // new JoystickButton(m_controller, 6)
        //     .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, 
        //         new Pose2d(13.66, 2.56, Rotation2d.fromDegrees(-4.97))));

        // new JoystickButton(m_controller, 7)
        //     .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator,     
        //         new Pose2d(14.40, 4.11, Rotation2d.fromDegrees(5.84))));

        // new JoystickButton(m_controller, 8)
        //     .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, 
        //         new Pose2d(12.65, 2.46, Rotation2d.fromDegrees(-180.00))));     
            
        // controller.rightTrigger().whileTrue(new DriveToPoseCommand(
        //     drivetrainSubsystem, poseEstimator::getCurrentPose, new Pose2d(14.59, 1.67, Rotation2d.fromDegrees(0.0)))
        //         .andThen(new JustShootCommand(0.4064, 1.05, 34.5, elevatorSubsystem, wristSubsystem, shooterSubsystem)));

        // Temporary commands to ensure that all commands and subsystems are able to compile
        // new JoystickButton(m_controller, 9)
        //     .whileTrue(new Arm_Move(1000, m_arm));

        // new JoystickButton(m_controller, 10)
        //     .whileTrue(new Wrist_Move(90, m_wrist));

        // new JoystickButton(m_controller, 11)
        //     .whileTrue(new Hand_Grip(90, m_hand));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value * value, value);

        return value;
    }

    private static double modifyTwistAxis(double value) {
        // Deadband
        value = deadband(value, 0.2);

        // Square the axis
        value = Math.copySign(value * value * value * value, value);

        return value;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static final double center_DriverOverRamp_inches = 220.0;
    private static final double center_DriveToRamp_inches = 80.0;
    private static final double left_StrafeToRamp = -70.0;
    private static final double right_StrafeToRamp = -left_StrafeToRamp;
    private static final double side_DrivePastRamp = 200.00;

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private final Joystick m_controller = new Joystick(0);
    private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);

    private final CubeFlipperSubsystem m_cubeFlipperSubsystem = new CubeFlipperSubsystem();
    
    private final WristSubsystem m_wristSubsystem = new WristSubsystem();

    private CommandBase EjectCubeCommand() {
        return Commands.runOnce(m_cubeFlipperSubsystem::eject, m_cubeFlipperSubsystem)
            .andThen(Commands.waitSeconds(Constants.CUBE_FLIPPER_EJECT_DELAY_S))
            .andThen(Commands.runOnce(m_cubeFlipperSubsystem::park, m_cubeFlipperSubsystem));
    }

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    private Pose2d startingPose;

    Pose2d translate_pose_meters(Pose2d start, double x_meters, double y_meters)
    {
        return start.transformBy(new Transform2d(new Translation2d(x_meters, y_meters), new Rotation2d()));
    }

    Pose2d translate_pose_inches(Pose2d start, double x_inches, double y_inches)
    {
        return translate_pose_meters(start, Units.inchesToMeters(x_inches), Units.inchesToMeters(y_inches));
    }

    DriveToPoseCommand GoToInches(double x, double y)
    {
        return new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, translate_pose_inches(startingPose, x, y), false);
    }

    DriveToPoseCommand GoToInches_ExitOnRoll(double x, double y)
    {
        return new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, translate_pose_inches(startingPose, x, y), true);
    }

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

        startingPose = poseEstimator.getCurrentPose();

        m_chooser.setDefaultOption("None", new InstantCommand());

        m_chooser.addOption("Eject Cube", EjectCubeCommand());
        
        m_chooser.addOption("Center: Charge",
                EjectCubeCommand()
                .andThen(GoToInches_ExitOnRoll(center_DriveToRamp_inches, 0.0))
                .andThen(new AutoBalanceCommand(m_drivetrainSubsystem)));
        
        m_chooser.addOption("Center: Drive Out + Charge",
                EjectCubeCommand()
                .andThen(GoToInches_ExitOnRoll(center_DriveToRamp_inches, 0.0))
                // Split move into two to avoid doing it too fast
                // .andThen(GoToInches(center_DriverOverRamp_inches * 0.65, 0.0))
                .andThen(GoToInches(center_DriverOverRamp_inches, 0.0))
                .andThen(GoToInches_ExitOnRoll(center_DriveToRamp_inches, 0.0))
                .andThen(new AutoBalanceCommand(m_drivetrainSubsystem)));

        m_chooser.addOption("Left: Drive Out",
                EjectCubeCommand()
                .andThen(GoToInches(side_DrivePastRamp, 0)));

        m_chooser.addOption("Left: Drive Out + Charge",
                EjectCubeCommand()
                .andThen(GoToInches(side_DrivePastRamp, 0))
                .andThen(GoToInches(side_DrivePastRamp, left_StrafeToRamp))
                .andThen(GoToInches_ExitOnRoll(center_DriveToRamp_inches, left_StrafeToRamp))
                .andThen(new AutoBalanceCommand(m_drivetrainSubsystem)));

        m_chooser.addOption("Right: Drive Out",
                EjectCubeCommand()
                .andThen(GoToInches(side_DrivePastRamp, 0)));

        m_chooser.addOption("Right: Drive Out + Charge",
                EjectCubeCommand()
                .andThen(GoToInches(side_DrivePastRamp, 0))
                .andThen(GoToInches(side_DrivePastRamp, right_StrafeToRamp))
                .andThen(GoToInches_ExitOnRoll(center_DriveToRamp_inches, right_StrafeToRamp))
                .andThen(new AutoBalanceCommand(m_drivetrainSubsystem)));

        SmartDashboard.putData("Auto choices", m_chooser);

        
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        new JoystickButton(m_controller, 3)
            .onTrue(Commands.runOnce(m_drivetrainSubsystem::zeroGyroscope, m_drivetrainSubsystem));

        new JoystickButton(m_controller, 4)
            .onTrue(new PrintPositionCommand(poseEstimator));

        // Borrowed from https://github.com/STMARobotics/frc-7028-2023/blob/main/src/main/java/frc/robot/RobotContainer.java
        // Drive to cone node to the left of tag 1, then just shoot

        /*
        new JoystickButton(m_controller, 5)
            .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, 
                new Pose2d(14.15, 1.07, Rotation2d.fromDegrees(-5.97))));

        new JoystickButton(m_controller, 6)
            .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, 
                new Pose2d(13.66, 2.56, Rotation2d.fromDegrees(-4.97))));

        new JoystickButton(m_controller, 7)
            .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator,     
                new Pose2d(14.40, 4.11, Rotation2d.fromDegrees(5.84))));

        new JoystickButton(m_controller, 8)
            .whileTrue(new DriveToPoseCommand(m_drivetrainSubsystem, poseEstimator, 
                new Pose2d(12.65, 2.46, Rotation2d.fromDegrees(-180.00))));     
        */

        // Debug commands which move to various points with button presses, relative to the starting position.
        // new JoystickButton(m_controller, 1).onTrue(GoToInches(0, 0));
        // new JoystickButton(m_controller, 7).onTrue(GoToInches(48, 0));
        // new JoystickButton(m_controller, 8).onTrue(GoToInches(24, 0));
        // new JoystickButton(m_controller, 9).onTrue(GoToInches(48, 24));
        // new JoystickButton(m_controller, 10).onTrue(GoToInches(24, 24));
        // new JoystickButton(m_controller, 11).onTrue(GoToInches(48, 48));
        // new JoystickButton(m_controller, 12).onTrue(GoToInches(24, 48));

        new JoystickButton(m_controller, 2).whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem));
        new JoystickButton(m_controller, 5).whileTrue(new AlignToCubeChannelCommand(m_drivetrainSubsystem, poseEstimator));

        new JoystickButton(m_controller, 6).onTrue(Commands.runOnce(m_cubeFlipperSubsystem::eject, m_cubeFlipperSubsystem));
        new JoystickButton(m_controller, 4).onTrue(Commands.runOnce(m_cubeFlipperSubsystem::park, m_cubeFlipperSubsystem));

        // controller.rightTrigger().whileTrue(new DriveToPoseCommand(
        //     drivetrainSubsystem, poseEstimator::getCurrentPose, new Pose2d(14.59, 1.67, Rotation2d.fromDegrees(0.0)))
        //         .andThen(new JustShootCommand(0.4064, 1.05, 34.5, elevatorSubsystem, wristSubsystem, shooterSubsystem)));

        new JoystickButton(m_controller, 11).onTrue(Commands.runOnce(m_wristSubsystem::park, m_wristSubsystem));
        // button 7 to deploy
        new JoystickButton(m_controller, 7).onTrue(Commands.runOnce(m_wristSubsystem::deploy, m_wristSubsystem));
        // button 12 to level1
        new JoystickButton(m_controller, 12).onTrue(Commands.runOnce(m_wristSubsystem::level1, m_wristSubsystem));
        // 10 to level2
        new JoystickButton(m_controller, 10).onTrue(Commands.runOnce(m_wristSubsystem::level2, m_wristSubsystem));
        // 8 to level3
        new JoystickButton(m_controller, 8).onTrue(Commands.runOnce(m_wristSubsystem::level3, m_wristSubsystem));
        
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

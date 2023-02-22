// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    // Limelight vision based pose estimation from AprilTags
    private final Field2d m_fieldSim = new Field2d();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        m_robotContainer = new RobotContainer();

        // NOT REQUIRED, since the PhotonVision library does this on its own!
        //CameraServer.startAutomaticCapture(new HttpCamera("limelight",
        //        "http://10.40.60.12:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer));
        //SmartDashboard.putData("Field", m_fieldSim);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        //UpdateBotPose();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}


    private void UpdateBotPose() {
        Double[] defaultPos = new Double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Double[] bp = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                .getDoubleArray(defaultPos);
        if (bp.length > 0) {
            // bp: [0:TX, 1:TY, 2:TZ, 3:RX, 4:RY, 5:RZ]
            // Coordinates are 0,0 at the center of the field, and are in meters.
            // Field2d expects 0,0 coordinates at the bottom left, from the judging table's
            // perspective
            double posX = bp[0] + Constants.FieldConstants.length / 2;
            double posY = bp[1] + Constants.FieldConstants.width / 2;
            Pose2d botPose2d =
                    new Pose2d(new Translation2d(posX, posY), Rotation2d.fromDegrees(bp[5]));
            m_fieldSim.setRobotPose(botPose2d);

            // The latest field image available in ShuffleBoard is from 2022, however it may be
            // possible
            // to insert one into the JAR:
            // Shuffleboard-2023.1.1-winx64\edu\wpi\first\shuffleboard\plugin\base\widget\field
        }
    }
}


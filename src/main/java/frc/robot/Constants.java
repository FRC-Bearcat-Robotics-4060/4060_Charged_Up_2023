// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.473075;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.473075;

    // public static final int DRIVETRAIN_PIGEON_ID = 0; // Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(230.416);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(222.9);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(38.1);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(104.0);

    public static final int ARM_MOTOR_ID = 9;
    public static final int WRIST_SERVO_ID = 0;
    public static final int HAND_SERVO_ID = 1;
    
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
    );

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }

    public static VisionCamera[] VisionCameras = new VisionCamera[] {
        /*new VisionCamera(
            CamType.LimeLight, 
            "Limelight", 
            "limelight", 
            new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d()),
            null
        ),*/
        // new VisionCamera(
        //     CamType.PhotonVision, 
        //     "PV1", 
        //     "pv1_onboard", 
        //     new Transform3d(new Translation3d(-0.7, 0.0, -0.5), new Rotation3d()),
        //     new PhotonCamera("pv1_onboard")
        // ),
      /*   new VisionCamera(
            CamType.PhotonVision, 
            "PV2", 
            "pv2_onboard", 
            new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d(0,0,Units.degreesToRadians(180))),
            new PhotonCamera("pv2_onboard")
        ), */
    };
}

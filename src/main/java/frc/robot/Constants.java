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

    // Servo assignments
    public static final int CUBE_FLIPPER_SERVO_CHANNEL = 3;
    public static final double CUBE_FLIPPER_SERVO_PARK_POS = 0.0;
    public static final double CUBE_FLIPPER_SERVO_EJECT_POS = 1.0;
    public static final double CUBE_FLIPPER_EJECT_DELAY_S = 2.0;

    public static final int WRIST_MOTOR_CAN_ID = 14;
    public static final int ROLLER_MOTOR_CAN_ID = 15;

    public static final double ROLLER_SPEED_DEPLOYED = 0.60;
    public static final double ROLLER_SPEED_LEVEL1 = 0.65;
    public static final double ROLLER_SPEED_LEVEL2 = 0.85;
    public static final double ROLLER_SPEED_LEVEL3 = 1.0;

    public static final float WRIST_PARK_POS = 20.0f;
    public static final float WRIST_LEVEL1_POS = 20.0f;
    public static final float WRIST_LEVEL2_POS = 20.0f;
    // public static final float WRIST_LEVEL2_POS = 30.0f;
    // public static final float WRIST_LEVEL2_POS = 30.0f;
    // public static final float WRIST_LEVEL2_POS = 30.0f;
    public static final float WRIST_LEVEL3_POS = 20.0f;
    public static final float WRIST_DEPLOY_POS = 150.0f;

    public static final double WRIST_P = 0.035;
    public static final double WRIST_I = 0.00006;
    public static final double WRIST_D = 0.01;

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

    public static final double DRIVE_TRANSLATION_RAMP_TIME = 1.0 / 3.0;
    public static final double DRIVE_ROTATION_RAMP_TIME = 1.0 / 5.0;
    
    // public static final int DRIVETRAIN_PIGEON_ID = 0; // Set Pigeon ID

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(139.0);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(106.35);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(164.7);

    // This motor is probably not Loctited...but we can't get the magnet out.
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(15.9);
    
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
        public static final double[] CubeYChannels = { 
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(174.19)
        };
    }

    // Used in AlignToCubeChannelCommnd
    // It might be more useful to have total robot length and with values
    public static final double BumperThickness = Units.inchesToMeters(3.5);

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
        //     new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d()),
        //     new PhotonCamera("pv1_onboard")
        // ),
        // new VisionCamera(
        //     CamType.PhotonVision, 
        //     "PV2", 
        //     "pv2_onboard", 
        //     new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d(0,0,Units.degreesToRadians(180))),
        //     new PhotonCamera("pv2_onboard")
        // ),
    };
}

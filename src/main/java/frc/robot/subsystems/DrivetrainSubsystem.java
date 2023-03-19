// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import java.lang.Thread;

import static frc.robot.Constants.*;
import java.util.Arrays;
//import java.util.function.Supplier;
//import java.util.stream.IntStream;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during
     * initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double NEO_FREE_RPM = 5676.0;
    public static final double FALCON500_FREE_RPM = 6380.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
            NEO_FREE_RPM / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                    * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this
    // with a measured amount.
    // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
    //         MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
    //                 DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double MAX_ROTATIONS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
            MAX_ROTATIONS_PER_SECOND * 2.0 * Math.PI;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a
    // NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating the robot
    // counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.

    // Uncomment if you are using a Pigeon
    // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

    // Uncomment if you are using a NavX
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final SwerveModule[] swerveModules;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Add a slight delay to make sure that all NEOs are up before we try to configure them.
        try{
            Thread.sleep(5000);
        }
        catch (Exception e)
        {}

        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        // Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        // Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and
        // the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
        // Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

        MkModuleConfiguration module_config = new MkModuleConfiguration();
        module_config.setDriveCurrentLimit(40.0);
        module_config.setSteerCurrentLimit(20.0);
        module_config.setSteerPID(1.0, 0.0, 0.1); // From getDefaultSteerNEO

        // By default we will use Falcon 500s in standard configuration. But if you use a
        // different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        m_frontLeftModule = new MkSwerveModuleBuilder(module_config)
                .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0))
                .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET).build();

        m_frontRightModule = new MkSwerveModuleBuilder(module_config)
                .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0))
                .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET).build();


        m_backLeftModule = new MkSwerveModuleBuilder(module_config)
                .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0))
                .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET).build();

        m_backRightModule = new MkSwerveModuleBuilder(module_config)
                .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0))
                .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET).build();

        swerveModules = new SwerveModule[] {m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule};
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
     * currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        // FIXME Remove if you are not using a Pigeon
        // m_pigeon.setFusedHeading(0.0);

        // FIXME Uncomment if you are using a NavX
        m_navx.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        // FIXME Remove if you are using a Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

        // FIXME Uncomment if you are using a NavX
        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public float getPitch() {
        return m_navx.getPitch();
    }

    public float getRoll() {
        float RollAngleDegrees = m_navx.getRoll();
        SmartDashboard.putNumber("autoBalance.Roll", RollAngleDegrees);
        return -RollAngleDegrees;
    }

    // Used to drive in a field-oriented X direction, as part of an auto-balance routine.
    public void drive_x_mps(double x_meters_per_second){
        SmartDashboard.putNumber("autoBalance.x_velocity", x_meters_per_second);

        double sign = x_meters_per_second >= 0.0 ? 1.0 : -1.0;

        double clamped = x_meters_per_second;
        if (clamped != 0) {
            clamped = sign * MathUtil.clamp(Math.abs(x_meters_per_second), 0.01, 0.75);
        }

        drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            clamped, 0.0,
            0.0, getGyroscopeRotation()));
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
    }
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(
                states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        m_frontRightModule.set(
                states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        m_backLeftModule.set(
                states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        m_backRightModule.set(
                states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    
}

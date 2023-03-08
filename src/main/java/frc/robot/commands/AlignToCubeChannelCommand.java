package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command to drive to the closest Cube Dropoff point.
 */
public class AlignToCubeChannelCommand extends CommandBase {

    private static final double TRANSLATION_TOLERANCE = 0.1;
    private static final double THETA_TOLERANCE = Units.degreesToRadians(0.5);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
            new TrapezoidProfile.Constraints(1, 1);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
            new TrapezoidProfile.Constraints(1, 1);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
            new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController =
            new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController =
            new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final Pose2d goalPose;

  public AlignToCubeChannelCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    // Where are we?
    var currentPose = poseProvider.get();

    // Determine which direction to face. What side are we on?
    Rotation2d direction = Rotation2d.fromDegrees(0); // Default facing RED side
    if (DriverStation.isFMSAttached() || DriverStation.isTest() == false){
        // This is a real game, so we will trust the FMS
        var alliance = DriverStation.getAlliance();
        if (alliance == DriverStation.Alliance.Blue){
            direction = Rotation2d.fromDegrees(-180.00); //facing BLUE side
        }
    } else {
        // We seem to be in test mode. Guess which side we are on based on the current pose estimation:
        if ( currentPose.getX() < Units.inchesToMeters(325.5)) {
            direction = Rotation2d.fromDegrees(-180.00); //facing BLUE side
        }
    }
    
    // what Y channel are we closest to?
    Double targetY = findClosestValue(Constants.FieldConstants.CubeYChannels, currentPose.getY());

    Double halfRobotThickness = ((Constants.BumperThickness * 2) + Constants.DRIVETRAIN_WHEELBASE_METERS) / 2;
    Double targetX = Units.inchesToMeters(596.6) - halfRobotThickness; //Default Red dropff X Position.

    if (direction == Rotation2d.fromDegrees(0.00)){
        // the red wall is at X: 596.6 inches away from 0,0 (corner by blue driverstation)
        // the center of the robot will be that value MINUS half of the robot length
        // ALREADY set as a default above
    } else {
        // the blue wall is at X: 53.854 inches away from 0,0 (corner by blue driverstation)
        // the center of the robot will be that value PLUS half of the robot length
        targetX = Units.inchesToMeters(53.854) + halfRobotThickness; //Default Red dropff X Position.

    }
    this.goalPose = new Pose2d(targetX, targetY, direction);

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

    @Override
    public void initialize() {
        var robotPose = poseProvider.get();
        thetaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());

        thetaController.setTolerance(THETA_TOLERANCE);
        xController.setTolerance(TRANSLATION_TOLERANCE);
        yController.setTolerance(TRANSLATION_TOLERANCE);

        thetaController.setGoal(goalPose.getRotation().getRadians());
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
    }

    @Override
    public void execute() {
        var robotPose = poseProvider.get();

        // Drive to the goal
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
        if (thetaController.atGoal()) {
            omegaSpeed = 0;
        }

        // board.putNumber("xSpeed", xSpeed);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
        drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
                robotPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }


    public static double findClosestValue(double[] arr, double target) {
        double closest = arr[0];
        double minDistance = Math.abs(target - closest);

        for (int i = 1; i < arr.length; i++) {
            double currentDistance = Math.abs(target - arr[i]);
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closest = arr[i];
            }
        }

        return closest;
    }
}

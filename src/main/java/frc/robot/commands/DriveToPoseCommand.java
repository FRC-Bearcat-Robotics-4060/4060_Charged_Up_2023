package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {
  
  private static final double TRANSLATION_TOLERANCE = Units.inchesToMeters(1.0);
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

  private final ProfiledPIDController xController = new ProfiledPIDController(10, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(10, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(2, 0.5, 0, OMEGA_CONSTRAINTS);

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;

  private Boolean exitOnRoll = false;
  private int rollExceededCount = 0;

  public DriveToPoseCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        Boolean exitIfRollEncountered) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.exitOnRoll = exitIfRollEncountered;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("DTP: Init");
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

    //board.putNumber("xSpeed", xSpeed);
    //SmartDashboard.putNumber("ySpeed", ySpeed);
    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {

    final double rollThreshold = 10.0;

    Boolean rolled = this.exitOnRoll && Math.abs(drivetrainSubsystem.getRoll()) >= rollThreshold;
    if (rolled)
    {
        rollExceededCount++;
        rolled = rollExceededCount >= 10;
    }
    else {
        rollExceededCount = 0;
    }
    Boolean atGoal = xController.atGoal() && yController.atGoal() && thetaController.atGoal();

    return rolled || atGoal;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DTP: End");
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

}

package frc.robot.commands;

import java.text.DecimalFormat;
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
public class PrintPositionCommand extends CommandBase {
  

  private final Supplier<Pose2d> poseProvider;
  private static final DecimalFormat df = new DecimalFormat("0.00");
  public PrintPositionCommand(Supplier<Pose2d> poseProvider) {

    this.poseProvider = poseProvider;


  }

  @Override
  public void initialize() {
    var robotPose = poseProvider.get();

    String outputStr = "new Pose2d("+df.format(robotPose.getX()) +", "+df.format(robotPose.getY()) +", Rotation2d.fromDegrees("+df.format(robotPose.getRotation().getDegrees()) +")";

    System.out.println("Current Position: " + outputStr);

  }

  @Override
  public void execute() {
    
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

}

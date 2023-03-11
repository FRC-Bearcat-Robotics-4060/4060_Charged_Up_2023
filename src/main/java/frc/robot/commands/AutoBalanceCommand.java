package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_onBalanceThreshold = 2.5;
    private double m_maxVelocity_mps = 0.35;

    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Rotation2d> robotAngleSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_onBalanceThreshold = SmartDashboard.getNumber("autoBalance.onBalanceThreshold", 1.5);
        m_maxVelocity_mps = SmartDashboard.getNumber("autoBalance.maxVelocity", 0.35);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        double x_meters_per_second = 0.0;
        double RollAngleDegrees = m_drivetrainSubsystem.getRoll();
        Boolean balanceRoll = Math.abs(RollAngleDegrees) >= m_onBalanceThreshold;
        
        // Control drive system automatically, 
        // driving in reverse direction of Roll/roll angle,
        // with a magnitude based upon the angle

        if ( balanceRoll ) {
            double sign = RollAngleDegrees > 0.0 ? 1.0 : -1.0;

            // 1m/s when we're at a 15 degree angle
            double min_velocity = 0.02;
            double max_velocity = 0.6;
            double P = max_velocity / 15;

            double amplitude = P * RollAngleDegrees;

            amplitude = Math.min(max_velocity, Math.abs(amplitude));
            amplitude = Math.max(min_velocity, Math.abs(amplitude));

            x_meters_per_second = amplitude * sign;
        }

        SmartDashboard.putNumber("autoBalance.Roll", RollAngleDegrees);
        SmartDashboard.putNumber("autoBalance.x_velocity", x_meters_per_second);

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x_meters_per_second, 0.0,
            0.0, m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}

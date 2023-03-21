package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.Constants;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private SlewRateLimiter RateLimiter_X;
    private SlewRateLimiter RateLimiter_Y;
    private SlewRateLimiter RateLimiter_R;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Rotation2d> robotAngleSupplier,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);

        RateLimiter_X = new SlewRateLimiter(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.DRIVE_TRANSLATION_RAMP_TIME);
        RateLimiter_Y = new SlewRateLimiter(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.DRIVE_TRANSLATION_RAMP_TIME);
        RateLimiter_R = new SlewRateLimiter(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.DRIVE_ROTATION_RAMP_TIME);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        double x = RateLimiter_X.calculate(m_translationXSupplier.getAsDouble());
        double y = RateLimiter_Y.calculate(m_translationYSupplier.getAsDouble());
        double r = RateLimiter_R.calculate(m_rotationSupplier.getAsDouble());

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                x, y, r, m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}

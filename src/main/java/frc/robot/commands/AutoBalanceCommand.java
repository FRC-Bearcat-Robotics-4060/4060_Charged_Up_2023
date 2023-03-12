package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class AutoBalanceCommand extends PIDCommand {
    private double m_onBalanceThreshold = 1.5;

    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        super(new PIDController(0.05, 0.0, 0.0), drivetrainSubsystem::getRoll, 0.0,
                output -> drivetrainSubsystem.drive_x_mps(output));
        getController().setTolerance(m_onBalanceThreshold);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist;

public class Wrist_Move extends CommandBase {

    private final Wrist m_wrist;
    private double m_angle;

    public Wrist_Move(double angle, Wrist subsystem) {

        m_angle = angle;

        m_wrist = subsystem;
        addRequirements(m_wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Tell m_write to move to angle
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Nothing to do here
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Nothing to do here unless we have a sensor on the hand. If open-loop (no sensors) we
        // might want to wait for a time to elapse.
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}

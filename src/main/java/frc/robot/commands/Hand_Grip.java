package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Hand;

public class Hand_Grip extends CommandBase {

    private final Hand m_hand;
    private double m_Position;

    public Hand_Grip(double Position, Hand subsystem) {

        m_Position = Position;

        m_hand = subsystem;
        addRequirements(m_hand);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Tell m_hand to move to position.
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
        // Nothing to do here unless we have a sensor on the wrist. If open-loop (no sensors) we
        // might want to wait for a time to elapse.
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}

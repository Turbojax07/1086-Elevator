package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetPosition extends Command {
    private Elevator elevator;
    private Distance position;

    /**
     * Creates a new SetPosition command.
     * Tells the elevator to go to a certain height above the ground.
     * 
     * @param elevator The elevator subsystem to use.
     * @param position The height to travel to as a {@link Distance}
     */
    public SetPosition(Elevator elevator, Distance position) {
        this.elevator = elevator;
        this.position = position;

        addRequirements(elevator);
    }

    /** Runs once when the command is first scheduled. */
    @Override
    public void initialize() {}

    /** Runs once every tick the command is scheduled. */
    @Override
    public void execute() {
        elevator.setGoalPosition(position);
    }

    /**
     * Runs once every tick the command is scheduled.
     * 
     * @return Whether or not the command should end.
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /** Runs once when the command is cancelled. */
    @Override
    public void end(boolean interrupted) {}
}
package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetPosition extends Command {
    private Distance position;
    private Elevator elevator;

    /**
     * Creates a new SetPosition command.
     * Tells the elevator to go to a certain height above the ground.
     * 
     * @param position The height to travel to as a {@link Distance}
     * @param elevator The elevator subsystem to use.
     */
    public SetPosition(Distance position, Elevator elevator) {
        this.position = position;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Command started");
    }

    @Override
    public void execute() {
        System.out.println("Command running");
        elevator.setGoalPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Command stopping");
    }
}
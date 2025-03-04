package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class SetPosition extends Command {
    private Distance position;
    private Elevator elevator;

    /**
     * Tells the elevator to go to a certain height above the ground.
     * 
     * @param position The height to travel to as a {@link Distance}
     * @param elevator The elevator subsystem to use.
     */
    public SetPosition(Distance position, Elevator elevator) {
        this.position = position;
        this.elevator = elevator;
    }

    /**
     * Tells the elevator to go to a certain height above the ground.
     * 
     * @param position The height to travel to as an {@link ElevatorPosition}
     * @param elevator The elevator subsystem to use.
     */
    public SetPosition(ElevatorPosition position, Elevator elevator) {
        this.position = position.value;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
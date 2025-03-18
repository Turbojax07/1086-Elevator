package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MoveDown extends Command {
    private Elevator elevator;

    /**
     * Tells the elevator to go down a stage.  Loops around at the bottom.
     * 
     * @param elevator The elevator subsystem to use.
     */
    public MoveDown(Elevator elevator) {
        this.elevator = elevator;
    }

    /** Runs once when the command is first scheduled. */
    @Override
    public void initialize() {}

    /** Runs every tick that the command is scheduled. */
    @Override
    public void execute() {
        Distance position = elevator.getGoalPosition();

        if (position.isEquivalent(ElevatorPosition.INTAKE)) {
            elevator.setGoalPosition(ElevatorPosition.L4);
        } else if (position.isEquivalent(ElevatorPosition.L1)) {
            elevator.setGoalPosition(ElevatorPosition.INTAKE);
        } else if (position.isEquivalent(ElevatorPosition.L2)) {
            elevator.setGoalPosition(ElevatorPosition.L1);
        } else if (position.isEquivalent(ElevatorPosition.L3)) {
            elevator.setGoalPosition(ElevatorPosition.L2);
        } else if (position.isEquivalent(ElevatorPosition.L4)) {
            elevator.setGoalPosition(ElevatorPosition.L3);
        }
    }

    /**
     * Runs every tick that the command is scheduled.
     * 
     * @return Whether or not the command should end.
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /** Runs once when the command is cancelled.  */
    @Override
    public void end(boolean interrupted) {}
}
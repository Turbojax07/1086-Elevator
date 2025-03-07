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

    @Override
    public void initialize() {}

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

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
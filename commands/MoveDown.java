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
        Distance position = elevator.getGoalPose();

        if (position.isEquivalent(ElevatorPosition.STOW.value)) {
            elevator.setPosition(ElevatorPosition.L4);
        } else if (position.isEquivalent(ElevatorPosition.L1.value)) {
            elevator.setPosition(ElevatorPosition.STOW);
        } else if (position.isEquivalent(ElevatorPosition.L2.value)) {
            elevator.setPosition(ElevatorPosition.L1);
        } else if (position.isEquivalent(ElevatorPosition.L3.value)) {
            elevator.setPosition(ElevatorPosition.L2);
        } else if (position.isEquivalent(ElevatorPosition.L4.value)) {
            elevator.setPosition(ElevatorPosition.L3);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
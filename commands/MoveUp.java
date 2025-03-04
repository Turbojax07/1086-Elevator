package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MoveUp extends Command {
    private Elevator elevator;

    /**
     * Tells the elevator to go up a stage.  Stops at L4.
     * 
     * @param elevator The elevator subsystem to use.
     */
    public MoveUp(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Distance curPos = elevator.getGoalPose();

        if (curPos.isEquivalent(ElevatorPosition.STOW.value)) {
            elevator.setPosition(ElevatorPosition.L1);
        } else if (curPos.isEquivalent(ElevatorPosition.L1.value)) {
            elevator.setPosition(ElevatorPosition.L2);
        } else if (curPos.isEquivalent(ElevatorPosition.L2.value)) {
            elevator.setPosition(ElevatorPosition.L3);
        } else if (curPos.isEquivalent(ElevatorPosition.L3.value)) {
            elevator.setPosition(ElevatorPosition.L4);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
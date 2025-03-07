package frc.robot.subsystems.elevator.commands;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ZeroElevator extends Command {
    private Elevator elevator;
    private Voltage volts;
    private Current maxCurrent;

    /**
     * Creates a new ZeroElevator command.
     * This runs the elevator with x volts until the current is greater than maxCurrent or the command is cancelled.
     * If the command isn't interrupted, it tells the elevator to reset the encoder position to 0.
     * Voltage should be negative, and the command will not run if it is positive.
     * 
     * @param elevator The elevator subsystem to control.
     * @param volts The volts to run at.  It should be negative.
     * @param current The current to run at.
     */
    public ZeroElevator(Elevator elevator, Voltage volts, Current maxCurrent) {
        this.elevator = elevator;
        this.volts = volts;
        this.maxCurrent = maxCurrent;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        if (volts.gt(Volts.zero())) cancel();
    }

    @Override
    public void execute() {
        Logger.recordOutput("/Subsystems/Elevator/Zeroing", true);

        elevator.setVoltage(volts);
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrent().gt(maxCurrent);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltage(Volts.zero());

        Logger.recordOutput("/Subsystems/Elevator/Zeroing", false);

        if (interrupted) elevator.resetPosition(Meters.zero());
    }
}

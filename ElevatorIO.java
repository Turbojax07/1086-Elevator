package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        Distance position = Meters.zero();
        LinearVelocity velocity = MetersPerSecond.zero();

        Current leftCurrent = Amps.zero();
        Temperature leftTemperature = Celsius.zero();
        Voltage leftVoltage = Volts.zero();

        Current rightCurrent = Amps.zero();
        Temperature rightTemperature = Celsius.zero();
        Voltage rightVoltage = Volts.zero();
    }

    /**
     * Updates a set of IO inputs with current values.
     * 
     * @param inputs The inputs to update.
     */
    public void updateInputs(ElevatorIOInputs inputs);
    
    /**
     * Sets the output voltage of the elevator.
     * 
     * @param volts The voltage to apply.
     */
    public void setVoltage(Voltage volts);

    /**
     * Resets the odometry of the elevator.
     * 
     * @param newPosition The position to reset to.
     */
    public void reset(Distance newPosition);
}
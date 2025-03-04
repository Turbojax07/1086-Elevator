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
        public Distance position = Meters.zero();
        public LinearVelocity velocity = MetersPerSecond.zero();

        public Current leftCurrent = Amps.zero();
        public Temperature leftTemperature = Celsius.zero();
        public Voltage leftVoltage = Volts.zero();

        public Current rightCurrent = Amps.zero();
        public Temperature rightTemperature = Celsius.zero();
        public Voltage rightVoltage = Volts.zero();
    }

    public void updateInputs(ElevatorIOInputs inputs);

    public void setPosition(Distance height, double ffVoltage);

    public void reset();
}

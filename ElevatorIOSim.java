package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(2), ElevatorConstants.mass.in(Kilograms), ElevatorConstants.radius.in(Meters), ElevatorConstants.gearRatio),
        DCMotor.getNEO(2), 0, ElevatorConstants.maxHeight.in(Meters), true, 0);

    private Voltage appliedVolts = Volts.zero();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.setInputVoltage(appliedVolts.in(Volts));
        sim.update(0.02);

        inputs.position = Meters.of(sim.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

        inputs.leftCurrent = Amps.of(sim.getCurrentDrawAmps());
        inputs.leftVoltage = appliedVolts;

        inputs.rightCurrent = inputs.leftCurrent;
        inputs.rightVoltage = inputs.leftVoltage.unaryMinus();
    }

    @Override
    public void setVoltage(Voltage volts) {
        appliedVolts = volts;
    }

    @Override
    public void reset(Distance newPosition) {
        sim.setState(newPosition.in(Meters), sim.getVelocityMetersPerSecond());
    }
}

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.AdjustableNumbers;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(2), ElevatorConstants.mass.in(Kilograms), ElevatorConstants.radius.in(Meters), ElevatorConstants.gearRatio),
        DCMotor.getNEO(2), 0, ElevatorConstants.maxHeight.in(Meters), true, 0);

    private PIDController feedback = new PIDController(AdjustableNumbers.getValue("kPElev"), AdjustableNumbers.getValue("kIElev"), AdjustableNumbers.getValue("kDElev"));
    private double feedforward = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        feedback.setPID(AdjustableNumbers.getValue("kPElev"), AdjustableNumbers.getValue("kIElev"), AdjustableNumbers.getValue("kDElev"));

        double position = sim.getPositionMeters();

        double voltage = MathUtil.clamp(feedforward + feedback.calculate(position), -12.0, 12.0);
        sim.setInputVoltage(voltage);
        sim.update(0.02);

        inputs.position = Meters.of(position);
        inputs.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

        inputs.leftCurrent = Amps.of(sim.getCurrentDrawAmps());
        inputs.leftVoltage = Volts.of(voltage);

        inputs.rightCurrent = inputs.leftCurrent;
        inputs.rightVoltage = inputs.leftVoltage.unaryMinus();
    }

    @Override
    public void setPosition(Distance height, double ffVoltage) {
        feedforward = ffVoltage;
        feedback.setSetpoint(height.in(Meters));
    }

    @Override
    public void reset() {
        sim.setState(0, sim.getVelocityMetersPerSecond());
    }
}

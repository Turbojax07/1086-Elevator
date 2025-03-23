package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public ElevatorIOReal(int leftId, int rightId) {
        leftMotor = new TalonFX(leftId);
        rightMotor = new TalonFX(rightId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.gearRatio;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        rightMotor.setControl(new Follower(leftId, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = Meters.of(leftMotor.getPosition().getValue().in(Radians) * Constants.ElevatorConstants.radius.in(Meters));
        inputs.velocity = MetersPerSecond.of(leftMotor.getVelocity().getValue().in(RadiansPerSecond) * Constants.ElevatorConstants.radius.in(Meters));

        inputs.leftCurrent = leftMotor.getStatorCurrent().getValue();
        inputs.leftTemperature = leftMotor.getDeviceTemp().getValue();
        inputs.leftVoltage = leftMotor.getMotorVoltage().getValue();

        inputs.rightCurrent = rightMotor.getStatorCurrent().getValue();
        inputs.rightTemperature = rightMotor.getDeviceTemp().getValue();
        inputs.rightVoltage = rightMotor.getMotorVoltage().getValue();
    }

    @Override
    public void setVoltage(Voltage volts) {
        leftMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void reset(Distance newPosition) {
        leftMotor.setPosition(newPosition.in(Meters) / Constants.ElevatorConstants.radius.in(Meters) / 2 / Math.PI);
        rightMotor.setPosition(newPosition.in(Meters) / Constants.ElevatorConstants.radius.in(Meters) / 2 / Math.PI);
    }
}
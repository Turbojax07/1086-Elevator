package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController feedback;

    public ElevatorIOReal(int leftId, int rightId) {
        leftMotor = new SparkMax(leftId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        config.encoder
            .positionConversionFactor(Constants.ElevatorConstants.positionConversionFactor)
            .velocityConversionFactor(Constants.ElevatorConstants.velocityConversionFactor);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(leftMotor, true);

        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leftMotor.getEncoder();
        feedback = leftMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = Meters.of(encoder.getPosition());
        inputs.velocity = MetersPerSecond.of(encoder.getVelocity());

        inputs.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.leftTemperature = Celsius.of(leftMotor.getMotorTemperature());
        inputs.leftVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());

        inputs.rightCurrent = Amps.of(rightMotor.getOutputCurrent());
        inputs.rightTemperature = Celsius.of(rightMotor.getMotorTemperature());
        inputs.rightVoltage = Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    }

    @Override
    public void setVoltage(Voltage volts) {
        feedback.setReference(volts.in(Volts), ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void reset(Distance newPosition) {
        encoder.setPosition(newPosition.in(Meters));
    }
}
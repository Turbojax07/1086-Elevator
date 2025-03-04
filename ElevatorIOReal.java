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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.AdjustableNumbers;

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
            .inverted(false)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        config.encoder
            .positionConversionFactor(ElevatorConstants.positionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
        config.closedLoop
            .p(AdjustableNumbers.getValue("kPElev"))
            .i(AdjustableNumbers.getValue("kIElev"))
            .d(AdjustableNumbers.getValue("kDElev"));

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
            .follow(leftMotor, true);

        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leftMotor.getEncoder();
        feedback = leftMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (AdjustableNumbers.hasChanged("kPElev") || AdjustableNumbers.hasChanged("kIElev") || AdjustableNumbers.hasChanged("kDElev")) {
            SparkMaxConfig pidConfig = new SparkMaxConfig();
            pidConfig.closedLoop.pid(AdjustableNumbers.getValue("kPElev"), AdjustableNumbers.getValue("kIElev"), AdjustableNumbers.getValue("kDElev"));

            leftMotor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            rightMotor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

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
    public void setPosition(Distance position, double ffVoltage) {
        feedback.setReference(position.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
    }

    @Override
    public void reset() {
        encoder.setPosition(0);
    }
}
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AdjustableNumbers;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public class ElevatorPosition {
        public static final Distance STOW = Meters.of(0);
        public static final Distance INTAKE = Meters.of(0.057);
        public static final Distance L1 = Meters.of(0.33);
        public static final Distance L2 = Meters.of(0.63);
        public static final Distance L3Algae = Meters.of(0.81);
        public static final Distance L3 = Meters.of(1.05);
        public static final Distance L4 = Meters.of(1.76);
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ProfiledPIDController pidController = new ProfiledPIDController(AdjustableNumbers.getNumber("Elev_kP"), AdjustableNumbers.getNumber("Elev_kI"), AdjustableNumbers.getNumber("Elev_kD"), new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity.in(MetersPerSecond), ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    private ElevatorFeedforward l1FeedForward = new ElevatorFeedforward(AdjustableNumbers.getNumber("Elev_kS_L1"), AdjustableNumbers.getNumber("Elev_kG_L1"), AdjustableNumbers.getNumber("Elev_kV"), AdjustableNumbers.getNumber("Elev_kA_L1"));
    private ElevatorFeedforward l2FeedForward = new ElevatorFeedforward(AdjustableNumbers.getNumber("Elev_kS_L2"), AdjustableNumbers.getNumber("Elev_kG_L2"), AdjustableNumbers.getNumber("Elev_kV"), AdjustableNumbers.getNumber("Elev_kA_L2"));
    private ElevatorFeedforward l3FeedForward = new ElevatorFeedforward(AdjustableNumbers.getNumber("Elev_kS_L3"), AdjustableNumbers.getNumber("Elev_kG_L3"), AdjustableNumbers.getNumber("Elev_kV"), AdjustableNumbers.getNumber("Elev_kA_L3"));

    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();

    private final SysIdRoutine routine;

    public Elevator(ElevatorIO io) {
        this.io = io;

        io.updateInputs(inputs);

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(ElevatorConstants.sysIdRampUp, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(ElevatorConstants.sysIdStep, Units.Volts),
                Time.ofRelativeUnits(ElevatorConstants.sysIdTimeout, Units.Seconds)
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("elevator")
                        .voltage(inputs.leftVoltage)
                        .linearPosition(inputs.position)
                        .linearVelocity(inputs.velocity);
                },
                this
            )
        );
    }

    @Override
    public void periodic() {
        // Updating IO inputs
        io.updateInputs(inputs);

        // Updating PID values
        pidController.setPID(AdjustableNumbers.getNumber("Elev_kP"), AdjustableNumbers.getNumber("Elev_kI"), AdjustableNumbers.getNumber("Elev_kD"));

        // Updating states
        setpointState = pidController.getSetpoint();
        goalState = pidController.getGoal();

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));

        double ffVolts = 0;
        if (getPosition().in(Meters) < 0.33) {
            ffVolts = l1FeedForward.calculateWithVelocities(currentState.velocity, pidController.getSetpoint().velocity);
        }

	    if (getPosition().in(Meters) < 0.65) {
            ffVolts = l2FeedForward.calculateWithVelocities(currentState.velocity, pidController.getSetpoint().velocity);
        }

        if (getPosition().in(Meters) > 0.65) {
            ffVolts = l3FeedForward.calculateWithVelocities(currentState.velocity, pidController.getSetpoint().velocity);
        }

        io.setVoltage(Volts.of(ffVolts + pidController.calculate(currentState.position)));

        Logger.recordOutput("/Subsystems/Elevator/Position/Measured", currentState.position);
        Logger.recordOutput("/Subsystems/Elevator/Velocity/Measured", currentState.velocity);
        Logger.recordOutput("/Subsystems/Elevator/Position/Goal",     goalState.position);

        Logger.recordOutput("/Subsystems/Elevator/Velocity/Setpoint", setpointState.velocity);
        Logger.recordOutput("/Subsystems/Elevator/Position/Setpoint", setpointState.position);
        Logger.recordOutput("/Subsystems/Elevator/Velocity/Goal",     goalState.velocity);

        Logger.recordOutput("/Subsystems/Elevator/Feedforward", ffVolts);

        Logger.processInputs("/RealOutputs/Subsystems/Elevator/Inputs", inputs);
    }

    public Distance getPosition() {
        return inputs.position;
    }

    public void resetPosition(Distance newPosition) {
        io.reset(newPosition);
    }

    public Distance getGoalPosition() {
        return Meters.of(goalState.position);
    }

    public void setGoalPosition(Distance position) {
        pidController.setGoal(new TrapezoidProfile.State(position.in(Meters), 0));
    }

    public LinearVelocity getGoalVelocity() {
        return MetersPerSecond.of(goalState.velocity);
    }

    public Distance getSetpointPosition() {
        return Meters.of(setpointState.position);
    }

    public LinearVelocity getSetpointVelocity() {
        return MetersPerSecond.of(setpointState.velocity);
    }

    public Voltage getVoltage() {
        return inputs.leftVoltage;
    }

    public void setVoltage(Voltage volts) {
        io.setVoltage(volts);
    }

    public Current getCurrent() {
        return inputs.leftCurrent;
    }

    public LinearVelocity getVelocity() {
        return inputs.velocity;
    }

    public Command sysIdRoutine() {
        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.position.gt(ElevatorConstants.sysIdMaxPosition)),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position.lt(ElevatorConstants.sysIdMinPosition)),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.position.gt(ElevatorConstants.sysIdMaxPosition)),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position.lt(ElevatorConstants.sysIdMinPosition))
        );
    }
}

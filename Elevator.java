package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public enum ElevatorPosition {
        STOW(0),
        INTAKE(0.066),
        L1(0.33),
        L2(0.65),
        L3(1.10),
        L4(1.75);

        public Distance value;

        private ElevatorPosition(double value) {
            this.value = Meters.of(value);
        }
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ExponentialProfile l1Profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(ElevatorConstants.maxProfileVoltage - ElevatorConstants.kS[0] - ElevatorConstants.kG[0], ElevatorConstants.kV, ElevatorConstants.kA[0]));
    private final ExponentialProfile l2Profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(ElevatorConstants.maxProfileVoltage - ElevatorConstants.kS[1] - ElevatorConstants.kG[1], ElevatorConstants.kV, ElevatorConstants.kA[1]));
    private final ExponentialProfile l3Profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(ElevatorConstants.maxProfileVoltage - ElevatorConstants.kS[2] - ElevatorConstants.kG[2], ElevatorConstants.kV, ElevatorConstants.kA[2]));

    private ExponentialProfile.State currentState = new ExponentialProfile.State(0, 0);
    private ExponentialProfile.State goalState = new ExponentialProfile.State(0, 0);
    private ExponentialProfile.State futureState = new ExponentialProfile.State(0, 0);

    private ElevatorFeedforward l1FeedForward = new ElevatorFeedforward(ElevatorConstants.kS[0], ElevatorConstants.kG[0], ElevatorConstants.kV, ElevatorConstants.kA[0]);
    private ElevatorFeedforward l2FeedForward = new ElevatorFeedforward(ElevatorConstants.kS[1], ElevatorConstants.kG[1], ElevatorConstants.kV, ElevatorConstants.kA[1]);
    private ElevatorFeedforward l3FeedForward = new ElevatorFeedforward(ElevatorConstants.kS[2], ElevatorConstants.kG[2], ElevatorConstants.kV, ElevatorConstants.kA[2]);

    private final SysIdRoutine routine;

    public Elevator(ElevatorIO io) {
        this.io = io;

        io.updateInputs(inputs);

        currentState.position = inputs.position.in(Meters);
        goalState.position = inputs.position.in(Meters);

        currentState.velocity = inputs.velocity.in(MetersPerSecond);
        goalState.velocity = inputs.velocity.in(MetersPerSecond);

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(ElevatorConstants.sysIdRampUp, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(ElevatorConstants.sysIdStep, Units.Volts),
                Time.ofRelativeUnits(ElevatorConstants.sysIdTimeout, Units.Seconds)
            ),
            new SysIdRoutine.Mechanism(
                voltage -> io.setPosition(Meters.zero(), voltage.magnitude()),
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
        currentState = new ExponentialProfile.State(inputs.position.in(Meters), inputs.velocity.in(MetersPerSecond));

        double ffVolts = 0;
	if (getPosition().in(Meters) < 0.33) {
            futureState = l1Profile.calculate(0.02, currentState, goalState);
            ffVolts = l1FeedForward.calculateWithVelocities(currentState.velocity, futureState.velocity);
        }

	if (getPosition().in(Meters) < 0.65) {
            futureState = l2Profile.calculate(0.02, currentState, goalState);
            ffVolts = l2FeedForward.calculateWithVelocities(currentState.velocity, futureState.velocity);
        }

	if (getPosition().in(Meters) > 0.65) {
            futureState = l3Profile.calculate(0.02, currentState, goalState);
            ffVolts = l3FeedForward.calculateWithVelocities(currentState.velocity, futureState.velocity);
        }

        io.setPosition(Meters.of(futureState.position), ffVolts);

        Logger.recordOutput("/Subsystems/Elevator/Position/Setpoint", futureState.position);
        Logger.recordOutput("/Subsystems/Elevator/Velocity/Setpoint", futureState.velocity);
        Logger.recordOutput("/Subsystems/Elevator/Position/Goal", goalState.position);
        Logger.recordOutput("/Subsystems/Elevator/Velocity/Goal", goalState.velocity);
        Logger.recordOutput("/Subsystems/Elevator/Feedforward", ffVolts);

        io.updateInputs(inputs);
        Logger.processInputs("/RealOuptuts/Subsystems/Elevator", inputs);
    }

    @AutoLogOutput(key="/Subsystems/Elevator/Position/Measured")
    public Distance getPosition() {
        return inputs.position;
    }

    public void setPosition(Distance position) {
        this.goalState = new ExponentialProfile.State(position.in(Meters), 0);
    }

    public void setPosition(ElevatorPosition position) {
        this.goalState = new ExponentialProfile.State(position.value.in(Meters), 0);
    }

    public Distance getGoalPose() {
        return Meters.of(goalState.position);
    }

    @AutoLogOutput(key="/Subsystems/Elevator/Velocity/Measured")
    public LinearVelocity getVelocity() {
        return inputs.velocity;
    }

    public void reset() {
        io.reset();
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

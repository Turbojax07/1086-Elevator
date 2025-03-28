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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AdjustableValues;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public class ElevatorPosition {
        public static final Distance STOW = Meters.of(0);
        public static final Distance INTAKE = Meters.of(0.057);
        public static final Distance L1 = Meters.of(0.33);
        public static final Distance L2 = Meters.of(0.63);
        public static final Distance L3 = Meters.of(1.05);
        public static final Distance L3Algae = Meters.of(0.81);
        public static final Distance L4 = Meters.of(1.76);
    }

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ProfiledPIDController pidController = new ProfiledPIDController(AdjustableValues.getNumber("Elev_kP"), AdjustableValues.getNumber("Elev_kI"), AdjustableValues.getNumber("Elev_kD"), new TrapezoidProfile.Constraints(Constants.ElevatorConstants.maxVelocity.in(MetersPerSecond), Constants.ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    private ElevatorFeedforward l1FeedForward = new ElevatorFeedforward(AdjustableValues.getNumber("Elev_kS_L1"), AdjustableValues.getNumber("Elev_kG_L1"), AdjustableValues.getNumber("Elev_kV"), AdjustableValues.getNumber("Elev_kA_L1"));
    private ElevatorFeedforward l2FeedForward = new ElevatorFeedforward(AdjustableValues.getNumber("Elev_kS_L2"), AdjustableValues.getNumber("Elev_kG_L2"), AdjustableValues.getNumber("Elev_kV"), AdjustableValues.getNumber("Elev_kA_L2"));
    private ElevatorFeedforward l3FeedForward = new ElevatorFeedforward(AdjustableValues.getNumber("Elev_kS_L3"), AdjustableValues.getNumber("Elev_kG_L3"), AdjustableValues.getNumber("Elev_kV"), AdjustableValues.getNumber("Elev_kA_L3"));

    private TrapezoidProfile.State measuredState = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();

    private SysIdRoutine routine;

    private boolean voltageControl = false;

    public Elevator(ElevatorIO io) {
        this.io = io;

        io.updateInputs(inputs);

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(Constants.ElevatorConstants.sysIdRampUp, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(Constants.ElevatorConstants.sysIdStep, Units.Volts),
                Time.ofRelativeUnits(Constants.ElevatorConstants.sysIdTimeout, Units.Seconds)
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
        if (AdjustableValues.hasChanged("Elev_kP")) pidController.setP(AdjustableValues.getNumber("Elev_kP"));
        if (AdjustableValues.hasChanged("Elev_kI")) pidController.setI(AdjustableValues.getNumber("Elev_kI"));
        if (AdjustableValues.hasChanged("Elev_kD")) pidController.setD(AdjustableValues.getNumber("Elev_kD"));

        if (AdjustableValues.hasChanged("Elev_kS_L1")) l1FeedForward.setKs(AdjustableValues.getNumber("Elev_kS_L1"));
        if (AdjustableValues.hasChanged("Elev_kG_L1")) l1FeedForward.setKg(AdjustableValues.getNumber("Elev_kG_L1"));
        if (AdjustableValues.hasChanged("Elev_kV_L1")) l1FeedForward.setKv(AdjustableValues.getNumber("Elev_kV_L1"));
        if (AdjustableValues.hasChanged("Elev_kA_L1")) l1FeedForward.setKa(AdjustableValues.getNumber("Elev_kA_L1"));
        
        if (AdjustableValues.hasChanged("Elev_kS_L2")) l2FeedForward.setKs(AdjustableValues.getNumber("Elev_kS_L2"));
        if (AdjustableValues.hasChanged("Elev_kG_L2")) l2FeedForward.setKg(AdjustableValues.getNumber("Elev_kG_L2"));
        if (AdjustableValues.hasChanged("Elev_kV_L2")) l2FeedForward.setKv(AdjustableValues.getNumber("Elev_kV_L2"));
        if (AdjustableValues.hasChanged("Elev_kA_L2")) l2FeedForward.setKa(AdjustableValues.getNumber("Elev_kA_L2"));
        
        if (AdjustableValues.hasChanged("Elev_kS_L3")) l3FeedForward.setKs(AdjustableValues.getNumber("Elev_kS_L3"));
        if (AdjustableValues.hasChanged("Elev_kG_L3")) l3FeedForward.setKg(AdjustableValues.getNumber("Elev_kG_L3"));
        if (AdjustableValues.hasChanged("Elev_kV_L3")) l3FeedForward.setKv(AdjustableValues.getNumber("Elev_kV_L3"));
        if (AdjustableValues.hasChanged("Elev_kA_L3")) l3FeedForward.setKa(AdjustableValues.getNumber("Elev_kA_L3"));

        // Updating states
        measuredState = new TrapezoidProfile.State(inputs.position.in(Meters), inputs.velocity.in(MetersPerSecond));
        setpointState = pidController.getSetpoint();
        goalState = pidController.getGoal();

        double ffVolts = 0;
        if (measuredState.position < 0.33) {
            ffVolts = l1FeedForward.calculateWithVelocities(measuredState.velocity, pidController.getSetpoint().velocity);
        }

	    if (measuredState.position < 0.65) {
            ffVolts = l2FeedForward.calculateWithVelocities(measuredState.velocity, pidController.getSetpoint().velocity);
        }

        if (measuredState.position > 0.65) {
            ffVolts = l3FeedForward.calculateWithVelocities(measuredState.velocity, pidController.getSetpoint().velocity);
        }

        // Only controls the elevator if voltage control is disabled
        // voltage control is disabled when the setGoalPosition function is run.
        if (!voltageControl) io.setVoltage(Volts.of(ffVolts + pidController.calculate(measuredState.position)));

        // Logging values
        Logger.recordOutput("/Subsystems/Elevator/Position/Measured", measuredState.position);
        Logger.recordOutput("/Subsystems/Elevator/Position/Setpoint", setpointState.position);
        Logger.recordOutput("/Subsystems/Elevator/Position/Goal",     goalState.position);

        Logger.recordOutput("/Subsystems/Elevator/Velocity/Measured", measuredState.velocity);
        Logger.recordOutput("/Subsystems/Elevator/Velocity/Setpoint", setpointState.velocity);
        Logger.recordOutput("/Subsystems/Elevator/Velocity/Goal",     goalState.velocity);

        Logger.recordOutput("/Subsystems/Elevator/Feedforward", ffVolts);

        Logger.processInputs("/RealOutputs/Subsystems/Elevator/Inputs", inputs);
    }

    /** Gets the measured position of the elevator. */
    public Distance getMeasuredPosition() {
        return Meters.of(measuredState.position);
    }

    /**
     * Resets the odometry of the elevator.
     * 
     * @param newPosition The position to set the odometry to.
     */
    public void setMeasuredPosition(Distance newPosition) {
        io.reset(newPosition);
    }

    /** Gets the measured velocity of the elevator. */
    public LinearVelocity getMeasuredVelocity() {
        return MetersPerSecond.of(measuredState.velocity);
    }

    /** Gets the goal position of the elevator. */
    public Distance getGoalPosition() {
        return Meters.of(goalState.position);
    }

    /**
     * Sets the goal position of the elevator.
     * 
     * @param position The position to travel to.
     */
    public void setGoalPosition(Distance position) {
        voltageControl = false;

        pidController.setGoal(new TrapezoidProfile.State(position.in(Meters), 0));
    }

    /** Gets the setpoint position of the elevator. */
    public Distance getSetpointPosition() {
        return Meters.of(setpointState.position);
    }

    /** Gets the setpoint velocity of the elevator. */
    public LinearVelocity getSetpointVelocity() {
        return MetersPerSecond.of(setpointState.velocity);
    }

    /** Gets the output voltage of the elevator. */
    public Voltage getVoltage() {
        return inputs.leftVoltage;
    }

    /**
     * Sets the input voltage of the elevator.
     * 
     * @param volts The voltage to run at.
     */
    public void setVoltage(Voltage volts) {
        voltageControl = true;

        io.setVoltage(volts);
    }

    /** Gets the current (Amps) output of the elevator. */
    public Current getCurrent() {
        return inputs.leftCurrent;
    }

    /** The sysId command for quasistatic forward. */
    public Command sysIdQuasistaticForward() {
        return routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.position.gt(Constants.ElevatorConstants.sysIdMaxPosition));
    }

    /** The sysId command for quasistatic reverse. */
    public Command sysIdQuasistaticReverse() {
        return routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position.lt(Constants.ElevatorConstants.sysIdMinPosition));
    }

    /** The sysId command for Dynamic forward. */
    public Command sysIdDynamicForward() {
        return routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.position.gt(Constants.ElevatorConstants.sysIdMaxPosition));
    }

    /** The sysId command for Dynamic reverse. */
    public Command sysIdDynamicReverse() {
        return routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position.lt(Constants.ElevatorConstants.sysIdMinPosition));
    }
}
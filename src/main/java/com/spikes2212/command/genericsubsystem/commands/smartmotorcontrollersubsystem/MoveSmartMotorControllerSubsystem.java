package com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a {@link SmartMotorControllerSubsystem} using its master motor controller's control loops.
 *
 * @author Yoel Perman Brilliant
 * @see SmartMotorControllerSubsystem
 */
public class MoveSmartMotorControllerSubsystem extends CommandBase {

    /**
     * The {@link SmartMotorControllerSubsystem} this command will run on.
     */
    protected final SmartMotorControllerSubsystem subsystem;

    /**
     * The setpoint this command should bring the {@link SmartMotorControllerSubsystem} to.
     */
    protected final Supplier<Double> setpoint;

    /**
     * The loop's control mode (e.g. voltage, velocity, position...). Only applicable when
     * using a CTRE motor controller.
     */
    protected final ControlMode controlMode;

    /**
     * The loop's control type (e.g. voltage, velocity, position...). Only applicable when
     * using a {@link CANSparkMax} motor controller.
     */
    protected final CANSparkMax.ControlType controlType;

    /**
     * The loop's PID constants.
     */
    protected final PIDSettings pidSettings;

    /**
     * The loop's feed forward gains.
     */
    protected final FeedForwardSettings feedForwardSettings;

    /**
     * The most recent timestamp on which the loop has not reached its target setpoint.
     */
    private double lastTimeNotOnTarget;

    /**
     * Constructs a new (generic) instance of {@link MoveSmartMotorControllerSubsystem}.
     *
     * @param subsystem           the {@link SmartMotorControllerSubsystem} this command will run on
     * @param pidSettings         the loop's PID constants
     * @param feedForwardSettings the loop's feed forward gains
     * @param controlMode         the loop's control mode (e.g. voltage, velocity, position...).
     *                            Only applicable when using CTRE motor controllers
     * @param controlType         the loop's control mode (e.g. voltage, velocity, position...).
     *                            Only applicable when using {@link CANSparkMax} motor controllers
     * @param setpoint            the setpoint this command should bring the {@link SmartMotorControllerSubsystem} to
     */
    protected MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                                FeedForwardSettings feedForwardSettings,
                                                ControlMode controlMode, CANSparkMax.ControlType controlType,
                                                Supplier<Double> setpoint) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.controlMode = controlMode;
        this.controlType = controlType;
        this.pidSettings = pidSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.setpoint = setpoint;
        this.lastTimeNotOnTarget = 0;
    }

    /**
     * Constructs a new instance of {@link MoveSmartMotorControllerSubsystem} for a subsystem which consists of
     * CTRE motor controllers.
     *
     * @param subsystem           the {@link SmartMotorControllerSubsystem} this command will run on
     * @param pidSettings         the loop's PID constants
     * @param feedForwardSettings the loop's feed forward gains
     * @param controlMode         the loop's control mode (e.g. voltage, velocity, position...).
     *                            Only applicable when using CTRE motor controllers
     * @param setpoint            the setpoint this command should bring the subsystem to
     */
    public MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                             FeedForwardSettings feedForwardSettings,
                                             ControlMode controlMode, Supplier<Double> setpoint) {
        this(subsystem, pidSettings, feedForwardSettings, controlMode, null, setpoint);
    }

    /**
     * Constructs a new instance of {@link MoveSmartMotorControllerSubsystem} for a subsystem which consists of
     * {@link CANSparkMax} motor controllers.
     *
     * @param subsystem           the {@link SmartMotorControllerSubsystem} this command will run on
     * @param pidSettings         the loop's PID constants
     * @param feedForwardSettings the loop's feed forward gains
     * @param controlType         the loop's control mode (e.g. voltage, velocity, position...).
     *                            Only applicable when using {@link CANSparkMax} motor controllers
     * @param setpoint            the setpoint this command should bring the subsystem to
     */
    public MoveSmartMotorControllerSubsystem(SmartMotorControllerSubsystem subsystem, PIDSettings pidSettings,
                                             FeedForwardSettings feedForwardSettings,
                                             CANSparkMax.ControlType controlType, Supplier<Double> setpoint) {
        this(subsystem, pidSettings, feedForwardSettings, null, controlType, setpoint);
    }

    /**
     * Configures the subsystem's control loops.
     */
    @Override
    public void initialize() {
        subsystem.configureLoop(pidSettings, feedForwardSettings);
    }

    /**
     * Updates any control loops running on the subsystem.
     * This method uses the {@code pidSet} method twice: once assuming the subsystem consists of CTRE motor controllers,
     * and once assuming it consists of {@link CANSparkMax}s. In any given {@link SmartMotorControllerSubsystem},
     * exactly one {@code pidSet} method should be implemented.
     */
    @Override
    public void execute() {
        subsystem.pidSet(controlMode, setpoint.get(), pidSettings, feedForwardSettings);
        subsystem.pidSet(controlType, setpoint.get(), pidSettings, feedForwardSettings);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.finish();
    }

    /**
     * Checks if the subsystem has been at its target setpoint for longer than its allowed wait time.
     *
     * @return {@code true} if the command should finish running, {@code false} otherwise
     */
    @Override
    public boolean isFinished() {
        if (!subsystem.onTarget(controlMode, controlType, pidSettings.getTolerance(), setpoint.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}

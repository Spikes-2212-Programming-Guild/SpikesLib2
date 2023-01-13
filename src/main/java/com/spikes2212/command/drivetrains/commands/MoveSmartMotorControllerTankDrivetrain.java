package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.SmartMotorControllerTankDrivetrain;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a {@link SmartMotorControllerTankDrivetrain} using its master motor controller's control loops.
 *
 * @author Yoel Perman Brilliant
 * @see SmartMotorControllerTankDrivetrain
 */
public class MoveSmartMotorControllerTankDrivetrain extends CommandBase {

    /**
     * The {@link SmartMotorControllerTankDrivetrain} this command will run on.
     */
    protected final SmartMotorControllerTankDrivetrain drivetrain;

    /**
     * The left side loop's PID constants.
     */
    protected final PIDSettings leftPIDSettings;

    /**
     * The right side loop's PID constants.
     */
    protected final PIDSettings rightPIDSettings;

    /**
     * The loop's feed forward gains.
     */
    protected final FeedForwardSettings feedForwardSettings;

    /**
     * The loop's control mode (e.g. voltage, velocity, position...).
     */
    protected final UnifiedControlMode controlMode;

    /**
     * The setpoint this command should bring the left side of the {@link SmartMotorControllerTankDrivetrain} to.
     */
    protected final Supplier<Double> leftSetpoint;

    /**
     * The setpoint this command should bring the right side of the {@link SmartMotorControllerTankDrivetrain} to.
     */
    protected final Supplier<Double> rightSetpoint;


    /**
     * The most recent timestamp on which the left side's loop has not reached its target setpoint.
     */
    private double lastTimeLeftNotOnTarget;

    /**
     * The most recent timestamp on which the right side's loop has not reached its target setpoint.
     */
    private double lastTimeRightNotOnTarget;


    /**
     * Constructs a new (generic) instance of {@link MoveSmartMotorControllerTankDrivetrain}.
     *
     * @param drivetrain           the {@link SmartMotorControllerGenericSubsystem} this command will run on
     * @param leftPIDSettings     the left side's loop's PID constants
     * @param rightPIDSettings    the right side's loop's PID constants
     * @param feedForwardSettings the loop's feed forward gains
     * @param controlMode         the loop's control mode (e.g. voltage, velocity, position...)
     * @param leftSetpoint        the setpoint this command should bring the
     *                            {@link SmartMotorControllerGenericSubsystem}'s left side to
     * @param rightSetpoint       the setpoint this command should bring the
     *                            {@link SmartMotorControllerGenericSubsystem}'s right side to
     */
    public MoveSmartMotorControllerTankDrivetrain(SmartMotorControllerTankDrivetrain drivetrain,
                                                  PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                                                  FeedForwardSettings feedForwardSettings,
                                                  UnifiedControlMode controlMode, Supplier<Double> leftSetpoint,
                                                  Supplier<Double> rightSetpoint) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.controlMode = controlMode;
        this.leftPIDSettings = leftPIDSettings;
        this.rightPIDSettings = rightPIDSettings;
        this.feedForwardSettings = feedForwardSettings;
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.lastTimeLeftNotOnTarget = 0;
        this.lastTimeRightNotOnTarget = 0;
    }

    /**
     * Configures the subsystem's control loops.
     */
    @Override
    public void initialize() {
        drivetrain.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings);
    }

    /**
     * Updates any control loops running on the subsystem.
     */
    @Override
    public void execute() {
        drivetrain.pidSet(controlMode, leftSetpoint.get(), rightSetpoint.get(), leftPIDSettings,
                rightPIDSettings, feedForwardSettings);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.finish();
    }

    /**
     * Checks if the subsystem has been at its target setpoint for longer than its allowed wait time.
     *
     * @return {@code true} if the command should finish running, {@code false} otherwise
     */
    @Override
    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();
        if (!drivetrain.leftOnTarget(controlMode, leftPIDSettings.getTolerance(), leftSetpoint.get())) {
            lastTimeLeftNotOnTarget = now;
        }
        if (!drivetrain.rightOnTarget(controlMode, leftPIDSettings.getTolerance(), rightSetpoint.get())) {
            lastTimeRightNotOnTarget = now;
        }
        //is this good? it seems wrong
        return Timer.getFPGATimestamp() - lastTimeLeftNotOnTarget >= leftPIDSettings.getWaitTime() &&
                Timer.getFPGATimestamp() - lastTimeRightNotOnTarget >= rightPIDSettings.getWaitTime();
    }
}

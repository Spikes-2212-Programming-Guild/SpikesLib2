package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.ClosedLoopSpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Predicate;
import java.util.function.Supplier;

/**
 * This command makes a {@link ClosedLoopSpeedController} run a PID loop.
 */
public class MovePIDSpeedController extends CommandBase {

    /**
     * The {@link ClosedLoopSpeedController} with the necessary details.
     */
    private ClosedLoopSpeedController speedController;

    /**
     * A {@link Supplier} that returns the maximum speed allowed for the PID loop.
     */
    private Supplier<Double> maxSpeed;

    /**
     * A {@link Supplier} that returns the minimum speed allowed for the PID loop.
     */
    private Supplier<Double> minSpeed;

    /**
     * A {@link Supplier} that returns the target setpoint for the PID loop.
     */
    private Supplier<Double> setpoint;

    /**
     * A {@link Predicate} that receives the speed of the motor and
     * returns whether that speed is valid for that motor.
     */
    private Predicate<Double> canMove;
    private double lastTimeNotOnTarget;

    /**
     * A full constructor for {@code MoveSpeedControllerWithPID}.
     *
     * @param subsystem       the subsystem this command requires
     * @param speedController the {@link ClosedLoopSpeedController} this command runs a PID loop on
     * @param maxSpeed        the maximum speed of the subsystem
     * @param minSpeed        the minimum speed of the subsystem
     * @param setpoint        the setpoint which the PID loop should reach
     * @param canMove         a predicate that returns true for speeds at which the subsystem can move
     */
    public MovePIDSpeedController(Subsystem subsystem, ClosedLoopSpeedController speedController, Supplier<Double> maxSpeed,
                                  Supplier<Double> minSpeed, Supplier<Double> setpoint, Predicate<Double> canMove) {
        addRequirements(subsystem);
        this.speedController = speedController;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.setpoint = setpoint;
        this.canMove = canMove;
    }

    /**
     * A constructor that uses a {@link GenericSubsystem}.
     *
     * @param genericSubsystem the subsystem this command requires
     * @param speedController  the {@link ClosedLoopSpeedController} this command runs a PID loop on
     * @param setpoint         the setpoint which the PID loop should reach
     */
    public MovePIDSpeedController(GenericSubsystem genericSubsystem, ClosedLoopSpeedController speedController,
                                  Supplier<Double> setpoint) {
        this(genericSubsystem, speedController, genericSubsystem.maxSpeed, genericSubsystem.minSpeed, setpoint, genericSubsystem::canMove);
    }

    /**
     * A partial constructor, omitting {@code canMove}. Assumes the subsystem can always move at any speed between
     * {@code maxSpeed} and {@code minSpeed}.
     *
     * @param subsystem       the subsystem this command requires
     * @param speedController the {@link ClosedLoopSpeedController} this command runs a PID loop on
     * @param maxSpeed        the maximum speed of the subsystem
     * @param minSpeed        the minimum speed of the subsystem
     * @param setpoint        the setpoint which the PID loop should reach
     */
    public MovePIDSpeedController(Subsystem subsystem, ClosedLoopSpeedController speedController, Supplier<Double> maxSpeed,
                                  Supplier<Double> minSpeed, Supplier<Double> setpoint) {
        this(subsystem, speedController, maxSpeed, minSpeed, setpoint, v -> true);
    }

    @Override
    public void initialize() {
        speedController.configureLoop(maxSpeed, minSpeed);
    }

    @Override
    public void execute() {
        speedController.pidSet(setpoint.get());
    }

    @Override
    public void end(boolean interrupted) {
        speedController.finish();
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        if (!speedController.onTarget(setpoint.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return !canMove.test(speedController.get()) ||
                currentTime - lastTimeNotOnTarget >= speedController.getWaitTime();
    }
}

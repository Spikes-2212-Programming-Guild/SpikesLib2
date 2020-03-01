package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.TalonSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * Move a {@link TalonSubsystem} using its Talon's control loops.
 */
public class MoveTalonSubsystem extends CommandBase {
    /**
     * The {@link TalonSubsystem} this command will run on.
     */
    private final TalonSubsystem subsystem;

    /**
     * The setpoint this command should bring the {@link TalonSubsystem} to.
     */
    private final Supplier<Double> setpoint;

    private final Supplier<Double> waitTime;
    private double lastTimeNotOnTarget;


    public MoveTalonSubsystem(TalonSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> waitTime) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.setpoint = setpoint;
        this.waitTime = waitTime;
        this.lastTimeNotOnTarget = 0;
    }

    public MoveTalonSubsystem(TalonSubsystem subsystem, double setpoint, Supplier<Double> waitTime) {
        this(subsystem, () -> setpoint, waitTime);
    }

    @Override
    public void initialize() {
        subsystem.configureLoop();
    }

    @Override
    public void execute() {
        subsystem.pidSet(setpoint.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.finish();
    }

    @Override
    public boolean isFinished() {
        if(!subsystem.onTarget(setpoint.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget > waitTime.get();
    }
}

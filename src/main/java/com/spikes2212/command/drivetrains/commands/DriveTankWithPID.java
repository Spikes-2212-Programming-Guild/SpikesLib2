package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} using two PID loops, one for each side.
 *
 * @author Yuval Levy
 * @see TankDrivetrain
 */
public class DriveTankWithPID extends Command {

    /**
     * The drivetrain this command operates on.
     */
    protected final TankDrivetrain drivetrain;

    /**
     * The PID Settings for the PID loop operating on the left side of the drivetrain.
     */
    protected final PIDSettings leftPIDSettings;

    /**
     * The PID Settings for the PID loop operating on the right side of the drivetrain.
     */
    protected final PIDSettings rightPIDSettings;

    /**
     * The FeedForwards Settings of the FeedForward loop operating on the left side of the drivetrain.
     */
    protected final FeedForwardSettings leftFeedForwardSettings;

    /**
     * The FeedForwards Settings of the FeedForward loop operating on the right side of the drivetrain.
     */
    protected final FeedForwardSettings rightFeedForwardSettings;

    /**
     * The PID Controller of the PID loop operating on the left side of the drivetrain.
     */
    protected final PIDController leftPIDController;

    /**
     * The PID Controller of the PID loop operating on the right side of the drivetrain.
     */
    protected final PIDController rightPIDController;

    /**
     * The setpoint the left side of the drivetrain should reach.
     */
    protected final Supplier<Double> leftSetpoint;

    /**
     * The setpoint the right side of the drivetrain should reach.
     */
    protected final Supplier<Double> rightSetpoint;

    /**
     * The acceleration the left side of the drivetrain should reach.
     */
    protected final Supplier<Double> leftAcceleration;

    /**
     * The acceleration the right side of the drivetrain should reach.
     */
    protected final Supplier<Double> rightAcceleration;

    /**
     * How far the left side of the drivetrain drove.
     */
    protected final Supplier<Double> leftSource;

    /**
     * How far the right side of the drivetrain drove.
     */
    protected final Supplier<Double> rightSource;

    /**
     * The FeedForwards Controller of the FeedForward loop operating on the left side of the drivetrain.
     */
    protected final FeedForwardController leftFeedForwardController;

    /**
     * The FeedForwards Controller of the FeedForward loop operating on the right side of the drivetrain.
     */
    protected final FeedForwardController rightFeedForwardController;

    /**
     * The last time the left side of the drivetrain was not within its target zone.
     */
    private double leftLastTimeNotOnTarget;

    /**
     * The last time the right side of the drivetrain was not within its target zone.
     */
    private double rightLastTimeNotOnTarget;

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, Supplier<Double> leftAcceleration,
                            Supplier<Double> rightAcceleration, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftPIDSettings = leftPIDSettings;
        this.leftPIDController = new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(),
                leftPIDSettings.getkD());
        this.rightPIDSettings = rightPIDSettings;
        this.rightPIDController = new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(),
                rightPIDSettings.getkD());
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftSource = leftSource;
        this.rightSource = rightSource;
        this.leftAcceleration = leftAcceleration;
        this.rightAcceleration = rightAcceleration;
        this.leftPIDController.setSetpoint(leftSetpoint.get());
        this.rightPIDController.setSetpoint(rightSetpoint.get());
        this.leftFeedForwardSettings = leftFeedForwardSettings;
        this.rightFeedForwardSettings = rightFeedForwardSettings;
        this.leftFeedForwardController = new FeedForwardController(leftFeedForwardSettings.getkS(),
                leftFeedForwardSettings.getkV(), leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG(),
                leftFeedForwardSettings.getControlMode());
        this.rightFeedForwardController = new FeedForwardController(rightFeedForwardSettings.getkS(),
                rightFeedForwardSettings.getkV(), rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG(),
                rightFeedForwardSettings.getControlMode());
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint,
                            Supplier<Double> leftSource, Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, leftSetpoint, rightSetpoint, leftSource, rightSource,
                () -> 0.0, () -> 0.0, FeedForwardSettings.EMPTY_FF_SETTINGS,
                FeedForwardSettings.EMPTY_FF_SETTINGS);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, Supplier<Double> leftAcceleration,
                            Supplier<Double> rightAcceleration, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint,
                leftAcceleration, rightAcceleration, leftSource, rightSource, leftFeedForwardSettings,
                rightFeedForwardSettings);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint, () -> 0.0,
                () -> 0.0, leftSource, rightSource, FeedForwardSettings.EMPTY_FF_SETTINGS,
                FeedForwardSettings.EMPTY_FF_SETTINGS);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, leftSetpoint, rightSetpoint, leftSource, rightSource,
                () -> 0.0, () -> 0.0, leftFeedForwardSettings, rightFeedForwardSettings);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint,
                () -> 0.0, () -> 0.0, leftSource, rightSource, leftFeedForwardSettings,
                rightFeedForwardSettings);
    }

    @Override
    public void execute() {
        leftPIDController.setSetpoint(leftSetpoint.get());
        rightPIDController.setSetpoint(rightSetpoint.get());
        leftPIDController.setTolerance(leftPIDSettings.getTolerance());
        rightPIDController.setTolerance(rightPIDSettings.getTolerance());
        leftPIDController.setPID(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
        rightPIDController.setPID(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
        leftFeedForwardController.setGains(leftFeedForwardSettings);
        rightFeedForwardController.setGains(rightFeedForwardSettings);
        drivetrain.tankDrive((leftPIDController.calculate(leftSource.get()) +
                        leftFeedForwardController.calculate(leftSource.get(), leftSetpoint.get(),
                                leftAcceleration.get())),
                rightPIDController.calculate(rightSource.get()) +
                        rightFeedForwardController.calculate(rightSource.get(), rightSetpoint.get(),
                                rightAcceleration.get()));
    }

    @Override
    public boolean isFinished() {
        if (!leftPIDController.atSetpoint()) {
            leftLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        if (!rightPIDController.atSetpoint()) {
            rightLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - leftLastTimeNotOnTarget >= leftPIDSettings.getWaitTime()
                && Timer.getFPGATimestamp() - rightLastTimeNotOnTarget >= rightPIDSettings.getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}

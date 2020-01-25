package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a drivetrain using two PID loops, one for each side.
 */
public class DriveTankWithPID extends CommandBase {
    /**
     * The drivetrain this command operates on.
     */
    private final TankDrivetrain drivetrain;

    /**
     * The PID Settings for the PID loop operating on the left side of the drivetrain.
     */
    private PIDSettings leftPIDSettings;

    /**
     * The PID Settings for the PID loop operating on the right side of the drivetrain.
     */
    private PIDSettings rightPIDSettings;

    /**
     * The PID Controller of the PID loop operating on the left side of the drivetrain.
     */
    private PIDController leftPIDController;

    /**
     * The PID Controller of the PID loop operating on the right side of the drivetrain.
     */
    private PIDController rightPIDController;

    /**
     * The setpoint the left side of the drivetrain should reach.
     */
    private Supplier<Double> leftSetpoint;

    /**
     * The setpoint the right side of the drivetrain should reach.
     */
    private Supplier<Double> rightSetpoint;

    /**
     * How far the left side of the drivetrain drove.
     */
    private Supplier<Double> leftSource;

    /**
     * How far the right side of the drivetrain drove.
     */
    private Supplier<Double> rightSource;

    /**
     * The last time the left side of the drivetrain was not within its target zone.
     */
    private double leftLastTimeNotOnTarget;

    /**
     * The last time the right side of the drivetrain was not within its target zone.
     */
    private double rightLastTimeNotOnTarget;

    private FeedForwardSettings leftFeedForwardSettings;
    private FeedForwardSettings rightFeedForwardSettings;
    private FeedForwardController leftFeedForwardController;
    private FeedForwardController rightFeedForwardController;

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftPIDSettings = leftPIDSettings;
        this.leftPIDController = new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
        this.rightPIDSettings = rightPIDSettings;
        this.rightPIDController = new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftSource = leftSource;
        this.rightSource = rightSource;
        this.leftPIDController.setSetpoint(leftSetpoint.get());
        this.rightPIDController.setSetpoint(rightSetpoint.get());
        this.leftFeedForwardSettings = leftFeedForwardSettings;
        this.rightFeedForwardSettings = rightFeedForwardSettings;
        this.leftFeedForwardController = new FeedForwardController(leftFeedForwardSettings.getkS(),
                leftFeedForwardSettings.getkV(), leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG());
        this.rightFeedForwardController = new FeedForwardController(rightFeedForwardSettings.getkS(),
                rightFeedForwardSettings.getkV(), rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG());
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, leftSetpoint, rightSetpoint, leftSource, rightSource,
                FeedForwardSettings.EMPTY_FFSETTINGS, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint, leftSource,
                rightSource, leftFeedForwardSettings, rightFeedForwardSettings);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint, leftSource,
                rightSource, FeedForwardSettings.EMPTY_FFSETTINGS, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    @Override
    public void execute() {
        leftPIDController.setSetpoint(rightSetpoint.get());
        rightPIDController.setSetpoint(leftSetpoint.get());
        leftPIDController.setTolerance(leftPIDSettings.getTolerance());
        rightPIDController.setTolerance(rightPIDSettings.getTolerance());
        leftPIDController.setPID(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
        rightPIDController.setPID(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
        leftFeedForwardController.setGains(leftFeedForwardSettings.getkS(), leftFeedForwardSettings.getkV(),
                leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG());
        rightFeedForwardController.setGains(rightFeedForwardSettings.getkS(), rightFeedForwardSettings.getkV(),
                rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG());
        drivetrain.tankDrive((leftPIDController.calculate(leftSource.get()) +
                        leftFeedForwardController.calculate(leftSetpoint.get()) / 2),
                rightPIDController.calculate(rightSource.get()) +
                        leftFeedForwardController.calculate(rightSetpoint.get()) / 2);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
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
}

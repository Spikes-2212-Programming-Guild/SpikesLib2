package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDFSettings;
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
    private PIDFSettings leftPIDFSettings;

    /**
     * The PID Settings for the PID loop operating on the right side of the drivetrain.
     */
    private PIDFSettings rightPIDFSettings;

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

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDFSettings leftPIDFSettings, PIDFSettings rightPIDFSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftPIDFSettings = leftPIDFSettings;
        this.leftPIDController = new PIDController(leftPIDFSettings.getkP(), leftPIDFSettings.getkI(), leftPIDFSettings.getkD());
        this.rightPIDFSettings = rightPIDFSettings;
        this.rightPIDController = new PIDController(rightPIDFSettings.getkP(), rightPIDFSettings.getkI(), rightPIDFSettings.getkD());
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

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDFSettings leftPIDFSettings, PIDFSettings rightPIDFSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource) {
        this(drivetrain, leftPIDFSettings, rightPIDFSettings, leftSetpoint, rightSetpoint, leftSource, rightSource,
                FeedForwardSettings.EMPTY_FFSETTINGS, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDFSettings leftPIDFSettings, PIDFSettings rightPIDFSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        this(drivetrain, leftPIDFSettings, rightPIDFSettings, () -> leftSetpoint, () -> rightSetpoint, leftSource,
                rightSource, leftFeedForwardSettings, rightFeedForwardSettings);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDFSettings leftPIDFSettings, PIDFSettings rightPIDFSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource) {
        this(drivetrain, leftPIDFSettings, rightPIDFSettings, () -> leftSetpoint, () -> rightSetpoint, leftSource,
                rightSource, FeedForwardSettings.EMPTY_FFSETTINGS, FeedForwardSettings.EMPTY_FFSETTINGS);
    }

    @Override
    public void execute() {
        leftPIDController.setSetpoint(rightSetpoint.get());
        rightPIDController.setSetpoint(leftSetpoint.get());
        leftPIDController.setTolerance(leftPIDFSettings.getTolerance());
        rightPIDController.setTolerance(rightPIDFSettings.getTolerance());
        leftPIDController.setPID(leftPIDFSettings.getkP(), leftPIDFSettings.getkI(), leftPIDFSettings.getkD());
        rightPIDController.setPID(rightPIDFSettings.getkP(), rightPIDFSettings.getkI(), rightPIDFSettings.getkD());
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
        double currentTime = Timer.getFPGATimestamp();

        if (!leftPIDController.atSetpoint()) {
            leftLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        if (!rightPIDController.atSetpoint()) {
            rightLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return currentTime - leftLastTimeNotOnTarget >= leftPIDFSettings.getWaitTime()
                && currentTime - rightLastTimeNotOnTarget >= rightPIDFSettings.getWaitTime();
    }
}

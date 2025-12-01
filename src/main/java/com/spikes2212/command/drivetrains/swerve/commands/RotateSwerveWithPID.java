package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class RotateSwerveWithPID extends Command {

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> setPoint;

    private final PIDSettings pidSettings;
    private final FeedForwardSettings feedForwardSettings;
    private final PIDController pidController;
    private final FeedForwardController feedForwardController;

    protected double lastGivenTime;
    protected double now;
    protected double lastTimeNotOnTarget;

    /**
     * Constructs a new {@link DriveSwerve} command that moves the given
     * {@link SwerveDrivetrain}
     *
     * @param drivetrain the swerve drivetrain this command operates on
     * @param setPoint   the desired angle
     */
    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setPoint, PIDSettings pidSettings,
                               FeedForwardSettings feedForwardSettings) {
        this.drivetrain = drivetrain;
        this.setPoint = setPoint;
        this.pidSettings = pidSettings;
        this.feedForwardSettings = feedForwardSettings;

        pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());

        feedForwardController = new FeedForwardController(feedForwardSettings);
        lastTimeNotOnTarget = 0;
    }

    @Override
    public void initialize() {
        lastGivenTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        now = Timer.getFPGATimestamp();
        feedForwardController.setGains(feedForwardSettings);
        drivetrain.drive(0, 0, pidController.calculate(
                        drivetrain.getCurrentRobotAngle().getDegrees(), setPoint.get())
                        + feedForwardController.calculate(drivetrain.getCurrentRobotAngle().getDegrees(),
                        setPoint.get()),    false, now - lastGivenTime, false);
        lastGivenTime = now;
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() <= now - lastTimeNotOnTarget;
    }
}

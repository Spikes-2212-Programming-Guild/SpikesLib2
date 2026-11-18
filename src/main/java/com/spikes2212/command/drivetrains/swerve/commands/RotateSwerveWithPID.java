package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that rotates a given {@link SwerveDrivetrain} to a target heading using closed-loop
 * position control while optionally translating.
 *
 * @author Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class RotateSwerveWithPID extends Command {

    private static final double MIN_ANGLE_DEGREES = -180;
    private static final double MAX_ANGLE_DEGREES = 180;
    private static final double DEFAULT_TRANSLATION_SPEED = 0;

    protected final PIDSettings pidSettings;
    protected final FeedForwardSettings ffSettings;
    protected final PIDController pidController;
    protected final FeedForwardController ffController;
    protected final Supplier<Double> xSpeed;
    protected final Supplier<Double> ySpeed;
    protected final boolean isFieldRelative;

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> setpoint;

    protected double lastGivenTime;
    protected double now;
    protected double lastTimeNotOnTarget;

    /**
     * Constructs a new {@link RotateSwerveWithPID} command that rotates the given {@code SwerveDrivetrain}
     * to a target angle while moving along the x-axis and y-axis.
     *
     * @param drivetrain      the swerve drivetrain this command operates on
     * @param setpoint        the desired heading angle in degrees
     * @param xSpeed          the speed along the x-axis
     * @param ySpeed          the speed along the y-axis
     * @param pidSettings     the PID settings for the rotational movement
     * @param ffSettings      the feed forward settings for the rotational movement
     * @param isFieldRelative whether translation speeds are field-relative
     */
    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> xSpeed,
                               Supplier<Double> ySpeed, PIDSettings pidSettings, FeedForwardSettings ffSettings,
                               boolean isFieldRelative) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidSettings = pidSettings;
        this.ffSettings = ffSettings;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.isFieldRelative = isFieldRelative;

        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        this.pidController.setIZone(pidSettings.getIZone());
        this.pidController.setTolerance(pidSettings.getTolerance());
        this.pidController.enableContinuousInput(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        this.ffController = new FeedForwardController(ffSettings);
        this.lastTimeNotOnTarget = 0;
    }

    /**
     * Constructs a new {@code RotateSwerveWithPID} command that rotates the given {@code SwerveDrivetrain}
     * to a target angle while translating robot-relatively along the x-axis and y-axis.
     *
     * @param drivetrain  the swerve drivetrain this command operates on
     * @param setpoint    the desired heading angle in degrees
     * @param xSpeed      the speed along the x-axis
     * @param ySpeed      the speed along the y-axis
     * @param pidSettings the PID settings for the rotational movement
     * @param ffSettings  the feed forward settings for the rotational movement
     */
    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> xSpeed,
                               Supplier<Double> ySpeed, PIDSettings pidSettings, FeedForwardSettings ffSettings) {
        this(drivetrain, setpoint, xSpeed, ySpeed, pidSettings, ffSettings, false);
    }

    /**
     * Constructs a new {@code RotateSwerveWithPID} command that rotates the given {@code SwerveDrivetrain}
     * in place to a target angle.
     *
     * @param drivetrain     the swerve drivetrain this command operates on
     * @param setpoint       the desired heading angle in degrees
     * @param pidSettings    the PID settings for the rotational movement
     * @param ffSettings     the feed forward settings for the rotational movement
     * @param useVelocityPID whether the module drives with velocity PID
     */
    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, PIDSettings pidSettings,
                               FeedForwardSettings ffSettings, boolean useVelocityPID) {
        this(drivetrain, setpoint, () -> DEFAULT_TRANSLATION_SPEED, () -> DEFAULT_TRANSLATION_SPEED,
                pidSettings, ffSettings, false);
    }

    @Override
    public void initialize() {
        lastGivenTime = Timer.getFPGATimestamp();
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        now = Timer.getFPGATimestamp();
        ffController.setGains(ffSettings);
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());
        double currentAngle = drivetrain.getAngle().getDegrees();
        double targetAngle = setpoint.get();
        double rotationOutput = pidController.calculate(currentAngle, targetAngle)
                + ffController.calculate(currentAngle, targetAngle);
        drivetrain.drive(xSpeed.get(), ySpeed.get(), rotationOutput, isFieldRelative, now - lastGivenTime);
        lastGivenTime = now;
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) lastTimeNotOnTarget = Timer.getFPGATimestamp();
        return pidSettings.getWaitTime() <= now - lastTimeNotOnTarget;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}

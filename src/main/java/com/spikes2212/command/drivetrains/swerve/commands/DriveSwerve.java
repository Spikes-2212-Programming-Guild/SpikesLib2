package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link SwerveDrivetrain} according to linear at rotational speeds.
 *
 * @author  Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class DriveSwerve extends Command {

    private final SwerveDrivetrain drivetrain;

    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;

    private final boolean isFieldRelative;
    private final boolean useVelocityPID;

    /**
     * These variables are used to calculate the time step.
     */
    protected double lastGivenTime;
    protected double now;

    /**
     * Constructs a new {@link DriveSwerve} command that moves the given
     * {@link SwerveDrivetrain}.
     *
     * @param drivetrain      the swerve drivetrain this command operates on
     * @param xSpeed          the desired {@link Supplier} speed on the x-axis
     * @param ySpeed          the desired {@link Supplier} speed on the y-axis
     * @param rotationSpeed   the desired rotational {@link Supplier} speed
     * @param isFieldRelative whether the drive should be relative to the field or to itself
     * @param useVelocityPID  whether the robot velocity will be applied using P.I.D or not
     */
    public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean isFieldRelative, boolean useVelocityPID) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.isFieldRelative = isFieldRelative;
        this.useVelocityPID = useVelocityPID;
    }

    @Override
    public void initialize() {
        lastGivenTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        now = Timer.getFPGATimestamp();
        drivetrain.drive(xSpeed.get(), ySpeed.get(), rotationSpeed.get(), isFieldRelative,
                now - lastGivenTime, useVelocityPID);
        lastGivenTime = now;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}

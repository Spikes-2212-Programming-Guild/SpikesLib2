package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link SwerveDrivetrain} according to linear and rotational speeds.
 *
 * @author Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class DriveSwerve extends Command {

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;
    private final boolean isFieldRelative;

    protected double lastGivenTime;
    protected double now;

    /**
     * Constructs a new {@link DriveSwerve} command that moves the given {@code SwerveDrivetrain}.
     *
     * @param drivetrain      the swerve drivetrain this command operates on
     * @param xSpeed          the desired {@link Supplier} speed on the x-axis
     * @param ySpeed          the desired {@code Supplier} speed on the y-axis
     * @param rotationSpeed   the desired rotational {@code Supplier} speed
     * @param isFieldRelative whether the drive should be relative to the field or to the robot
     */
    public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean isFieldRelative) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.isFieldRelative = isFieldRelative;
    }

    @Override
    public void initialize() {
        lastGivenTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        now = Timer.getFPGATimestamp();
        drivetrain.drive(xSpeed.get(), ySpeed.get(), rotationSpeed.get(), isFieldRelative,
                now - lastGivenTime);
        lastGivenTime = now;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}

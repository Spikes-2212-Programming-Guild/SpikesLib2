package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.SwerveDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class DriveSwerve extends Command {

    private static final double DEFAULT_X_ACCELERATION_LIMIT = 2;
    private static final double DEFAULT_Y_ACCELERATION_LIMIT = 2;
    private static final double DEFAULT_ROTATION_ACCELERATION_LIMIT = 4;

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;
    private final boolean fieldRelative;
    private final boolean usePID;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotationLimiter;

    private boolean limitAcceleration;

    public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean usePID,
                       double xAccelerationLimit, double yAccelerationLimit, double rotationAccelerationLimit) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.fieldRelative = fieldRelative;
        this.usePID = usePID;
        this.xLimiter = new SlewRateLimiter(xAccelerationLimit);
        this.yLimiter = new SlewRateLimiter(yAccelerationLimit);
        this.rotationLimiter = new SlewRateLimiter(rotationAccelerationLimit);
    }

    public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean usePID,
                       boolean limitAcceleration) {
        this(drivetrain, xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID, DEFAULT_X_ACCELERATION_LIMIT,
                DEFAULT_Y_ACCELERATION_LIMIT, DEFAULT_ROTATION_ACCELERATION_LIMIT);
        this.limitAcceleration = limitAcceleration;
    }

    public DriveSwerve(SwerveDrivetrain drivetrain, double xSpeed, double ySpeed,
                       double rotationSpeed, boolean fieldRelative, boolean usePID,
                       double xAccelerationLimit, double yAccelerationLimit, double rotationAccelerationLimit) {
        this(drivetrain, () -> xSpeed, () -> ySpeed, () -> rotationSpeed, fieldRelative, usePID,
                xAccelerationLimit, yAccelerationLimit, rotationAccelerationLimit);
    }

    public DriveSwerve(SwerveDrivetrain drivetrain, double xSpeed, double ySpeed,
                       double rotationSpeed, boolean fieldRelative, boolean usePID, boolean limitAcceleration) {
        this(drivetrain, xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID, DEFAULT_X_ACCELERATION_LIMIT,
                DEFAULT_Y_ACCELERATION_LIMIT, DEFAULT_ROTATION_ACCELERATION_LIMIT);
        this.limitAcceleration = limitAcceleration;
    }

    @Override
    public void execute() {
        double xSpeed;
        double ySpeed;
        double rotationSpeed;
        if (limitAcceleration) {
            xSpeed = xLimiter.calculate(this.xSpeed.get());
            ySpeed = yLimiter.calculate(this.ySpeed.get());
            rotationSpeed = rotationLimiter.calculate(this.rotationSpeed.get());
        } else {
            xSpeed = this.xSpeed.get();
            ySpeed = this.ySpeed.get();
            rotationSpeed = this.rotationSpeed.get();
        }

        drivetrain.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}

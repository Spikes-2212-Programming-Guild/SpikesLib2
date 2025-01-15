package com.spikes2212.command.drivetrains.swervedrivetrains.commands;

import com.spikes2212.command.drivetrains.swervedrivetrains.drivetrains.SwerveDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class SwerveDrive extends Command {

    private final SwerveDrivetrain swerveDrivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;

    private final boolean fieldRelative;
    private final boolean usePID;
    private final boolean limitSpeed;
    private final boolean limitAcceleration;

    private final SlewRateLimiter xRateLimiter;
    private final SlewRateLimiter yRateLimiter;
    private final SlewRateLimiter rotationRateLimiter;

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean usePID, boolean limitSpeed,
                       double xRateLimit, double yRateLimit, double rotationRateLimit) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.fieldRelative = fieldRelative;
        this.usePID = usePID;
        this.limitSpeed = limitSpeed;
        xRateLimiter = new SlewRateLimiter(xRateLimit);
        yRateLimiter = new SlewRateLimiter(yRateLimit);
        rotationRateLimiter = new SlewRateLimiter(rotationRateLimit);
        limitAcceleration = true;
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean usePID, boolean limitSpeed) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.fieldRelative = fieldRelative;
        this.usePID = usePID;
        this.limitSpeed = limitSpeed;
        xRateLimiter = new SlewRateLimiter(0);
        yRateLimiter = new SlewRateLimiter(0);
        rotationRateLimiter = new SlewRateLimiter(0);
        limitAcceleration = false;
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean limitSpeed,
                       double xRateLimit, double yRateLimit, double rotationRateLimit) {
        this(swerveDrivetrain, xSpeed, ySpeed, rotationSpeed, fieldRelative, false, limitSpeed, xRateLimit, yRateLimit,
                rotationRateLimit);
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean limitSpeed) {
        this(swerveDrivetrain, xSpeed, ySpeed, rotationSpeed, fieldRelative, false, limitSpeed);
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, double xSpeed, double ySpeed,
                       double rotationSpeed, boolean fieldRelative, boolean usePID, boolean limitSpeed,
                       double xRateLimit, double yRateLimit, double rotationRateLimit) {
        this(swerveDrivetrain, () -> xSpeed, () -> ySpeed, () -> rotationSpeed, fieldRelative, usePID, limitSpeed,
                xRateLimit, yRateLimit, rotationRateLimit);
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, double xSpeed, double ySpeed,
                       double rotationSpeed, boolean fieldRelative, boolean usePID, boolean limitSpeed) {
        this(swerveDrivetrain, () -> xSpeed, () -> ySpeed, () -> rotationSpeed, fieldRelative, usePID, limitSpeed);
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, double xSpeed, double ySpeed,
                       double rotationSpeed, boolean fieldRelative, boolean limitSpeed,
                       double xRateLimit, double yRateLimit, double rotationRateLimit) {
        this(swerveDrivetrain, () -> xSpeed, () -> ySpeed, () -> rotationSpeed, fieldRelative, false, limitSpeed,
                xRateLimit, yRateLimit, rotationRateLimit);
    }

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, double xSpeed, double ySpeed,
                       double rotationSpeed, boolean fieldRelative, boolean limitSpeed) {
        this(swerveDrivetrain, () -> xSpeed, () -> ySpeed, () -> rotationSpeed, fieldRelative, false, limitSpeed);
    }

    @Override
    public void execute() {
        double xSpeed;
        double ySpeed;
        double rotationSpeed;
        if (limitAcceleration) {
            xSpeed = xRateLimiter.calculate(this.xSpeed.get());
            ySpeed = yRateLimiter.calculate(this.ySpeed.get());
            rotationSpeed = rotationRateLimiter.calculate(this.rotationSpeed.get());
        } else {
            xSpeed = this.xSpeed.get();
            ySpeed = this.xSpeed.get();
            rotationSpeed = this.xSpeed.get();
        }
        swerveDrivetrain.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID, limitSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrivetrain.stop();
    }
}

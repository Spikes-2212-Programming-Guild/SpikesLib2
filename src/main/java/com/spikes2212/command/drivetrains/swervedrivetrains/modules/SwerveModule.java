package com.spikes2212.command.drivetrains.swervedrivetrains.modules;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.function.Supplier;

public abstract class SwerveModule extends DashboardedSubsystem {

    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DEGREES_TO_REVERSE = 180;
    private static final double MAX_DEGREES_NEEDED_TO_ROTATE = 90;

    private final MotorController driveController;
    private final MotorController turnController;
    private final Supplier<Double> absoluteAngle;

    private final double minSpeed;

    private final Translation2d modulePosition;

    public SwerveModule(String namespaceName, MotorController driveController, MotorController turnController,
                        Supplier<Double> absoluteAngle, double minSpeed, Translation2d modulePosition) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.absoluteAngle = absoluteAngle;
        this.minSpeed = minSpeed;
        this.modulePosition = modulePosition;
    }

    public SwerveModule(String namespaceName, MotorController driveController, MotorController turnController,
                        Supplier<Double> absoluteAngle, Translation2d modulePosition) {
        this(namespaceName, driveController, turnController, absoluteAngle, 0, modulePosition);
    }

    public void set(SwerveModuleState state, boolean usePID, boolean limitSpeed) {
        if (limitSpeed && state.speedMetersPerSecond < minSpeed) {
            stop();
            return;
        }
        state = optimize(state);
        setSpeed(state.speedMetersPerSecond, usePID, limitSpeed);
        setAngle(state.angle.getDegrees());
    }

    protected abstract void setSpeed(double speed, boolean usePID, boolean limitSpeed);

    protected abstract void setAngle(double angle);

    public void stop() {
        driveController.stopMotor();
        turnController.stopMotor();
    }

    public abstract void resetRelativeEncoders();

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public abstract double getAngle();

    public double getAbsoluteAngle() {
        return absoluteAngle.get();
    }

    public abstract double getVelocity();

    private SwerveModuleState optimize(SwerveModuleState state) {
        double wantedAngle = placeInAppropriate0To360Scope(state.angle.getDegrees());
        double wantedSpeed = state.speedMetersPerSecond;
        double error = wantedAngle - getAngle();
        while (Math.abs(error) > MAX_DEGREES_NEEDED_TO_ROTATE) {
            wantedAngle = error > 0 ? wantedAngle - DEGREES_TO_REVERSE : wantedAngle + DEGREES_TO_REVERSE;
            wantedSpeed *= -1;
        }
        return new SwerveModuleState(wantedSpeed, Rotation2d.fromDegrees(wantedAngle));
    }

    private double placeInAppropriate0To360Scope(double wantedAngle) {
        int rotations = (int) (getAngle() / DEGREES_IN_ROTATIONS);
        if (getAngle() < 0) rotations--;
        return wantedAngle + DEGREES_IN_ROTATIONS * rotations;
    }

    @Override
    public void configureDashboard() {
    }
}

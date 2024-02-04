package com.spikes2212.command.drivetrains;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.function.Supplier;

public abstract class GenericSwerveModule extends DashboardedSubsystem {

    protected final MotorController driveController;
    protected final MotorController turnController;
    private final Supplier<Double> moduleRotation;
    private final double minSpeedMetersPerSecond;

    public GenericSwerveModule(String namespaceName, MotorController driveController, MotorController turnController,
                               Supplier<Double> moduleRotation, boolean driveInverted, double minSpeedMetersPerSecond) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.moduleRotation = moduleRotation;
        driveController.setInverted(driveInverted);
        this.minSpeedMetersPerSecond = minSpeedMetersPerSecond;
    }

    public void set(SwerveModuleState state, boolean usePID) {
        if (state.speedMetersPerSecond < minSpeedMetersPerSecond) {
            stop();
            return;
        }
        state = optimize(state, Rotation2d.fromDegrees(moduleRotation.get()));
        double angle = state.angle.getDegrees();
        setAngle(angle);
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    public abstract double getSpeed();

    public abstract void setAngle(double angle);

    public abstract void setSpeed(double speedMetersPerSecond, boolean usePID);

    public void stop() {
        driveController.stopMotor();
        turnController.stopMotor();
    }

    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}

package com.spikes2212.control.speedcontrollers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.SmartMotionSettings;

import java.util.function.Supplier;

public class SmartMotionSparkMax extends PIDSparkMax {

    private final SmartMotionSettings motionSettings;

    public SmartMotionSparkMax(CANSparkMax sparkMax, PIDSettings settings, SmartMotionSettings motionSettings,
                               Supplier<Double> kF, int timeout) {
        super(sparkMax, settings, kF, ControlType.kVelocity, timeout);
        this.motionSettings = motionSettings;
    }

    @Override
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed) {
        super.configureLoop(maxSpeed, minSpeed);

        pidController.setSmartMotionMinOutputVelocity(motionSettings.getMinVelocity().get(), 0);
        pidController.setSmartMotionMaxVelocity(motionSettings.getMinVelocity().get(), 0);
        pidController.setSmartMotionMaxAccel(motionSettings.getMaxAcceleration().get(), 0);
    }

    @Override
    public void pidSet(double setpoint) {
        pidController.setSmartMotionMinOutputVelocity(motionSettings.getMinVelocity().get(), 0);
        pidController.setSmartMotionMaxVelocity(motionSettings.getMinVelocity().get(), 0);
        pidController.setSmartMotionMaxAccel(motionSettings.getMaxAcceleration().get(), 0);

        super.pidSet(setpoint);
    }
}

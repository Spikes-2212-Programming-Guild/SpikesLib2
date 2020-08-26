package com.spikes2212.control.speedcontrollers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.SmartMotionSettings;

import java.util.function.Supplier;

/**
 * A {@link CANSparkMax} that can run a PID loop.
 *
 * @author Ergoold
 */
public class SmartMotionSparkMax extends PIDSparkMax {

    /**
     * The smart motion loop's {@link SmartMotionSettings}.
     */
    private final SmartMotionSettings motionSettings;

    /**
     * Constructs a {@code SmartMotionSparkMax} with the given parameters as field values.
     *
     * @param sparkMax       the Spark Max speed controller on which the PID loop is calculated
     * @param settings       the PID loop's {@link PIDSettings}
     * @param motionSettings the smart motion loop's {@link SmartMotionSettings}
     * @param kF             the PID loop's feed forward constant
     */
    public SmartMotionSparkMax(CANSparkMax sparkMax, PIDSettings settings, SmartMotionSettings motionSettings,
                               Supplier<Double> kF, int timeout) {
        super(sparkMax, settings, kF, ControlType.kVelocity, timeout);
        this.motionSettings = motionSettings;
    }

    @Override
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed) {
        super.configureLoop(maxSpeed, minSpeed);

        pidController.setSmartMotionMinOutputVelocity(motionSettings.getMinVelocity(), 0);
        pidController.setSmartMotionMaxVelocity(motionSettings.getMinVelocity(), 0);
        pidController.setSmartMotionMaxAccel(motionSettings.getMaxAcceleration(), 0);
    }

    @Override
    public void pidSet(double setpoint) {
        pidController.setSmartMotionMinOutputVelocity(motionSettings.getMinVelocity(), 0);
        pidController.setSmartMotionMaxVelocity(motionSettings.getMinVelocity(), 0);
        pidController.setSmartMotionMaxAccel(motionSettings.getMaxAcceleration(), 0);

        super.pidSet(setpoint);
    }
}

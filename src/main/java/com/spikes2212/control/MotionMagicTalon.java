package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.function.Supplier;

public class MotionMagicTalon extends PIDTalon {

    private final MotionMagicSettings motionMagicSettings;

    /**
     * Constructs a PIDTalon instance with the given parameters as field values.
     *
     * @param talon    The Talon speed controller on which the PID loop is calculated.
     * @param settings The PID loop's {@link PIDFSettings}.
     * @param timeout
     */
    public MotionMagicTalon(WPI_TalonSRX talon, PIDFSettings settings, MotionMagicSettings mmSeetings, int timeout) {
        super(talon, settings, ControlMode.MotionMagic, timeout);
        motionMagicSettings = mmSeetings;
    }

    @Override
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed) {
        super.configureLoop(maxSpeed, minSpeed);
        talon.configMotionAcceleration(motionMagicSettings.getTargetAcceleration());
        talon.configMotionCruiseVelocity(motionMagicSettings.getMaxVelocity());
        talon.configMotionSCurveStrength(motionMagicSettings.getSmoothing());
    }

    @Override
    public void pidSet(double setpoint) {
        talon.configMotionAcceleration(motionMagicSettings.getTargetAcceleration());
        talon.configMotionCruiseVelocity(motionMagicSettings.getMaxVelocity());
        talon.configMotionSCurveStrength(motionMagicSettings.getSmoothing());
    }
}

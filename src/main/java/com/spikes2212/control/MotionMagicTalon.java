package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.function.Supplier;

/**
 * A {@link WPI_TalonSRX} that can use the Talon's MotionMagic feature.
 *
 * @author Tal Sitton
 */

public class MotionMagicTalon extends PIDTalon {

    private final MotionMagicSettings motionMagicSettings;

    /**
     * Constructs a MotionMagicTalon instance with the given parameters as field values.
     *
     * @param talon    the Talon speed controller on which the PID loop is calculated
     * @param pidfSettings the PID loop's {@link PIDFSettings}
     * @param timeout  the timeout of the motion magic
     */
    public MotionMagicTalon(WPI_TalonSRX talon, PIDFSettings pidfSettings, MotionMagicSettings mmSettings, int timeout) {
        super(talon, pidfSettings, ControlMode.MotionMagic, timeout);
        motionMagicSettings = mmSettings;
    }

    public MotionMagicTalon(WPI_TalonSRX talon, PIDFSettings pidfSettings, MotionMagicSettings mmSettings) {
        this(talon, pidfSettings, mmSettings, 30);
    }

    @Override
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed) {
        super.configureLoop(maxSpeed, minSpeed);
        talon.configMotionAcceleration(motionMagicSettings.getMaximumAcceleration());
        talon.configMotionCruiseVelocity(motionMagicSettings.getMaxVelocity());
        talon.configMotionSCurveStrength(motionMagicSettings.getSmoothing());
    }

    @Override
    public void pidSet(double setpoint) {
        talon.configMotionAcceleration(motionMagicSettings.getMaximumAcceleration());
        talon.configMotionCruiseVelocity(motionMagicSettings.getMaxVelocity());
        talon.configMotionSCurveStrength(motionMagicSettings.getSmoothing());
        super.pidSet(setpoint);
    }
}

package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.function.Supplier;

public class MotionMagicTalon extends PIDTalon {

    /**
     * Constructs a PIDTalon instance with the given parameters as field values.
     *
     * @param talon    The Talon speed controller on which the PID loop is calculated.
     * @param settings The PID loop's {@link PIDFSettings}.
     * @param mode     The PID loop's {@link ControlMode}.
     * @param timeout
     */
    public MotionMagicTalon(WPI_TalonSRX talon, PIDFSettings settings, ControlMode mode, int timeout) {
        super(talon, settings, mode, timeout);
    }
}

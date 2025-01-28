package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;

/**
 * A wrapper for the CTRE motor controllers and {@link CANSparkMax} control modes.
 *
 * @author Yoel Perman Brilliant
 * @see ControlMode
 * @see CANSparkMax.ControlType
 */
public enum UnifiedControlMode {
    POSITION(ControlMode.Position, CANSparkMax.ControlType.kPosition),
    VELOCITY(ControlMode.Velocity, CANSparkMax.ControlType.kVelocity),
    CURRENT(ControlMode.Current, CANSparkMax.ControlType.kCurrent),
    PERCENT_OUTPUT(ControlMode.PercentOutput, CANSparkMax.ControlType.kDutyCycle),
    TRAPEZOID_PROFILE(ControlMode.MotionMagic, CANSparkMax.ControlType.kSmartMotion),
    MOTION_PROFILING(ControlMode.MotionProfile, null),
    VOLTAGE(null, CANSparkMax.ControlType.kVoltage);

    private final ControlMode ctreControlMode;
    private final CANSparkMax.ControlType sparkMaxControlType;

    UnifiedControlMode(ControlMode ctreControlMode, CANSparkMax.ControlType sparkMaxControlType) {
        this.ctreControlMode = ctreControlMode;
        this.sparkMaxControlType = sparkMaxControlType;
    }

    public ControlMode getCTREControlMode() {
        return ctreControlMode;
    }

    public CANSparkMax.ControlType getSparkMaxControlType() {
        return sparkMaxControlType;
    }
}

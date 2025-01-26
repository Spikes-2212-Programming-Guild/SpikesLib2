package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkBase;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * A wrapper for the REV motor controllers and the {@link TalonFX} control modes.
 *
 * @author Yoel Perman Brilliant
 * @see ControlMode
 * @see SparkBase.ControlType
 */
public enum UnifiedControlMode {
    POSITION(ControlMode.Position, SparkBase.ControlType.kPosition),
    VELOCITY(ControlMode.Velocity, SparkBase.ControlType.kVelocity),
    CURRENT(ControlMode.Current, SparkBase.ControlType.kCurrent),
    PERCENT_OUTPUT(ControlMode.PercentOutput, SparkBase.ControlType.kDutyCycle),
    TRAPEZOID_PROFILE(ControlMode.MotionMagic, SparkBase.ControlType.kMAXMotionPositionControl),
    MOTION_PROFILING(ControlMode.MotionProfile, null),
    VOLTAGE(null, SparkBase.ControlType.kVoltage);

    private final ControlMode ctreControlMode;
    private final SparkBase.ControlType sparkControlType;

    UnifiedControlMode(ControlMode ctreControlMode, SparkBase.ControlType sparkControlType) {
        this.ctreControlMode = ctreControlMode;
        this.sparkControlType = sparkControlType;
    }

    public ControlMode getCTREControlMode() {
        return ctreControlMode;
    }

    public SparkBase.ControlType getSparkControlType() {
        return sparkControlType;
    }
}

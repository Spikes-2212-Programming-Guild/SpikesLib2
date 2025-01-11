package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

/**
 * A wrapper for the CTRE motor controllers and {@link SparkMax} control modes.
 *
 * @author Yoel Perman Brilliant
 * @see ControlMode
 * @see SparkMax.ControlType
 */
public enum UnifiedControlMode {
    POSITION(ControlMode.Position, SparkMax.ControlType.kPosition),
    VELOCITY(ControlMode.Velocity, SparkMax.ControlType.kVelocity),
    CURRENT(ControlMode.Current, SparkMax.ControlType.kCurrent),
    PERCENT_OUTPUT(ControlMode.PercentOutput, SparkMax.ControlType.kDutyCycle),
    TRAPEZOID_PROFILE(ControlMode.MotionMagic, SparkMax.ControlType.kMAXMotionPositionControl),
    MOTION_PROFILING(ControlMode.MotionProfile, null),
    VOLTAGE(null, SparkMax.ControlType.kVoltage);

    private final ControlMode ctreControlMode;
    private final SparkMax.ControlType sparkMaxControlType;

    UnifiedControlMode(ControlMode ctreControlMode, SparkMax.ControlType sparkMaxControlType) {
        this.ctreControlMode = ctreControlMode;
        this.sparkMaxControlType = sparkMaxControlType;
    }

    public ControlMode getCTREControlMode() {
        return ctreControlMode;
    }

    public SparkMax.ControlType getSparkMaxControlType() {
        return sparkMaxControlType;
    }
}

package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.spark.SparkBase;

/**
 * A wrapper for the REV motor controllers and the {@link TalonFX} control modes.
 *
 * @author Yoel Perman Brilliant
 * @see ControlMode
 * @see SparkBase.ControlType
 */
public enum UnifiedControlMode {

    POSITION(ControlModeValue.PositionDutyCycle, SparkBase.ControlType.kPosition),
    VELOCITY(ControlModeValue.VelocityDutyCycle, SparkBase.ControlType.kVelocity),
    CURRENT(ControlModeValue.TorqueCurrentFOC, SparkBase.ControlType.kCurrent),
    PERCENT_OUTPUT(ControlModeValue.DutyCycleOut, SparkBase.ControlType.kDutyCycle),
    TRAPEZOID_PROFILE(ControlModeValue.MotionMagicDutyCycle, SparkBase.ControlType.kMAXMotionPositionControl),
    VOLTAGE(ControlModeValue.VoltageOut, SparkBase.ControlType.kVoltage);

    private final ControlModeValue talonFXControlMode;
    private final SparkBase.ControlType sparkControlType;

    UnifiedControlMode(ControlModeValue talonFXControlMode, SparkBase.ControlType sparkControlType) {
        this.talonFXControlMode = talonFXControlMode;
        this.sparkControlType = sparkControlType;
    }

    public ControlModeValue getTalonFXControlMode() {
        return talonFXControlMode;
    }

    public SparkBase.ControlType getSparkControlType() {
        return sparkControlType;
    }
}

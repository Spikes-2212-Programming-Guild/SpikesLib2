package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A subsystem that runs control loops on an applicable motor controller.
 *
 * @author Yoel Perman Brilliant
 */
public interface SmartMotorControllerSubsystem extends Subsystem {

    /**
     * Configures the loop's PID constants and feed forward gains.
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    default void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
    }

    /**
     * Configures the loop's trapezoid profiling.
     * @param settings the trapezoid profile configurations
     */
    default void configureTrapezoid(TrapezoidProfileSettings settings) {
    }

    /**
     * configures the loop's settings
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        configPIDF(pidSettings, feedForwardSettings);
    }

    /**
     * configures the loop's settings
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                               TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on the motor controller
     *
     * @param controlType the loop's control type (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a Spark Max motor controller.
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    default void pidSet(CANSparkMax.ControlType controlType, double setpoint, PIDSettings pidSettings,
                        FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    /**
     * Updates any control loops running on the motor controller
     *
     * @param controlType the loop's control type (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a Spark Max motor controller.
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    default void pidSet(CANSparkMax.ControlType controlType, double setpoint,
                PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        pidSet(controlType, setpoint, pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    /**
     * Updates any control loops running on the motor controller
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a CTRE motor controller.
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    default void pidSet(ControlMode controlMode, double setpoint, PIDSettings pidSettings,
                FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    /**
     * Updates any control loops running on the motor controller
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a CTRE motor controller.
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    default void pidSet(ControlMode controlMode, double setpoint, PIDSettings pidSettings,
                        FeedForwardSettings feedForwardSettings) {
        pidSet(controlMode, setpoint, pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    /**
     * Stops running control loops on the motor controller.
     */
    default void finish() {
    }

    /**
     * Checks whether the loop is currently on the target setpoint.
     *
     * @return `true` when on target setpoint, `false` otherwise
     */
    default boolean onTarget(double setpoint) {
        return false;
    }

}

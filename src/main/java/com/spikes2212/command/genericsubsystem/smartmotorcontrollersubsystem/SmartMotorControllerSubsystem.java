package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} that runs control loops on an applicable motor controller.
 *
 * @author Yoel Perman Brilliant
 */
public interface SmartMotorControllerSubsystem extends Subsystem {

    /**
     * Configures the loop's PID constants and feed forward gains.
     */
    default void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
    }

    /**
     * Configures the loop's trapezoid profile settings.
     */
    default void configureTrapezoid(TrapezoidProfileSettings settings) {
    }

    /**
     * Configures the loop's settings.
     */
    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                               TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Configures the loop's settings.
     */
    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        configureLoop(pidSettings, feedForwardSettings, TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode              the loop's control type (e.g. voltage, velocity, position...)
     * @param setpoint                 the loop's target setpoint
     * @param pidSettings              the PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                        FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings);

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode         the loop's control type (e.g. voltage, velocity, position...)
     * @param setpoint            the loop's target setpoint
     * @param pidSettings         the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    default void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                        FeedForwardSettings feedForwardSettings) {
        pidSet(controlMode, setpoint, pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    /**
     * Stops any control loops running on the motor controller.
     */
    default void finish() {
    }

    /**
     * Checks whether the loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint);
}

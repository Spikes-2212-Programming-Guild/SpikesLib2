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

    default void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
    }

    default void configureTrapezoid(TrapezoidProfileSettings settings) {
    }

    /**
     * Configure the motor controller configurations and its control loops.
     *
     * @param pidSettings PID loop configuration settings
     * @param feedForwardSettings feed forward configuration settings
     */
    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        configPIDF(pidSettings, feedForwardSettings);
    }

    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                               TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    default void pidSet(CANSparkMax.ControlType controlType, double setpoint, PIDSettings pidSettings,
                        FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    /**
     * Update any control loops running on the motor controller.
     *
     * @param controlType the loop's control type (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a CTRE motor controller.
     * @param pidSettings PID loop configuration settings
     * @param feedForwardSettings feed forward configuration settings
     */
    default void pidSet(CANSparkMax.ControlType controlType, double setpoint,
                PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        pidSet(controlType, setpoint, pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    default void pidSet(ControlMode controlMode, double setpoint, PIDSettings pidSettings,
                FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    /**
     * Update any control loops running on the motor controller.
     *
     * @param controlMode the loop's control mode (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a CTRE motor controller.
     * @param pidSettings PID loop configuration settings
     * @param feedForwardSettings feed forward configuration settings
     */
    default void pidSet(ControlMode controlMode, double setpoint, PIDSettings pidSettings,
                        FeedForwardSettings feedForwardSettings) {
        pidSet(controlMode, setpoint, pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    /**
     * Stop running control loops on the motor controller.
     */
    default void finish() {
    }

    /**
     * Check whether the loop is currently on the target setpoint.
     *
     * @return `true` when on target setpoint, `false` otherwise
     */
    default boolean onTarget(double setpoint) {
        return false;
    }

    default void checkPIDAndFeedForward(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        if (pidSettings.isNull())
            throw new UnassignedPIDSettingsException();

        if (feedForwardSettings.isNull())
            throw new UnassignedFeedForwardSettingsException();
    }

    class UnassignedPIDSettingsException extends RuntimeException {

        public UnassignedPIDSettingsException() {
            super("PID Gains Were Not Assigned");
        }
    }

    class UnassignedFeedForwardSettingsException extends RuntimeException {

        public UnassignedFeedForwardSettingsException() {
            super("Feed Forward Gains Were Not Assigned");
        }
    }
}

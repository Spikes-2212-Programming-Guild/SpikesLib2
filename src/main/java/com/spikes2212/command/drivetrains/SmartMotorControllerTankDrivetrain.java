package com.spikes2212.command.drivetrains;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link TankDrivetrain} that runs control loops on an applicable motor controllers
 *
 * @author Yoel Perman Brilliant
 */
public interface SmartMotorControllerTankDrivetrain extends Subsystem {

    /**
     * Configures the loop's PID constants and feed forward gains.
     */
    default void configPIDF(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            FeedForwardSettings feedForwardSettings) {
    }

    /**
     * Configures the loop's trapezoid profile settings.
     */
    default void configureTrapezoid(TrapezoidProfileSettings settings) {
    }

    /**
     * Configures the loop's settings.
     */
    default void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                               FeedForwardSettings feedForwardSettings,
                               TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Configures the loop's settings.
     */
    default void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                               FeedForwardSettings feedForwardSettings) {
        configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode              the loop's control type (e.g. voltage, velocity, position...)
     * @param leftSetpoint             the left side loop's target setpoint
     * @param rightSetpoint            the right side loop's target setpoint
     * @param leftPIDSettings          the left side's PID constants
     * @param rightPIDSettings         the right side's PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint, PIDSettings leftPIDSettings,
                PIDSettings rightPIDSettings, FeedForwardSettings feedForwardSettings,
                TrapezoidProfileSettings trapezoidProfileSettings);

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode              the loop's control type (e.g. voltage, velocity, position...)
     * @param leftSetpoint             the left side loop's target setpoint
     * @param rightSetpoint            the right side loop's target setpoint
     * @param leftPIDSettings          the left side's PID constants
     * @param rightPIDSettings         the right side's PID constants
     * @param feedForwardSettings      the feed forward gains
     */
    default void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                        PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                        FeedForwardSettings feedForwardSettings) {
        pidSet(controlMode, leftSetpoint, rightSetpoint, leftPIDSettings, rightPIDSettings, feedForwardSettings,
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
     * @param controlMode   the loop's control type (e.g. voltage, velocity, position...)
     * @param leftTolerance the maximum difference from the left target to still be considered on target
     * @param rightTolerance the maximum difference from the right target to still be considered on target
     * @param leftSetpoint  the left side's wanted setpoint
     * @param rightSetpoint the right side's wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    default boolean onTarget(UnifiedControlMode controlMode, double leftTolerance, double rightTolerance,
                             double leftSetpoint, double rightSetpoint) {
        return leftOnTarget(controlMode, leftTolerance, leftSetpoint) &&
                rightOnTarget(controlMode, rightTolerance, rightSetpoint);
    }

    /**
     * Checks whether the left side's loop is currently on the target setpoint.
     *
     * @param controlMode   the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance     the maximum difference from the target to still be considered on target
     * @param setpoint      the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    boolean leftOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint);

    /**
     * Checks whether the right side's loop is currently on the target setpoint.
     *
     * @param controlMode   the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance     the maximum difference from the target to still be considered on target
     * @param setpoint      the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    boolean rightOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint);
}

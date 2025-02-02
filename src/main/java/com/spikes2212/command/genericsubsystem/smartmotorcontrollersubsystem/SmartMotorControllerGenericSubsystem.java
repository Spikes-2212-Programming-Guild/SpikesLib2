package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;

/**
 * A {@link Subsystem} that runs control loops on an applicable motor controller.
 *
 * @author Yoel Perman Brilliant
 */
public class SmartMotorControllerGenericSubsystem extends MotoredGenericSubsystem {

    private final List<? extends SmartMotorController> motorControllers;

    public SmartMotorControllerGenericSubsystem(String namespaceName, SmartMotorController... motorControllers) {
        super(namespaceName, motorControllers);
        this.motorControllers = List.of(motorControllers);
    }

    /**
     * Configures the loop's PID constants.
     */
    public void configurePID(PIDSettings pidSettings) {
        motorControllers.forEach(smartMotorController -> smartMotorController.configurePID(pidSettings));
    }

    /**
     * Configures the loop's feed forward gains.
     */
    public void configureFF(FeedForwardSettings feedForwardSettings) {
        motorControllers.forEach(smartMotorController -> smartMotorController.configureFF(feedForwardSettings));
    }

    /**
     * Configures the loop's trapezoid profile settings.
     */
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        motorControllers.forEach(smartMotorController -> smartMotorController.configureTrapezoid(settings));
    }

    /**
     * Configures the loop's settings.
     */
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        configurePID(pidSettings);
        configureFF(feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Configures the loop's settings.
     */
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
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
     * @param updatePeriodically       whether to update the loop's settings periodically
     */
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean updatePeriodically) {
        motorControllers.forEach(motorController -> motorController.pidSet(controlMode, setpoint, pidSettings,
                feedForwardSettings, trapezoidProfileSettings, updatePeriodically));
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode         the loop's control type (e.g. voltage, velocity, position...)
     * @param setpoint            the loop's target setpoint
     * @param acceleration        the loop's target acceleration
     * @param pidSettings         the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param updatePeriodically  whether to update the loop's settings periodically
     */
    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        motorControllers.forEach(motorController -> motorController.pidSet(controlMode, setpoint, acceleration,
                pidSettings, feedForwardSettings, updatePeriodically));
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode         the loop's control type (e.g. voltage, velocity, position...)
     * @param setpoint            the loop's target setpoint
     * @param pidSettings         the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param updatePeriodically  whether to update the loop's settings periodically
     */
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        motorControllers.forEach(motorController -> motorController.pidSet(controlMode, setpoint, 0,
                pidSettings, feedForwardSettings, updatePeriodically));
    }

    /**
     * Stops any control loops running on the motor controller.
     */
    public void finish() {
        motorControllers.forEach(motorControllers -> motorController.stopMotor());
    }

    /**
     * Checks whether the loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        return motorControllers.stream().allMatch(motorController -> motorController.onTarget(controlMode, tolerance,
                setpoint));
    }
}

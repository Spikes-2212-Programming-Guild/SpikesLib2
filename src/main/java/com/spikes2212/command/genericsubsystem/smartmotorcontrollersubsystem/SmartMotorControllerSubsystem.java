package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A subsystem that runs control loops on a Talon motor controller.
 *
 * @author Eran Goldstein
 */
public interface SmartMotorControllerSubsystem extends Subsystem {

    /**
     * Configure the motor controller and its control loops.
     */
    void configureLoop(int slot);

    /**
     * Update any control loops running on the motor controller.
     */
    void pidSet(int slot, double setpoint);

    /**
     * Stop running control loops on the motor controller.
     */
    void finish();

    /**
     * Check whether the loop is currently on the target setpoint.
     *
     * @return `true` when on target setpoint, `false` otherwise
     */
    boolean onTarget(int slot, double setpoint);

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

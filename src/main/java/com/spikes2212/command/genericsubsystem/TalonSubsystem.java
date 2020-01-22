package com.spikes2212.command.genericsubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A subsystem that runs control loops on a Talon motor controller.
 *
 * @author Eran Goldstein
 */
public interface TalonSubsystem extends Subsystem {
    /**
     * Configure the Talon motor controller and its control loops.
     */
    void config();

    /**
     * Update any control loops running on the Talon.
     */
    void update();

    /**
     * Change the `setpoint` the subsystem is aiming towards.
     *
     * @param setpoint the new setpoint to aim towards
     */
    void setSetpoint(double setpoint);

    /**
     * Stop running control loops on the Talon.
     */
    void finish();
}

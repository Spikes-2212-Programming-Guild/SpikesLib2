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
    void configureLoop();

    /**
     * Update any control loops running on the Talon.
     */
    void pidSet(double setpoint);

    /**
     * Stop running control loops on the Talon.
     */
    void finish();

    /**
     * Check whether the loop is currently on the target setpoint.
     *
     * @return `true` when on target setpoint, `false` otherwise
     */
    boolean onTarget(double setpoint);
}

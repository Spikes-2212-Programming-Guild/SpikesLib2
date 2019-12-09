package com.spikes2212.control;

/**
 * An interface for PID loops.
 *
 * @author Eran Goldstein
 */
public interface PIDLoop {
    /**
     * Starts the PID loop.
     */
    void enable();

    /**
     * Stops the PID loop.
     */
    void disable();

    /**
     * Update the setpoint of the loop.
     */
    void update();

    /**
     * Check whether the loop is currently within the target range.
     *
     * @return true when within target range, false otherwise
     */
    boolean onTarget();
}

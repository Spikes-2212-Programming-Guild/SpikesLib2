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
     * Updates the PID loop.
     */
    void update();

    /**
     * Check whether the loop has been within the target range for at least `waitTime` seconds.
     *
     * @return `true` when within target range for `waitTime`, `false` otherwise
     */
    boolean onTarget();
}

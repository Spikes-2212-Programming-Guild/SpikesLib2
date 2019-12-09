package com.spikes2212.control;

/**
 * An interface for PID loop controllers.
 *
 * @author Eran Goldstein
 */
public interface PIDLoop {
    /**
     * Starts running the PID loop.
     */
    void enable();

    /**
     * Stops running the PID loop.
     */
    void disable();
}

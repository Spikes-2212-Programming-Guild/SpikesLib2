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
}

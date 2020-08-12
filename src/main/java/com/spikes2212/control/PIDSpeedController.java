package com.spikes2212.control;

import edu.wpi.first.wpilibj.SpeedController;

import java.util.function.Supplier;

/**
 * A speed controller that can run a PID loop.
 *
 * @author Eran Goldstein
 */
public interface PIDSpeedController extends SpeedController {
    /**
     * Configures this speed controller's PID constants as well as others that help maintain safety.
     *
     * @param maxSpeed A {@link Supplier} that returns the maximum speed allowed for the PID loop.
     * @param minSpeed A {@link Supplier} that returns the minimum speed allowed for the PID loop.
     *                 Any process that changes the speed controller's configurations will time out after that many ms.
     *                 Use 0 if you want to have no time limit on such processes.
     */
    void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed);

    /**
     * Sets the speed controller to move as dictated by the PID calculation.
     *
     * @param setpoint The PID loop's setpoint.
     *                 Any process that changes the speed controller's configurations will time out after that many ms.
     *                 Use 0 if you want to have no time limit on such processes.
     */
    void pidSet(double setpoint);

    /**
     * Returns whether the PID loop is close enough to the target setpoint.
     *
     * @param setpoint The target setpoint.
     * @return Whether the PID loop is close enough to the target setpoint.
     */
    boolean onTarget(double setpoint);

    double getWaitTime();

    /**
     * Finishing the PID loop. Put any necessary finalization code here.
     */
    void finish();
}

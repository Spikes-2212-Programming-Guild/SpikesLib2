package com.spikes2212.command.genericsubsystem;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} that runs control loops on a Talon motor controller. <br>
 * This interface is deprecated. Please use {@link SmartMotorControllerSubsystem} instead.
 *
 * @author Eran Goldstein
 */
@Deprecated(since = "2022", forRemoval = true)
public interface TalonSubsystem extends Subsystem {

    /**
     * Configures the Talon motor controller and its control loops.
     */
    void configureLoop();

    /**
     * Updates any control loops running on the Talon.
     */
    void pidSet(double setpoint);

    /**
     * Stops running control loops on the Talon.
     */
    void finish();

    /**
     * Checks whether the loop is currently on the target setpoint.
     *
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    boolean onTarget(double setpoint);
}

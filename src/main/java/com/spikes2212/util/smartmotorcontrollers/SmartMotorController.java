package com.spikes2212.util.smartmotorcontrollers;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * A {@link MotorController} that can run control loops.
 *
 * @author Camellia Lami
 */
public interface SmartMotorController extends MotorController {

    /**
     * Configures the loop's PID constants.
     */
    void configurePID(PIDSettings pidSettings);

    /**
     * Configures the loop's feed forward gains.
     */
    void configureFF(FeedForwardSettings feedForwardSettings);


    /**
     * Configures the loop's trapezoid profile settings.
     */
    void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings);


    /**
     * Configures the loop's settings.
     */
    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                               TrapezoidProfileSettings trapezoidProfileSettings) {
        configurePID(pidSettings);
        configureFF(feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Sets the encoder's position.
     */
    void setPosition(double position);

    /**
     * Resets the encoder's position to 0.
     */
    default void resetPosition() {
        setPosition(0);
    }

    /**
     * Updates any control loops running on the drivetrain's motor controllers.
     *
     * @param controlMode              the loop's control mode
     * @param setpoint                 the loop's target
     * @param pidSettings              the PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     * @param updatePeriodically       whether to update the loop's constants periodically
     */
    void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                boolean updatePeriodically);

    /**
     * Updates any control loops running on the drivetrain's motor controllers.
     *
     * @param controlMode         the loop's control mode
     * @param setpoint            the loop's target
     * @param acceleration        the desired acceleration
     * @param pidSettings         the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param updatePeriodically  whether to update the loop's constants periodically
     */
    void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                FeedForwardSettings feedForwardSettings, boolean updatePeriodically);

    /**
     * Updates any control loops running on the drivetrain's motor controllers.
     *
     * @param controlMode         the loop's control mode
     * @param setpoint            the loop's target
     * @param pidSettings         the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param updatePeriodically  whether to update the loop's constants periodically
     */
    void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                FeedForwardSettings feedForwardSettings, boolean updatePeriodically);

    /**
     * Checks whether the loops are currently on the target setpoints.
     *
     * @param controlMode the loop's control type
     * @param tolerance   the maximum difference from the target to still consider the loop to be on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint);

    double getPosition();

    double getVelocity();
}

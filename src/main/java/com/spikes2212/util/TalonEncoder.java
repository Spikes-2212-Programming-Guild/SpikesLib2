package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;


/**
 * This class represents an encoder that is connected to a Talon motor controller.
 *
 * @author Tuval
 */
public class TalonEncoder {
    /**
     * The Talon this encoder is connected to.
     */
    private BaseTalon talon;

    /**
     * The distance the subsystem with this encoder passes with every encoder tick.
     */
    private double distancePerPulse;

    /**
     * Constructs the {@link TalonEncoder} using the talon and the number of counts per
     * revolution of the motor.
     *
     * @param talon            The talon the encoder is connected to.
     * @param distancePerPulse Counts per revolution of the motor. Can be used to change the
     *                         scale of the value of the encoder.
     */
    public TalonEncoder(BaseTalon talon, double distancePerPulse) {
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        talon.setSensorPhase(true);

        this.talon = talon;
        this.distancePerPulse = distancePerPulse;
    }

    /**
     * Constructs the {@link TalonEncoder} using the talon. The {@link TalonEncoder} will return the raw
     * value of the encoder.
     *
     * @param talon The talon the encoder is connected to.
     */
    public TalonEncoder(BaseTalon talon) {
        this(talon, 1);
    }

    public double getDistancePerPulse() {
        return distancePerPulse;
    }

    public void setDistancePerPulse(double distancePerPulse) {
        this.distancePerPulse = distancePerPulse;
    }

    public double getVelocity() {
        return talon.getSelectedSensorVelocity() * distancePerPulse;
    }

    public double getPosition() {
        return talon.getSelectedSensorPosition() * distancePerPulse;
    }

    public void reset() {
        talon.setSelectedSensorPosition(0);
    }
}

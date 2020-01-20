package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 * This class makes an encoder that is connected to a {@link WPI_TalonSRX} to a
 * PIDSource.
 *
 * @author Tuval
 */
public class TalonEncoder {

    private BaseTalon talon;
    private double distancePerPulse;

    /**
     * Constructs the PIDSource using the talon and the number of counts per
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
     * Constructs the PIDSource using the talon. The source will return the raw
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
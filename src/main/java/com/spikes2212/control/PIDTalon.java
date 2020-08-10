package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

public class PIDTalon {

    /**
     * The {@link WPI_TalonSRX} on which the PID is calculated.
     */
    private WPI_TalonSRX talon;
    /**
     * The PID loop's {@link PIDSettings}.
     */
    private PIDSettings settings;
    /**
     * The PID loop's feed forward constant.
     */
    private Supplier<Double> kF;
    /**
     * The PID loop's {@link ControlMode}.
     */
    private ControlMode mode;

    /**
     * Constructs a PIDTalon instance with the given parameters as field values.
     *
     * @param talon The Talon speed controller on which the PID loop is calculated.
     * @param settings The PID loop's {@link PIDSettings}.
     * @param kF The PID loop's feed forward constant.
     * @param mode The PID loop's {@link ControlMode}.
     */
    public PIDTalon(WPI_TalonSRX talon, PIDSettings settings, Supplier<Double> kF, ControlMode mode) {
        this.talon = talon;
        this.settings = settings;
        this.kF= kF;
        this.mode = mode;
    }

    /**
     * Configures the Talon speed controller's PID constants as well as others that help maintain safety.
     * @param maxSpeed A {@link Supplier} that returns the maximum speed allowed for the PID loop.
     * @param minSpeed A {@link Supplier} that returns the minimum speed allowed for the PID loop.
     * @param timeout The Talon's timeout.
     *                Any process that changes the Talon's configurations will time out after that many ms.
     *                Use 0 if you want to have no time limit on such processes.
     */
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed, int timeout) {
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);

        talon.configNominalOutputForward(0, timeout);
        talon.configNominalOutputReverse(0, timeout);
        talon.configPeakOutputForward(maxSpeed.get(), timeout);
        talon.configPeakOutputReverse(minSpeed.get(), timeout);

        talon.configAllowableClosedloopError(0, 0, timeout);
        talon.config_kP(0, settings.getkP(), timeout);
        talon.config_kI(0, settings.getkI(), timeout);
        talon.config_kD(0, settings.getkD(), timeout);
        talon.config_kF(0, kF.get(), timeout);
    }

    /**
     * Sets the Talon speed controller to move as dictated by the PID calculation.
     * @param setpoint The PID loop's setpoint.
     * @param timeout The Talon's timeout.
     *                Any process that changes the Talon's configurations will time out after that many ms.
     *                Use 0 if you want to have no time limit on such processes.
     */
    public void pidSet(double setpoint, int timeout) {
        talon.config_kP(0, settings.getkP(), timeout);
        talon.config_kI(0, settings.getkI(), timeout);
        talon.config_kD(0, settings.getkD(), timeout);
        talon.config_kF(0, kF.get(), timeout);
        talon.set(mode, setpoint);
    }

    /**
     * Finishing the PID loop. Put any necessary finalization code here.
     */
    public void finish() {
        talon.stopMotor();
    }

    /**
     * Returns whether the PID loop is close enough to the target setpoint.
     * @param setpoint The target setpoint.
     * @return Whether the PID loop is close enough to the target setpoint.
     */
    public boolean onTarget(double setpoint) {
        return Math.abs(setpoint - talon.getSelectedSensorPosition()) <= settings.getTolerance();
    }

    /**
     * Returns the {@link WPI_TalonSRX} on which the PID is calculated.
     * @return The {@link WPI_TalonSRX} on which the PID is calculated.
     */
    public WPI_TalonSRX getTalon() {
        return talon;
    }

    /**
     * Returns the PID loop's {@link PIDSettings}.
     * @return The PID loop's {@link PIDSettings}.
     */
    public PIDSettings getSettings() {
        return settings;
    }
}

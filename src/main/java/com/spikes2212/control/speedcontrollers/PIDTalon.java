package com.spikes2212.control.speedcontrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

/**
 * A {@link WPI_TalonSRX} that can run a PID loop.
 *
 * @author Tuval Rivkind Barlev
 */
public class PIDTalon implements ClosedLoopSpeedController {

    /**
     * The {@link WPI_TalonSRX} on which the PID is calculated.
     */
    private final WPI_TalonSRX talon;

    /**
     * The PID loop's {@link PIDSettings}.
     */
    private final PIDSettings settings;

    /**
     * The PID loop's feed forward constant.
     */
    private final Supplier<Double> kF;

    /**
     * The PID loop's {@link ControlMode}.
     */
    private final ControlMode mode;

    /**
     * The talon's timeout.
     */
    private int timeout;

    /**
     * Constructs a PIDTalon instance with the given parameters as field values.
     *
     * @param talon    The Talon speed controller on which the PID loop is calculated.
     * @param settings The PID loop's {@link PIDSettings}.
     * @param kF       The PID loop's feed forward constant.
     * @param mode     The PID loop's {@link ControlMode}.
     */
    public PIDTalon(WPI_TalonSRX talon, PIDSettings settings, Supplier<Double> kF, ControlMode mode, int timeout) {
        this.talon = talon;
        this.settings = settings;
        this.kF = kF;
        this.mode = mode;
        this.timeout = timeout;
    }

    @Override
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed) {
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

    @Override
    public void pidSet(double setpoint) {
        talon.config_kP(0, settings.getkP(), timeout);
        talon.config_kI(0, settings.getkI(), timeout);
        talon.config_kD(0, settings.getkD(), timeout);
        talon.config_kF(0, kF.get(), timeout);
        talon.set(mode, setpoint);
    }

    @Override
    public boolean onTarget(double setpoint) {
        return Math.abs(setpoint - talon.getSelectedSensorPosition()) <= settings.getTolerance();
    }

    @Override
    public double getWaitTime() {
        return settings.getWaitTime();
    }

    @Override
    public void finish() {
        talon.stopMotor();
    }

    @Override
    public void set(double speed) {
        talon.set(speed);
    }

    @Override
    public double get() {
        return talon.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        talon.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return talon.getInverted();
    }

    @Override
    public void disable() {
        talon.disable();
    }

    @Override
    public void stopMotor() {
        talon.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        talon.pidWrite(output);
    }
}

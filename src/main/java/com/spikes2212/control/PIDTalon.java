package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.function.Supplier;

/**
 * A {@link WPI_TalonSRX} that can run a PID loop.
 *
 * @author Tuval Rivkind Barlev
 */
public class PIDTalon implements PIDSpeedController {

    /**
     * The {@link WPI_TalonSRX} on which the PID is calculated.
     */
    protected final WPI_TalonSRX talon;

    /**
     * The PID loop's {@link PIDFSettings}.
     */
    private final PIDFSettings settings;

    /**
     * The PID loop's {@link ControlMode}.
     */
    private final ControlMode mode;

    /**
     * The talon's timeout.
     */
    protected int timeout;

    /**
     * Constructs a PIDTalon instance with the given parameters as field values.
     *
     * @param talon    The Talon speed controller on which the PID loop is calculated.
     * @param settings The PID loop's {@link PIDFSettings}.
     * @param mode     The PID loop's {@link ControlMode}.
     */
    public PIDTalon(WPI_TalonSRX talon, PIDFSettings settings, ControlMode mode, int timeout) {
        this.talon = talon;
        this.settings = settings;
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
        talon.config_kF(0, settings.getkF(), timeout);
    }

    @Override
    public void pidSet(double setpoint) {
        talon.config_kP(0, settings.getkP(), timeout);
        talon.config_kI(0, settings.getkI(), timeout);
        talon.config_kD(0, settings.getkD(), timeout);
        talon.config_kF(0, settings.getkF(), timeout);
        talon.set(mode, setpoint);
    }

    @Override
    public boolean onTarget(double setpoint) {
        switch (mode) {
            case Position:
            case MotionMagic:
            case MotionProfile:
            case MotionProfileArc:
            case PercentOutput:
                return Math.abs(setpoint - talon.getSelectedSensorPosition()) <= settings.getTolerance();
            case Velocity:
                return Math.abs(setpoint - talon.getSelectedSensorVelocity()) <= settings.getTolerance();
            default:
                throw new IllegalArgumentException("unknown mode");
        }
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

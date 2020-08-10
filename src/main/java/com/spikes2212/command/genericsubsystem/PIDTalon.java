package com.spikes2212.command.genericsubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.spikes2212.control.PIDSettings;

import java.util.function.Supplier;

public class PIDTalon {

    private WPI_TalonSRX talon;
    private PIDSettings settings;
    private Supplier<Double> kF;
    private ControlMode mode;

    public PIDTalon(WPI_TalonSRX talon, PIDSettings settings, Supplier<Double> kF, ControlMode mode) {
        this.talon = talon;
        this.settings = settings;
        this.kF= kF;
        this.mode = mode;
    }

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

    public void pidSet(double setpoint, int timeout) {
        talon.config_kP(0, settings.getkP(), timeout);
        talon.config_kI(0, settings.getkI(), timeout);
        talon.config_kD(0, settings.getkD(), timeout);
        talon.config_kF(0, kF.get(), timeout);
        talon.set(mode, setpoint);
    }

    public void finish() {
        talon.stopMotor();
    }

    public boolean onTarget(double setpoint) {
        return Math.abs(setpoint - talon.getSelectedSensorPosition()) <= settings.getTolerance();
    }

    public WPI_TalonSRX getTalon() {
        return talon;
    }

    public PIDSettings getSettings() {
        return settings;
    }
}

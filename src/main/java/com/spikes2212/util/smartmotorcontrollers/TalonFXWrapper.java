package com.spikes2212.util.smartmotorcontrollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

import java.time.Clock;

public class TalonFXWrapper extends CoreTalonFX implements SmartMotorController {

    private TalonFX talonFX;
    private boolean inverted;

    public TalonFXWrapper(int deviceId, String canbus) {
        super(-deviceId, canbus);
        talonFX = new TalonFX(deviceId);
        inverted = false;
    }

    public TalonFXWrapper(int deviceId, CANBus canbus) {
        this(deviceId, canbus.getName());
    }

    public TalonFXWrapper(int deviceId) {
        this(deviceId, "");
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        Slot0Configs config = new Slot0Configs();
        config.kP = pidSettings.getkP();
        config.kI = pidSettings.getkI();
        config.kD = pidSettings.getkD();
        talonFX.getConfigurator().apply(config);
    }

    @Override
    public void configureFF(FeedForwardSettings feedForwardSettings) {
        Slot0Configs config = new Slot0Configs();
        config.kS = feedForwardSettings.getkS();
        config.kV = feedForwardSettings.getkV();
        config.kA = feedForwardSettings.getkA();
        config.kG = feedForwardSettings.getkG();
        talonFX.getConfigurator().apply(config);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = trapezoidProfileSettings.getAccelerationRate();
        config.MotionMagicCruiseVelocity = trapezoidProfileSettings.getMaxVelocity();
        config.MotionMagicJerk = trapezoidProfileSettings.getCurve();
        talonFX.getConfigurator().apply(config);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    @Override
    public void set(double speed) {
        talonFX.set(speed);
    }

    @Override
    public double get() {
        return talonFX.get();
    }

    @Override
    public void setInverted(boolean inverted) {
        talonFX.getConfigurator().apply(new MotorOutputConfigs().withInverted(inverted ?
                InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive));
        this.inverted = inverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void disable() {
        talonFX.disable();
    }

    @Override
    public void stopMotor() {
        talonFX.stopMotor();
    }
}

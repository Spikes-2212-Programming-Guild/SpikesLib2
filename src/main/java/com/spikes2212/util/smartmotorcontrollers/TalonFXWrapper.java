package com.spikes2212.util.smartmotorcontrollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
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
        configurePID(pidSettings);
        configureFF(feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        ControlRequest request = switch (controlMode) {
            case CURRENT -> new TorqueCurrentFOC(setpoint);
            case PERCENT_OUTPUT -> new DutyCycleOut(setpoint);
            case TRAPEZOID_PROFILE -> new MotionMagicDutyCycle(setpoint);
            case MOTION_PROFILING -> throw new UnsupportedOperationException("Motion Profiling is not yet implemented in SpikesLib2!");
            case VOLTAGE -> new VoltageOut(setpoint);
            case VELOCITY -> new VelocityDutyCycle(setpoint);
            case POSITION -> new PositionDutyCycle(setpoint);
        };
        talonFX.setControl(request);
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
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
        this.inverted = inverted;
    }

    @Override
    public boolean getInverted() {
        //@TODO figure out a better way to do this
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

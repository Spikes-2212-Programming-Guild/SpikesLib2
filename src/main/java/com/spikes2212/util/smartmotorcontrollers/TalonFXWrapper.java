package com.spikes2212.util.smartmotorcontrollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

public class TalonFXWrapper implements SmartMotorController {

    protected final TalonFX talonFX;
    protected final Slot0Configs closedLoopConfig;
    protected final MotorOutputConfigs motorOutputConfigs;

    public TalonFXWrapper(int deviceId, String canbus) {
        talonFX = new TalonFX(deviceId, canbus);
        closedLoopConfig = new Slot0Configs();
        motorOutputConfigs = new MotorOutputConfigs();
    }

    public TalonFXWrapper(int deviceId, CANBus canbus) {
        this(deviceId, canbus.getName());
    }

    public TalonFXWrapper(int deviceId) {
        this(deviceId, "");
    }

    public void setIdleMode(NeutralModeValue neutralModeValue) {
        talonFX.getConfigurator().apply(motorOutputConfigs.withNeutralMode(neutralModeValue));
    }

    public TalonFXConfigurator getConfigurator() {
        return talonFX.getConfigurator();
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        closedLoopConfig.kP = pidSettings.getkP();
        closedLoopConfig.kI = pidSettings.getkI();
        closedLoopConfig.kD = pidSettings.getkD();
        talonFX.getConfigurator().apply(closedLoopConfig);
    }

    @Override
    public void configureFF(FeedForwardSettings feedForwardSettings) {
        closedLoopConfig.kS = feedForwardSettings.getkS();
        closedLoopConfig.kV = feedForwardSettings.getkV();
        closedLoopConfig.kA = feedForwardSettings.getkA();
        closedLoopConfig.kG = feedForwardSettings.getkG();
        talonFX.getConfigurator().apply(closedLoopConfig);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = trapezoidProfileSettings.getMaxAcceleration();
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

    public void follow(TalonFX master, boolean invert) {
        talonFX.setControl(new Follower(master.getDeviceID(), invert));
    }

    public void follow(TalonFXWrapper master, boolean invert) {
        follow(master.talonFX, invert);
    }

    public void follow(TalonFX master) {
        follow(master, false);
    }

    public void follow(TalonFXWrapper master) {
        follow(master.talonFX);
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
        talonFX.getConfigurator().apply(motorOutputConfigs.withInverted(inverted ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
    }

    @Override
    public boolean getInverted() {
        return motorOutputConfigs.Inverted == InvertedValue.Clockwise_Positive;
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

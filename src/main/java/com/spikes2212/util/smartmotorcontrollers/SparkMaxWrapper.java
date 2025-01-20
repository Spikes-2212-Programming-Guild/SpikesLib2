package com.spikes2212.util.smartmotorcontrollers;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

public class SparkMaxWrapper extends SparkMax implements SmartMotorController {

    private final FeedForwardController feedForwardController;

    private SparkMaxConfig sparkConfig;
    private ClosedLoopConfig closedLoopConfig;

    public SparkMaxWrapper(int deviceID, MotorType type, FeedForwardController.ControlMode controlMode) {
        super(deviceID, type);
        feedForwardController = new FeedForwardController(new FeedForwardSettings(controlMode));
        sparkConfig = new SparkMaxConfig();
        closedLoopConfig = new ClosedLoopConfig();
    }

    public SparkMaxWrapper(int deviceID, MotorType type) {
        this(deviceID, type, FeedForwardController.ControlMode.LINEAR_VELOCITY);
    }

    public SparkMaxConfig getSparkConfiguration() {
        return sparkConfig;
    }

    public void applyConfiguration(SparkMaxConfig newConfig) {
        sparkConfig.apply(newConfig);
        configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public ClosedLoopConfig getClosedLoopConfiguration() {
        return closedLoopConfig;
    }

    public void applyClosedLoopConfig(ClosedLoopConfig newConfig) {
        closedLoopConfig.apply(newConfig);
        configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void restoreFactoryDefaults() {
        sparkConfig = new SparkMaxConfig();
        closedLoopConfig = new ClosedLoopConfig();
        configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setInverted(boolean inverted) {
        int leaderID = configAccessor.getFollowerModeLeaderId();
        if (leaderID != 0) {
            sparkConfig.follow(leaderID, inverted);
        } else sparkConfig.inverted(inverted);
        configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void follow(SparkMax master) {
        setIdleMode(master.configAccessor.getIdleMode());
        configure(sparkConfig.follow(master, configAccessor.getInverted()), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void unfollow() {
        sparkConfig.disableFollowerMode();
        configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setIdleMode(SparkBaseConfig.IdleMode idleMode) {
        sparkConfig.idleMode(idleMode);
        configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        closedLoopConfig.pid(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void configureFF(FeedForwardSettings feedForwardSettings) {
        feedForwardController.setGains(feedForwardSettings);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
        maxMotionConfig.maxVelocity(trapezoidProfileSettings.getMaxVelocity()).
                maxAcceleration(trapezoidProfileSettings.getMaxAcceleration());
        closedLoopConfig.apply(maxMotionConfig);
        // @TODO add s-curve when rev implements it
        configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        double source;
        if (feedForwardSettings.getControlMode() == FeedForwardController.ControlMode.LINEAR_POSITION ||
                feedForwardSettings.getControlMode() == FeedForwardController.ControlMode.ANGULAR_POSITION) {
            source = getEncoder().getPosition();
        } else {
            source = getEncoder().getVelocity();
        }
        getClosedLoopController().setReference(setpoint, controlMode.getSparkMaxControlType(), ClosedLoopSlot.kSlot0,
                feedForwardController.calculate(source, setpoint, acceleration));
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        pidSet(controlMode, setpoint, 0, pidSettings, feedForwardSettings, trapezoidProfileSettings);
    }
}

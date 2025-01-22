package com.spikes2212.util.smartmotorcontrollers;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

public class SparkWrapper implements SmartMotorController {

    private final FeedForwardController feedForwardController;

    private SparkBase sparkBase;
    private SparkBaseConfigAccessor configAccessor;
    private SparkBaseConfig sparkConfig;
    private ClosedLoopConfig closedLoopConfig;
    private EncoderConfig encoderConfig;

    public static SparkWrapper createSparkMax(int deviceID, SparkLowLevel.MotorType type,
                                              FeedForwardController.ControlMode controlMode) {
        SparkWrapper sparkWrapper = new SparkWrapper(controlMode);
        sparkWrapper.sparkBase = new SparkMax(deviceID, type);
        sparkWrapper.sparkConfig = new SparkMaxConfig();
        sparkWrapper.configAccessor = ((SparkMax) (sparkWrapper.sparkBase)).configAccessor;
        return sparkWrapper;
    }

    public static SparkWrapper createSparkMax(int deviceID, SparkLowLevel.MotorType type) {
        return createSparkMax(deviceID, type, FeedForwardController.ControlMode.LINEAR_VELOCITY);
    }

    public static SparkWrapper createSparkFlex(int deviceID, SparkLowLevel.MotorType type,
                                               FeedForwardController.ControlMode controlMode) {
        SparkWrapper sparkWrapper = new SparkWrapper(controlMode);
        sparkWrapper.sparkBase = new SparkFlex(deviceID, type);
        sparkWrapper.sparkConfig = new SparkFlexConfig();
        sparkWrapper.configAccessor = ((SparkFlex) (sparkWrapper.sparkBase)).configAccessor;
        return sparkWrapper;
    }

    public static SparkWrapper createSparkFlex(int deviceID, SparkLowLevel.MotorType type) {
        return createSparkFlex(deviceID, type, FeedForwardController.ControlMode.LINEAR_VELOCITY);
    }

    private SparkWrapper(FeedForwardController.ControlMode controlMode) {
        feedForwardController = new FeedForwardController(new FeedForwardSettings(controlMode));
        closedLoopConfig = new ClosedLoopConfig();
        encoderConfig = new EncoderConfig();
    }

    public SparkBaseConfig getSparkConfiguration() {
        return sparkConfig;
    }

    public void applyConfiguration(SparkBaseConfig newConfig) {
        sparkConfig.apply(newConfig);
        sparkBase.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public ClosedLoopConfig getClosedLoopConfiguration() {
        return closedLoopConfig;
    }

    public void applyClosedLoopConfig(ClosedLoopConfig newConfig) {
        closedLoopConfig.apply(newConfig);
        sparkBase.configure(sparkConfig.apply(closedLoopConfig), SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public void restoreFactoryDefaults() {
        if (sparkBase instanceof SparkMax) sparkConfig = new SparkMaxConfig();
        else if (sparkBase instanceof SparkFlex) sparkConfig = new SparkFlexConfig();
        closedLoopConfig = new ClosedLoopConfig();
        encoderConfig = new EncoderConfig();
        sparkBase.configure(sparkConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void set(double speed) {
        sparkBase.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        sparkBase.setVoltage(voltage);
    }

    @Override
    public double get() {
        return sparkBase.get();
    }

    @Override
    public void setInverted(boolean inverted) {
        int leaderID = configAccessor.getFollowerModeLeaderId();
        if (leaderID != 0) {
            sparkConfig.follow(leaderID, inverted);
        } else sparkConfig.inverted(inverted);
        sparkBase.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean getInverted() {
        return configAccessor.getInverted();
    }

    @Override
    public void disable() {
        sparkBase.disable();
    }

    @Override
    public void stopMotor() {
        sparkBase.stopMotor();
    }

    public void follow(SparkWrapper master) {
        setIdleMode(master.configAccessor.getIdleMode());
        sparkBase.configure(sparkConfig.follow(master.sparkBase, getInverted()),
                SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void unfollow() {
        sparkConfig.disableFollowerMode();
        sparkBase.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public void setIdleMode(SparkBaseConfig.IdleMode idleMode) {
        sparkConfig.idleMode(idleMode);
        sparkBase.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        closedLoopConfig.pid(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        closedLoopConfig.iZone(pidSettings.getIZone());
        sparkBase.configure(sparkConfig.apply(closedLoopConfig), SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
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
        // @TODO add s-curve when REV implements it
        sparkBase.configure(sparkConfig.apply(closedLoopConfig), SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean configurePeriodically) {
        if (configurePeriodically) configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        double source;
        if (feedForwardSettings.getControlMode() == FeedForwardController.ControlMode.LINEAR_POSITION ||
                feedForwardSettings.getControlMode() == FeedForwardController.ControlMode.ANGULAR_POSITION) {
            source = sparkBase.getEncoder().getPosition();
        } else {
            source = sparkBase.getEncoder().getVelocity();
        }
        sparkBase.getClosedLoopController().setReference(setpoint, controlMode.getSparkMaxControlType(),
                ClosedLoopSlot.kSlot0, feedForwardController.calculate(source, setpoint, acceleration));
    }

    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean configurePeriodically) {
        pidSet(controlMode, setpoint, 0, pidSettings, feedForwardSettings, trapezoidProfileSettings,
                configurePeriodically);
    }

    public void setPositionConversionFactor(double factor) {
        encoderConfig.positionConversionFactor(factor);
        sparkBase.configure(sparkConfig.apply(encoderConfig), SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public void setVelocityConversionFactor(double factor) {
        encoderConfig.velocityConversionFactor(factor);
        sparkBase.configure(sparkConfig.apply(encoderConfig), SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public double getPosition() {
        return sparkBase.getEncoder().getPosition();
    }

    public double getVelocity() {
        return sparkBase.getEncoder().getVelocity();
    }

    public double getVoltage() {
        return sparkBase.getBusVoltage();
    }

    public double getCurrent() {
        return sparkBase.getOutputCurrent();
    }
}

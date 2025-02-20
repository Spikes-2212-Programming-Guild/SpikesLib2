package com.spikes2212.util.smartmotorcontrollers;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

/**
 * A {@link SmartMotorController} representation of a {@link SparkBase} motor controller.
 *
 * @author Camellia Lami
 * @see SmartMotorController
 */
public class SparkWrapper implements SmartMotorController {

    /**
     * The feed forward controller running the feed forward calculations.
     */
    private final FeedForwardController feedForwardController;

    /**
     * The spark motor controller.
     */
    protected SparkBase sparkBase;

    /**
     * The spark's configuration accessor.
     */
    protected SparkBaseConfigAccessor configAccessor;

    /**
     * The spark's configuration.
     */
    protected SparkBaseConfig sparkConfig;

    /**
     * The closed loop configuration for the spark.
     */
    protected ClosedLoopConfig closedLoopConfig;

    /**
     * The encoder configuration for the spark.
     */
    protected EncoderConfig encoderConfig;

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkMax} as the motor controller.
     *
     * @param deviceID    the spark's ID
     * @param type        the motor type
     * @param controlMode the type of feed forward control to use
     */
    public static SparkWrapper createSparkMax(int deviceID, SparkLowLevel.MotorType type,
                                              FeedForwardController.ControlMode controlMode) {
        SparkWrapper sparkWrapper = new SparkWrapper(controlMode);
        sparkWrapper.sparkBase = new SparkMax(deviceID, type);
        sparkWrapper.sparkConfig = new SparkMaxConfig();
        sparkWrapper.configAccessor = ((SparkMax) (sparkWrapper.sparkBase)).configAccessor;
        return sparkWrapper;
    }

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkMax} as the motor controller and with a
     * default {@link FeedForwardController.ControlMode} of linear velocity.
     *
     * @param deviceID the spark's ID
     * @param type     the motor type
     */
    public static SparkWrapper createSparkMax(int deviceID, SparkLowLevel.MotorType type) {
        return createSparkMax(deviceID, type, FeedForwardController.ControlMode.LINEAR_VELOCITY);
    }

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkFlex} as the motor controller.
     *
     * @param deviceID    the spark's ID
     * @param type        the motor type
     * @param controlMode the type of feed forward control to use
     */
    public static SparkWrapper createSparkFlex(int deviceID, SparkLowLevel.MotorType type,
                                               FeedForwardController.ControlMode controlMode) {
        SparkWrapper sparkWrapper = new SparkWrapper(controlMode);
        sparkWrapper.sparkBase = new SparkFlex(deviceID, type);
        sparkWrapper.sparkConfig = new SparkFlexConfig();
        sparkWrapper.configAccessor = ((SparkFlex) (sparkWrapper.sparkBase)).configAccessor;
        return sparkWrapper;
    }

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkFlex} as the motor controller and with a
     * default {@link FeedForwardController.ControlMode} of linear velocity.
     *
     * @param deviceID the spark's ID
     * @param type     the motor type
     */
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

    @Override
    public void setPosition(double position) {
        sparkBase.getEncoder().setPosition(position);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean updatePeriodically) {
        if (updatePeriodically) configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        double source;
        if (feedForwardController.getControlMode() == FeedForwardController.ControlMode.LINEAR_POSITION ||
                feedForwardController.getControlMode() == FeedForwardController.ControlMode.ANGULAR_POSITION) {
            source = sparkBase.getEncoder().getPosition();
        } else {
            source = sparkBase.getEncoder().getVelocity();
        }
        sparkBase.getClosedLoopController().setReference(setpoint, controlMode.getSparkControlType(),
                ClosedLoopSlot.kSlot0, feedForwardController.calculate(source, setpoint));
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        if (updatePeriodically) configureLoop(pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
        double source;
        if (feedForwardSettings.getControlMode() == FeedForwardController.ControlMode.LINEAR_POSITION ||
                feedForwardSettings.getControlMode() == FeedForwardController.ControlMode.ANGULAR_POSITION) {
            source = sparkBase.getEncoder().getPosition();
        } else {
            source = sparkBase.getEncoder().getVelocity();
        }
        sparkBase.getClosedLoopController().setReference(setpoint, controlMode.getSparkControlType(),
                ClosedLoopSlot.kSlot0, feedForwardController.calculate(source, setpoint, acceleration));
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        pidSet(controlMode, setpoint, 0, pidSettings, feedForwardSettings, updatePeriodically);
    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case PERCENT_OUTPUT -> sparkBase.getAppliedOutput();
            case VELOCITY -> getVelocity();
            case CURRENT -> getCurrent();
            case VOLTAGE -> getVoltage() * sparkBase.getAppliedOutput();
            default -> getPosition();
        };
        return Math.abs(value - setpoint) <= tolerance;
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

    @Override
    public double getPosition() {
        return sparkBase.getEncoder().getPosition();
    }

    @Override
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

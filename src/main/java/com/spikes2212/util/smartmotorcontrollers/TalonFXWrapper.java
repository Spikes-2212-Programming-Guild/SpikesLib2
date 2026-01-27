package com.spikes2212.util.smartmotorcontrollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

/**
 * A {@link SmartMotorController} representation of a {@link TalonFX} motor controller.
 *
 * @author Camellia Lami
 * @see SmartMotorController
 */
public class TalonFXWrapper implements SmartMotorController {

    /**
     * The TalonFX motor controller.
     */
    protected final TalonFX talonFX;

    /**
     * The closed loop configurations of the talon.
     */
    protected final Slot0Configs closedLoopConfig;

    /**
     * The talon configurations which affect motor output.
     */
    protected final MotorOutputConfigs motorOutputConfigs;

    /**
     * Constructs a new instance of {@link TalonFXWrapper}.
     *
     * @param deviceId the talon's id
     * @param canbus   the name of the canbus which the talon runs on
     */
    public TalonFXWrapper(int deviceId, String canbus) {
        talonFX = new TalonFX(deviceId, canbus);
        closedLoopConfig = new Slot0Configs();
        motorOutputConfigs = new MotorOutputConfigs();
    }

    /**
     * Constructs a new instance of {@link TalonFXWrapper}.
     *
     * @param deviceId the talon's id
     * @param canbus   the canbus which the talon runs on
     */
    public TalonFXWrapper(int deviceId, CANBus canbus) {
        this(deviceId, canbus.getName());
    }

    /**
     * Constructs a new instance of {@link TalonFXWrapper}.
     *
     * @param deviceId the talon's id
     */
    public TalonFXWrapper(int deviceId) {
        this(deviceId, "");
    }

    public void restoreFactoryDefaults() {
        talonFX.getConfigurator().apply(new TalonFXConfiguration());
        motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
        setIdleMode(NeutralModeValue.Coast);
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
        StaticFeedforwardSignValue kSType;
        GravityTypeValue kGType;
        switch (feedForwardSettings.getControlMode()) {
            case LINEAR_POSITION -> {
                kSType = StaticFeedforwardSignValue.UseClosedLoopSign;
                kGType = GravityTypeValue.Elevator_Static;
            }
            case ANGULAR_POSITION -> {
                kSType = StaticFeedforwardSignValue.UseClosedLoopSign;
                kGType = GravityTypeValue.Arm_Cosine;
            }
            case LINEAR_VELOCITY -> {
                kSType = StaticFeedforwardSignValue.UseVelocitySign;
                kGType = GravityTypeValue.Elevator_Static;
            }
            case ANGULAR_VELOCITY -> {
                kSType = StaticFeedforwardSignValue.UseVelocitySign;
                kGType = GravityTypeValue.Arm_Cosine;
            }
            default -> throw new IllegalArgumentException("Invalid feed forward type!");
        }
        closedLoopConfig.withStaticFeedforwardSign(kSType);
        closedLoopConfig.withGravityType(kGType);
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
    public void setPosition(double position) {
        talonFX.setPosition(position);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean updatePeriodically) {
        if (updatePeriodically) configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        ControlRequest request = switch (controlMode) {
            case CURRENT -> new TorqueCurrentFOC(setpoint);
            case PERCENT_OUTPUT -> new DutyCycleOut(setpoint);
            case TRAPEZOID_PROFILE -> new MotionMagicDutyCycle(setpoint);
            case VOLTAGE -> new VoltageOut(setpoint);
            case VELOCITY -> new VelocityDutyCycle(setpoint);
            case POSITION -> new PositionDutyCycle(setpoint);
        };
        talonFX.setControl(request);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        pidSet(controlMode, setpoint, pidSettings, feedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS, updatePeriodically);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        pidSet(controlMode, setpoint, 0, pidSettings, feedForwardSettings, updatePeriodically);
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
    public void setVoltage(double voltage) {
        talonFX.setVoltage(voltage);
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

    public void setEncoderConversionFactor(double factor) {
        FeedbackConfigs configs = new FeedbackConfigs();
        configs.withSensorToMechanismRatio(1 / factor);
        talonFX.getConfigurator().apply(configs);
    }

    @Override
    public double getPosition() {
        return talonFX.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return talonFX.getVelocity().getValueAsDouble();
    }

    public double getAcceleration() {
        return talonFX.getAcceleration().getValueAsDouble();
    }

    public double getCurrent() {
        return talonFX.getTorqueCurrent().getValueAsDouble();
    }

    public double getVoltage() {
        return talonFX.getMotorVoltage().getValueAsDouble();
    }

    public TalonFX getDevice() {
        return talonFX;
    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case VELOCITY -> getVelocity();
            case POSITION, TRAPEZOID_PROFILE -> getPosition();
            case CURRENT -> getCurrent();
            case PERCENT_OUTPUT -> talonFX.get();
            case VOLTAGE -> getVoltage();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }
}

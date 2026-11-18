package com.spikes2212.util.smartmotorcontrollers;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.Simulatable;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A simulated wrapper for a {@link SmartMotorController} using {@link DCMotorSim}.
 *
 * @author Adi Yekutiel
 * @see SmartMotorController
 * @see Simulatable
 */
public class SimulatedSmartMotorControllerWrapper implements SmartMotorController, Simulatable {

    private static final double MAX_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE = -12.0;
    private static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;
    private static final double DEFAULT_VALUE = 0.0;

    private final DCMotorSim motorSim;
    private final PIDController pidController;

    private PIDSettings pidSettings;
    private FeedForwardSettings feedForwardSettings;
    private FeedForwardController feedForwardController;
    private TrapezoidProfileSettings trapezoidProfileSettings;
    private TrapezoidProfile trapezoidProfile;
    private TrapezoidProfile.State trapezoidGoal;
    private TrapezoidProfile.State trapezoidSetpoint;
    private UnifiedControlMode currentControlMode;
    private double appliedVoltage;
    private double currentSetpoint;
    private double currentAcceleration;
    private boolean inverted;

    public SimulatedSmartMotorControllerWrapper(DCMotor motor, double gearRatio, double moi) {
        this.motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, moi, gearRatio),
                motor
        );
        this.pidSettings = new PIDSettings(DEFAULT_VALUE, DEFAULT_VALUE, DEFAULT_VALUE);
        this.pidController = new PIDController(
                pidSettings.getkP(),
                pidSettings.getkI(),
                pidSettings.getkD()
        );
        this.feedForwardSettings = FeedForwardSettings.EMPTY_FF_SETTINGS;
        this.feedForwardController = new FeedForwardController(feedForwardSettings);
        this.trapezoidProfileSettings = new TrapezoidProfileSettings(DEFAULT_VALUE, DEFAULT_VALUE);
        this.trapezoidProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        trapezoidProfileSettings.getMaxVelocity(),
                        trapezoidProfileSettings.getMaxAcceleration()
                )
        );
        this.trapezoidGoal = new TrapezoidProfile.State();
        this.trapezoidSetpoint = new TrapezoidProfile.State();
        this.currentControlMode = UnifiedControlMode.PERCENT_OUTPUT;
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        this.pidSettings = pidSettings;
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());
    }

    @Override
    public void configureFF(FeedForwardSettings feedForwardSettings) {
        this.feedForwardSettings = feedForwardSettings;
        this.feedForwardController = new FeedForwardController(feedForwardSettings);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        this.trapezoidProfileSettings = trapezoidProfileSettings;
        this.trapezoidProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        trapezoidProfileSettings.getMaxVelocity(),
                        trapezoidProfileSettings.getMaxAcceleration()
                )
        );
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean updatePeriodically) {
        if (updatePeriodically) configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        pidSet(controlMode, setpoint, DEFAULT_VALUE, pidSettings, feedForwardSettings, false);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        if (updatePeriodically) {
            configurePID(pidSettings);
            configureFF(feedForwardSettings);
        }
        if (this.currentControlMode != UnifiedControlMode.TRAPEZOID_PROFILE
                && controlMode == UnifiedControlMode.TRAPEZOID_PROFILE) {
            this.trapezoidSetpoint = new TrapezoidProfile.State(getPosition(), getVelocity());
        }
        this.currentControlMode = controlMode;
        this.currentSetpoint = setpoint;
        this.currentAcceleration = acceleration;
        if (controlMode == UnifiedControlMode.TRAPEZOID_PROFILE) {
            this.trapezoidGoal = new TrapezoidProfile.State(setpoint, DEFAULT_VALUE);
        }
        applyControlLoop(DEFAULT_DT_SECONDS);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        pidSet(controlMode, setpoint, DEFAULT_VALUE, pidSettings, feedForwardSettings, updatePeriodically);
    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double current = controlMode == UnifiedControlMode.VELOCITY ? getVelocity() : getPosition();
        return Math.abs(setpoint - current) <= tolerance;
    }

    @Override
    public void disable() {
        setVoltage(DEFAULT_VALUE);
    }

    @Override
    public void stopMotor() {
        disable();
    }

    @Override
    public void update(double dtSeconds) {
        applyControlLoop(dtSeconds);
        motorSim.update(dtSeconds);
    }

    @Override
    public double get() {
        return appliedVoltage / MAX_VOLTAGE;
    }

    @Override
    public void set(double speed) {
        double adjustedSpeed = inverted ? -speed : speed;
        setVoltage(adjustedSpeed * MAX_VOLTAGE);
    }

    @Override
    public void setVoltage(double outputVolts) {
        this.currentControlMode = UnifiedControlMode.PERCENT_OUTPUT;
        applyVoltage(outputVolts);
    }

    @Override
    public double getPosition() {
        return motorSim.getAngularPositionRotations();
    }

    @Override
    public void setPosition(double position) {
        motorSim.setState(position * RADIANS_PER_ROTATION, motorSim.getAngularVelocityRadPerSec());
        trapezoidSetpoint = new TrapezoidProfile.State(position, getVelocity());
    }

    @Override
    public double getVelocity() {
        return motorSim.getAngularVelocityRPM();
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    private void applyControlLoop(double dtSeconds) {
        if (currentControlMode == UnifiedControlMode.VELOCITY) {
            double feedbackOutput = pidController.calculate(getVelocity(), currentSetpoint);
            double feedForwardOutput = feedForwardController.calculate(
                    getPosition(),
                    currentSetpoint,
                    currentAcceleration
            );
            applyVoltage(feedbackOutput + feedForwardOutput);
        }
        if (currentControlMode == UnifiedControlMode.POSITION) {
            double feedbackOutput = pidController.calculate(getPosition(), currentSetpoint);
            double feedForwardOutput = feedForwardController.calculate(
                    currentSetpoint,
                    DEFAULT_VALUE,
                    currentAcceleration
            );
            applyVoltage(feedbackOutput + feedForwardOutput);
        }
        if (currentControlMode == UnifiedControlMode.TRAPEZOID_PROFILE && dtSeconds > DEFAULT_VALUE) {
            double previousVelocity = trapezoidSetpoint.velocity;
            trapezoidSetpoint = trapezoidProfile.calculate(dtSeconds, trapezoidSetpoint, trapezoidGoal);
            double profileAcceleration = (trapezoidSetpoint.velocity - previousVelocity) / dtSeconds;
            double feedbackOutput = pidController.calculate(getPosition(), trapezoidSetpoint.position);
            double feedForwardOutput = feedForwardController.calculate(
                    trapezoidSetpoint.position,
                    trapezoidSetpoint.velocity,
                    profileAcceleration
            );
            applyVoltage(feedbackOutput + feedForwardOutput);
        }
    }

    private void applyVoltage(double outputVolts) {
        this.appliedVoltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, outputVolts));
        motorSim.setInputVoltage(inverted ? -appliedVoltage : appliedVoltage);
    }
}

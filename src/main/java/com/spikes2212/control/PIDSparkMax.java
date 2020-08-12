package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import java.util.function.Supplier;

/**
 * A {@link CANSparkMax} that can run a PID loop.
 *
 * @author Eran Goldstein
 */
public class PIDSparkMax implements PIDSpeedController {

    /**
     * The {@link CANSparkMax} on which the PID is calculated.
     */
    private final CANSparkMax sparkMax;

    /**
     * The {@link CANPIDController} which is responsible for handling the PID loop.
     */
    private final CANPIDController pidController;

    /**
     * The {@link CANEncoder} of the motor connected to the spark max.
     */
    private final CANEncoder encoder;

    /**
     * The PID loop's {@link PIDSettings}.
     */
    private final PIDSettings settings;

    /**
     * The PID loop's feed forward constant.
     */
    private final Supplier<Double> kF;

    /**
     * The PID loop's {@link ControlType}.
     */
    private final ControlType mode;

    /**
     * Constructs a PIDSparkMax instance with the given parameters as field values.
     *
     * @param sparkMax The Spark Max speed controller on which the PID loop is calculated.
     * @param settings The PID loop's {@link PIDSettings}.
     * @param kF       The PID loop's feed forward constant.
     * @param mode     The PID loop's {@link ControlMode}.
     */
    public PIDSparkMax(CANSparkMax sparkMax, PIDSettings settings, Supplier<Double> kF, ControlType mode, int timeout) {
        this.sparkMax = sparkMax;
        this.pidController = sparkMax.getPIDController();
        this.encoder = sparkMax.getEncoder();
        this.settings = settings;
        this.kF = kF;
        this.mode = mode;
        sparkMax.setCANTimeout(timeout);
    }

    @Override
    public void configureLoop(Supplier<Double> maxSpeed, Supplier<Double> minSpeed) {
        sparkMax.restoreFactoryDefaults();

        pidController.setOutputRange(maxSpeed.get(), minSpeed.get());

        pidController.setP(settings.getkP());
        pidController.setI(settings.getkI());
        pidController.setD(settings.getkD());
        pidController.setFF(kF.get());
    }

    @Override
    public void pidSet(double setpoint) {
        pidController.setP(settings.getkP());
        pidController.setI(settings.getkI());
        pidController.setD(settings.getkD());
        pidController.setFF(kF.get());

        pidController.setReference(setpoint, mode);
    }

    @Override
    public boolean onTarget(double setpoint) {
        return Math.abs(setpoint - encoder.getPosition()) <= settings.getTolerance();
    }

    @Override
    public double getWaitTime() {
        return settings.getWaitTime();
    }

    @Override
    public void finish() {
        sparkMax.stopMotor();
    }

    @Override
    public void set(double speed) {
        sparkMax.set(speed);
    }

    @Override
    public double get() {
        return sparkMax.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        sparkMax.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return sparkMax.getInverted();
    }

    @Override
    public void disable() {
        sparkMax.disable();
    }

    @Override
    public void stopMotor() {
        sparkMax.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        sparkMax.pidWrite(output);
    }
}

package com.spikes2212.util.smartmotorcontrollers;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulatedSmartMotorControllerWrapper implements SmartMotorController {

    private static final double MAX_VOLTAGE = 12.0;

    private static final double MIN_VOLTAGE = -12.0;

    private static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;

    private final DCMotorSim motorSim;

    private final PIDController pidController;

    private double appliedVoltage = 0.0;

    private double currentSetpoint = 0.0;

    private boolean inverted = false;

    private UnifiedControlMode currentControlMode = UnifiedControlMode.PERCENT_OUTPUT;

    public SimulatedSmartMotorControllerWrapper(DCMotor motor, double gearRatio, double moi) {
        this.motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, moi, gearRatio),
                motor
        );
        this.pidController = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        pidController.setP(pidSettings.getkP());
        pidController.setI(pidSettings.getkI());
        pidController.setD(pidSettings.getkD());
    }

    @Override
    public void configureFF(FeedForwardSettings feedForwardSettings) {
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    @Override
    public void setPosition(double position) {
        motorSim.setState(position * RADIANS_PER_ROTATION, motorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean updatePeriodically) {
        configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        pidSet(controlMode, setpoint, pidSettings, feedForwardSettings, updatePeriodically);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        if (updatePeriodically) {
            configurePID(pidSettings);
            configureFF(feedForwardSettings);
        }
        this.currentControlMode = controlMode;
        this.currentSetpoint = setpoint;

        if (controlMode == UnifiedControlMode.VELOCITY) {
            double feedbackOutput = pidController.calculate(getVelocity(), setpoint);
            setVoltage(feedbackOutput);
        } else if (controlMode == UnifiedControlMode.POSITION) {
            double feedbackOutput = pidController.calculate(getPosition(), setpoint);
            setVoltage(feedbackOutput);
        }
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, boolean updatePeriodically) {
        pidSet(controlMode, setpoint, 0.0, pidSettings, feedForwardSettings, updatePeriodically);
    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double current = controlMode == UnifiedControlMode.VELOCITY ? getVelocity() : getPosition();
        return Math.abs(setpoint - current) <= tolerance;
    }

    @Override
    public double getPosition() {
        return motorSim.getAngularPositionRotations();
    }

    @Override
    public double getVelocity() {
        return motorSim.getAngularVelocityRPM();
    }

    @Override
    public void set(double speed) {
        double adjustedSpeed = inverted ? -speed : speed;
        setVoltage(adjustedSpeed * MAX_VOLTAGE);
    }

    @Override
    public double get() {
        return appliedVoltage / MAX_VOLTAGE;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void disable() {
        setVoltage(0.0);
    }

    @Override
    public void stopMotor() {
        disable();
    }

    @Override
    public void setVoltage(double outputVolts) {
        this.appliedVoltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, outputVolts));
        motorSim.setInputVoltage(inverted ? -appliedVoltage : appliedVoltage);
    }

    public void update(double dtSeconds) {
        if (currentControlMode == UnifiedControlMode.VELOCITY) {
            double feedbackOutput = pidController.calculate(getVelocity(), currentSetpoint);
            setVoltage(feedbackOutput);
        } else if (currentControlMode == UnifiedControlMode.POSITION) {
            double feedbackOutput = pidController.calculate(getPosition(), currentSetpoint);
            setVoltage(feedbackOutput);
        }
        motorSim.update(dtSeconds);
    }
}

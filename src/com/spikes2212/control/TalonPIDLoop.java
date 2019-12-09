package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import java.util.function.Supplier;

/**
 * A PIDLoop using a CTRE speed controller (TalonSRX or VictorSPX).
 *
 * @author Eran Goldstein
 */
public class TalonPIDLoop implements PIDLoop {
    /**
     * The speed controller this loop runs on.
     */
    private BaseMotorController motor;

    /**
     * The proportional component of the loop.
     */
    private double kp;

    /**
     * The integral component of the loop.
     */
    private double ki;

    /**
     * The derivative component of the loop.
     */
    private double kd;

    /**
     * The time required to stay on target.
     */
    private double waitTime;

    /**
     * The acceptable distance from the target.
     */
    private double tolerance;

    /**
     * The setpoint the loop should go towards.
     */
    private Supplier<Double> setpoint;

    /**
     * Which loop to run on.
     * <p>
     * 0 - primary loop
     * 1 - secondary loop
     */
    private int loop;

    private int timeout;

    public TalonPIDLoop(BaseMotorController motor, double kp, double ki, double kd,
                        double waitTime, double tolerance, Supplier<Double> setpoint) {
        this(motor, kp, ki, kd, waitTime, tolerance, setpoint, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, double kp, double ki, double kd, double waitTime,
                        double tolerance, Supplier<Double> setpoint, int loop) {
        this(motor, kp, ki, kd, waitTime, tolerance, setpoint, loop, 30);
    }

    public TalonPIDLoop(BaseMotorController motor, double kp, double ki, double kd, double waitTime,
                        double tolerance, Supplier<Double> setpoint, int loop, int timeout) {
        this.motor = motor;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.waitTime = waitTime;
        this.tolerance = tolerance;
        this.setpoint = setpoint;
        this.loop = loop;
        this.timeout = timeout;
    }

    /**
     * Initialize the PID settings on the motor.
     */
    private void initialize() {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, loop, timeout);

        motor.configNominalOutputForward(0, timeout);
        motor.configNominalOutputReverse(0, timeout);
        motor.configPeakOutputForward(1.0, timeout);
        motor.configPeakOutputReverse(-1.0, timeout);

        motor.configAllowableClosedloopError(loop, 0, timeout);

        motor.config_kP(loop, kp, timeout);
        motor.config_kI(loop, ki, timeout);
        motor.config_kD(loop, kd, timeout);

        motor.setSelectedSensorPosition(0, loop, timeout);
    }

    @Override
    public void enable() {
        initialize();
    }

    @Override
    public void disable() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void update() {
        motor.set(ControlMode.Position, setpoint.get());
    }

    @Override
    public boolean onTarget() {
        return Math.abs(setpoint.get() - motor.getSelectedSensorPosition(loop)) < tolerance;
    }
}

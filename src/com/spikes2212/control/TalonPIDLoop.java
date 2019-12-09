package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Notifier;

public class TalonPIDLoop implements PIDLoop {
    private BaseMotorController motor;

    private double kp;
    private double ki;
    private double kd;

    private double waitTime;
    private double tolerance;
    private double setpoint;

    private int loop;
    private int timeout;

    private Notifier notifier;

    public TalonPIDLoop(BaseMotorController motor, double kp, double ki, double kd,
                        double waitTime, double tolerance, double setpoint) {
        this(motor, kp, ki, kd, waitTime, tolerance, setpoint, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, double kp, double ki, double kd, double waitTime,
                        double tolerance, double setpoint, int loop) {
        this(motor, kp, ki, kd, waitTime, tolerance, setpoint, loop, 30);
    }

    public TalonPIDLoop(BaseMotorController motor, double kp, double ki, double kd, double waitTime,
                        double tolerance, double setpoint, int loop, int timeout) {
        this.motor = motor;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.waitTime = waitTime;
        this.tolerance = tolerance;
        this.setpoint = setpoint;
        this.loop = loop;
        this.timeout = timeout;

        this.notifier = new Notifier(() -> motor.set(ControlMode.Position, setpoint));
    }

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
        notifier.startPeriodic(0.01);
    }

    @Override
    public void disable() {
        notifier.stop();
        notifier.close();

        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public boolean onTarget() {
        return Math.abs(setpoint - motor.getSelectedSensorPosition(loop)) < tolerance;
    }
}

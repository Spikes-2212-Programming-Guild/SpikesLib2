package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Function;
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
    private Supplier<Double> kP;

    /**
     * The integral component of the loop.
     */
    private Supplier<Double> kI;

    /**
     * The derivative component of the loop.
     */
    private Supplier<Double> kD;

    /**
     * The setpoint the loop should go towards.
     */
    private Supplier<Double> setpoint;

    /**
     * The acceptable distance from the target.
     */
    private Supplier<Double> tolerance;

    /**
     * The `canMove` method of the Subsystem this PIDLoop belongs to.
     */
    private Function<Double, Boolean> canMove;

    /**
     * The maximum output of the loop in any direction.
     */
    private Supplier<Double> peakOutput;

    /**
     * The time required to stay on target.
     */
    private Supplier<Double> waitTime;

    /**
     * The control mode to use (position, velocity or current) for PID.
     */
    private ControlMode controlMode;

    /**
     * Which loop to run on.
     * <p>
     * 0 - primary loop
     * 1 - secondary loop
     */
    private int loop;

    private int timeout;

    private double lastTimeNotOnTarget;

    public TalonPIDLoop(BaseMotorController motor, Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD,
                        Supplier<Double> setpoint, Supplier<Double> tolerance, Function<Double, Boolean> canMove) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, () -> 1.0);
    }

    public TalonPIDLoop(BaseMotorController motor, Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD,
                        Supplier<Double> setpoint, Supplier<Double> tolerance, Function<Double, Boolean> canMove,
                        Supplier<Double> peakOutput) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, () -> 0.0);
    }

    public TalonPIDLoop(BaseMotorController motor, Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD,
                        Supplier<Double> setpoint, Supplier<Double> tolerance, Function<Double, Boolean> canMove,
                        Supplier<Double> peakOutput, Supplier<Double> waitTime) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, waitTime, ControlMode.Position);
    }

    public TalonPIDLoop(BaseMotorController motor, Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD,
                        Supplier<Double> setpoint, Supplier<Double> tolerance, Function<Double, Boolean> canMove,
                        Supplier<Double> peakOutput, Supplier<Double> waitTime, ControlMode controlMode) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, waitTime, controlMode, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD,
                        Supplier<Double> setpoint, Supplier<Double> tolerance, Function<Double, Boolean> canMove,
                        Supplier<Double> peakOutput, Supplier<Double> waitTime, ControlMode controlMode, int loop) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, waitTime, controlMode, loop, 30);
    }

    public TalonPIDLoop(BaseMotorController motor, Supplier<Double> kP, Supplier<Double> kI, Supplier<Double> kD,
                        Supplier<Double> setpoint, Supplier<Double> tolerance, Function<Double, Boolean> canMove,
                        Supplier<Double> peakOutput, Supplier<Double> waitTime, ControlMode controlMode, int loop,
                        int timeout) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = setpoint;
        this.tolerance = tolerance;
        this.canMove = canMove;
        this.peakOutput = peakOutput;
        this.waitTime = waitTime;
        this.controlMode = controlMode;
        this.loop = loop;
        this.timeout = timeout;
        this.lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    public TalonPIDLoop(BaseMotorController motor, double kP, double kI, double kD, double setpoint, double tolerance,
                        Function<Double, Boolean> canMove) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, 1.0);
    }

    public TalonPIDLoop(BaseMotorController motor, double kP, double kI, double kD, double setpoint, double tolerance,
                        Function<Double, Boolean> canMove, double peakOutput) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, double kP, double kI, double kD, double setpoint, double tolerance,
                        Function<Double, Boolean> canMove, double peakOutput, double waitTime) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, waitTime, ControlMode.Position);
    }

    public TalonPIDLoop(BaseMotorController motor, double kP, double kI, double kD, double setpoint, double tolerance,
                        Function<Double, Boolean> canMove, double peakOutput, double waitTime,
                        ControlMode controlMode) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, waitTime, controlMode, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, double kP, double kI, double kD, double setpoint, double tolerance,
                        Function<Double, Boolean> canMove, double peakOutput, double waitTime, ControlMode controlMode,
                        int loop) {
        this(motor, kP, kI, kD, setpoint, tolerance, canMove, peakOutput, waitTime, controlMode, loop, 30);
    }

    public TalonPIDLoop(BaseMotorController motor, double kP, double kI, double kD, double setpoint, double tolerance,
                        Function<Double, Boolean> canMove, double peakOutput, double waitTime, ControlMode controlMode,
                        int loop, int timeout) {
        this(motor, () -> kP, () -> kI, () -> kD, () -> setpoint, () -> tolerance, canMove, () -> peakOutput,
                () -> waitTime, controlMode, loop, timeout);
    }

    private void initialize() {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, loop, timeout);

        motor.configNominalOutputForward(0, timeout);
        motor.configNominalOutputReverse(0, timeout);
        motor.configPeakOutputForward(peakOutput.get(), timeout);
        motor.configPeakOutputReverse(-peakOutput.get(), timeout);

        motor.configAllowableClosedloopError(loop, 0, timeout);

        motor.config_kP(loop, kP.get(), timeout);
        motor.config_kI(loop, kI.get(), timeout);
        motor.config_kD(loop, kD.get(), timeout);
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
        motor.configPeakOutputForward(peakOutput.get(), timeout);
        motor.configPeakOutputReverse(-peakOutput.get(), timeout);

        motor.config_kP(loop, kP.get(), timeout);
        motor.config_kI(loop, kI.get(), timeout);
        motor.config_kD(loop, kD.get(), timeout);

        motor.set(controlMode, setpoint.get());
    }

    @Override
    public boolean onTarget() {
        if(!inPosition()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get()
                || !canMove.apply(motor.getMotorOutputPercent());
    }

    private boolean inPosition() throws IllegalArgumentException {
        double currentlyOn;
        switch(controlMode) {
            case Position:
                currentlyOn = motor.getSelectedSensorPosition(loop);
                break;
            case Velocity:
                currentlyOn = motor.getSelectedSensorVelocity(loop);
                break;
            default:
                throw new IllegalArgumentException(controlMode.toString() + " is illegal in SpikesLib TalonPIDLoop");
        }

        return Math.abs(setpoint.get() - currentlyOn) < tolerance.get();
    }
}

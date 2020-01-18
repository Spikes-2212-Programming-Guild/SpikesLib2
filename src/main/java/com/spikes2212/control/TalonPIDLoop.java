package com.spikes2212.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Predicate;
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
     * The {@link PIDSettings} this loop will use.
     */
    private PIDSettings PIDSettings;

    /**
     * The setpoint the loop should go towards.
     */
    private double setpoint;

    /**
     * The `canMove` method of the Subsystem this PIDLoop belongs to.
     */
    private Predicate<Double> canMove;

    /**
     * The maximum output of the loop in any direction.
     */
    private Supplier<Double> peakOutput;

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

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove) {
        this(motor, PIDSettings, canMove, () -> 1.0);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, Supplier<Double> peakOutput) {
        this(motor, PIDSettings, canMove, peakOutput, ControlMode.Position);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, Supplier<Double> peakOutput, ControlMode controlMode) {
        this(motor, PIDSettings, canMove, peakOutput, controlMode, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, Supplier<Double> peakOutput, ControlMode controlMode, int loop) {
        this(motor, PIDSettings, canMove, peakOutput, controlMode, loop, 30);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings, Predicate<Double> canMove,
                        Supplier<Double> peakOutput, ControlMode controlMode, int loop, int timeout) {
        this.motor = motor;
        this.PIDSettings = PIDSettings;
        this.canMove = canMove;
        this.peakOutput = peakOutput;
        this.controlMode = controlMode;
        this.loop = loop;
        this.timeout = timeout;
        this.lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, double peakOutput) {
        this(motor, PIDSettings, canMove, peakOutput, ControlMode.Position);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, double peakOutput,
                        ControlMode controlMode) {
        this(motor, PIDSettings, canMove, peakOutput, controlMode, 0);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, double peakOutput, ControlMode controlMode,
                        int loop) {
        this(motor, PIDSettings, canMove, peakOutput, controlMode, loop, 30);
    }

    public TalonPIDLoop(BaseMotorController motor, PIDSettings PIDSettings,
                        Predicate<Double> canMove, double peakOutput, ControlMode controlMode,
                        int loop, int timeout) {
        this(motor, PIDSettings, canMove, () -> peakOutput, controlMode, loop, timeout);
    }

    private void initialize() {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, loop, timeout);

        motor.configNominalOutputForward(0, timeout);
        motor.configNominalOutputReverse(0, timeout);
        motor.configPeakOutputForward(peakOutput.get(), timeout);
        motor.configPeakOutputReverse(-peakOutput.get(), timeout);

        motor.configAllowableClosedloopError(loop, 0, timeout);

        motor.config_kP(loop, PIDSettings.getkP(), timeout);
        motor.config_kI(loop, PIDSettings.getkI(), timeout);
        motor.config_kD(loop, PIDSettings.getkD(), timeout);
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

        motor.config_kP(loop, PIDSettings.getkP(), timeout);
        motor.config_kI(loop, PIDSettings.getkI(), timeout);
        motor.config_kD(loop, PIDSettings.getkD(), timeout);

        motor.set(controlMode, setpoint);
    }

    @Override
    public boolean onTarget() {
        if(!inPosition()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= PIDSettings.getWaitTime()
                || !canMove.test(motor.getMotorOutputPercent());
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Test if the loop is currently within `tolerance` of `setpoint`.
     *
     * @return true when the loop is currently within `tolerance` of `setpoint`, `false` otherwise
     * @throws IllegalArgumentException when the `ControlMode` is unsupported
     */
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

        return Math.abs(setpoint - currentlyOn) < PIDSettings.getTolerance();
    }
}

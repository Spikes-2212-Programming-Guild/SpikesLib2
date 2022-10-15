package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;

/**
 * A {@link Subsystem} which consists of a master CTRE motor controller that can run PID loops and additional
 * CTRE motor controllers that follow it.
 *
 * @param <T> the type of the motor controller on which the loop is run
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerSubsystem
 * @see BaseMotorController
 * @see MotorController
 */
public class CTRESmartMotorControllerGenericSubsystem<T extends BaseMotorController & MotorController>
        extends DashboardedSubsystem implements SmartMotorControllerSubsystem {

    /**
     * The slot on the motor controller on which the loop is run.
     */
    private static final int SLOT = 0;

    /**
     * The motor controller which runs the loops.
     */
    protected final T master;

    /**
     * Additional motor controllers that follow the master.
     */
    protected final List<? extends IFollower> slaves;

    /**
     * Constructs a new instance of {@link CTRESmartMotorControllerGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the master
     */
    public CTRESmartMotorControllerGenericSubsystem(String namespaceName, T master,
                                                    IFollower... slaves) {
        super(namespaceName);
        this.master = master;
        this.slaves = List.of(slaves);
        this.slaves.forEach(s -> s.follow(master));
    }

    /**
     * Adds any data or commands to the {@link NetworkTable}s, which can be accessed using the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {
    }

    /**
     * Configures the loop's PID constants and feed forward gains.
     */
    @Override
    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        master.config_kP(SLOT, pidSettings.getkP());
        master.config_kI(SLOT, pidSettings.getkI());
        master.config_kD(SLOT, pidSettings.getkD());
        master.config_kF(SLOT, feedForwardSettings.getkV());
    }

    /**
     * Configures the loop's trapezoid profile settings.
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        master.configMotionAcceleration(settings.getAccelerationRate());
        master.configMotionCruiseVelocity(settings.getMaxVelocity());
        master.configMotionSCurveStrength(settings.getCurve());
    }

    /**
     * Configures the loop's settings.
     */
    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        master.configFactoryDefault();
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode              the loop's control mode (e.g. voltage, velocity, position...)
     * @param setpoint                 the loop's target setpoint
     * @param pidSettings              the PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void pidSet(ControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.set(controlMode, setpoint);
    }

    /**
     * Stops any control loops running on the motor controller.
     */
    @Override
    public void finish() {
        master.stopMotor();
    }

    /**
     * Checks whether the loop is currently on the target setpoint. <br>
     * This method, as is, <b>does not</b> cover every case and should be overridden if necessary.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param controlType the loop's control type (e.g. voltage, velocity, position...). Only applicable
     *                    when running the loop on a Spark Max motor controller, and is therefore unused
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean onTarget(ControlMode controlMode, CANSparkMax.ControlType controlType, double tolerance,
                            double setpoint) {
        double value;
        switch (controlMode) {
            case Velocity:
                value = master.getSelectedSensorVelocity();
                break;
            case PercentOutput:
                value = master.getMotorOutputPercent();
                break;
            default:
                value = master.getSelectedSensorPosition();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }
}

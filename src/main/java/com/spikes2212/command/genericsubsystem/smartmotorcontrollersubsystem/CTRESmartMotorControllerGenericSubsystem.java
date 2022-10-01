package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.List;

/**
 * A subsystem which consists of a CTRE motor controller that can run PID loops and additional
 * CTRE motors that follow it.
 * @param <T> the type of the motor controller on which the loop is run
 *
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerSubsystem
 * @see BaseMotorController
 * @see MotorController
 */
public class CTRESmartMotorControllerGenericSubsystem<T extends BaseMotorController & MotorController>
        extends DashboardedSubsystem implements SmartMotorControllerSubsystem {

    /**
     * the slot on the motor controller on which the loop is run
     */
    private static final int SLOT = 0;

    /**
     * the motor controller which runs the loops
     */
    protected final T master;

    /**
     * additional motor controllers that follow the master
     */
    protected final List<? extends IFollower> slaves;

    /**
     * constructs a new instance of {@link CTRESmartMotorControllerGenericSubsystem}
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master the motor controller which runs the loops
     * @param slaves additional motor controllers that follow the master
     */
    public CTRESmartMotorControllerGenericSubsystem(String namespaceName, T master,
                                                    IFollower... slaves) {
        super(namespaceName);
        this.master = master;
        this.slaves = List.of(slaves);
        this.slaves.forEach(s -> s.follow(master));
    }

    /**
     * adds any data or commands to the {@link Shuffleboard}
     */
    @Override
    public void configureDashboard() {

    }

    /**
     * configures the loop's PID and feed forward constants
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    @Override
    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        master.config_kP(SLOT, pidSettings.getkP());
        master.config_kI(SLOT, pidSettings.getkI());
        master.config_kD(SLOT, pidSettings.getkD());
        master.config_kF(SLOT, feedForwardSettings.getkV());
    }

    /**
     * configures the loop's trapezoid profiling
     * @param settings the trapezoid profile configurations
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        master.configMotionAcceleration(settings.getAccelerationRate());
        master.configMotionCruiseVelocity(settings.getMaxVelocity());
        master.configMotionSCurveStrength(settings.getCurve());
    }

    /**
     * configures the loop's settings
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
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
     * @param controlMode the loop's control mode (e.g. voltage, velocity, position...)
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
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
     * Stop running control loops on the motor controller.
     */
    @Override
    public void finish() {
        master.stopMotor();
    }
}

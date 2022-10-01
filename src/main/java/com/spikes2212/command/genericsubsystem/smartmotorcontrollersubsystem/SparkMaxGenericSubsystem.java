package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.List;

/**
 * A subsystem which consists of a Spark Max motor controller that runs PID loops and additional
 * Spark Max controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerSubsystem
 * @see CANSparkMax
 */
public class SparkMaxGenericSubsystem extends DashboardedSubsystem implements SmartMotorControllerSubsystem {

    /**
     * The slot on the {@link CANSparkMax} on which the trapezoid profiling configurations are saved.
     */
    private static final int TRAPEZOID_SLOT_ID = 0;

    /**
     * The {@link CANSparkMax} which runs the loops.
     */
    protected final CANSparkMax master;

    /**
     * Additional {@link CANSparkMax}s that follow the master.
     */
    protected final List<CANSparkMax> slaves;

    /**
     * Constructs a new instance of {@link CTRESmartMotorControllerGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master the motor controller which runs the loops
     * @param slaves additional motor controllers that follow the master
     */
    public SparkMaxGenericSubsystem(String namespaceName, CANSparkMax master, CANSparkMax... slaves) {
        super(namespaceName);
        this.master = master;
        this.slaves = List.of(slaves);
        this.slaves.forEach(s -> s.follow(master));
    }

    /**
     * Adds any data or commands to the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {

    }

    /**
     * Configures the loop's PID and feed forward constants.
     *
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    @Override
    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        master.getPIDController().setFF(feedForwardSettings.getkV());
        master.getPIDController().setP(pidSettings.getkP());
        master.getPIDController().setI(pidSettings.getkI());
        master.getPIDController().setD(pidSettings.getkD());
    }

    /**
     * Configures the loop's trapezoid profiling.
     *
     * @param settings the trapezoid profile configurations
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        master.getPIDController().setSmartMotionMaxAccel(settings.getAccelerationRate(), TRAPEZOID_SLOT_ID);
        master.getPIDController().setSmartMotionMaxVelocity(settings.getMaxVelocity(), TRAPEZOID_SLOT_ID);
        master.getPIDController().setSmartMotionAccelStrategy(
                SparkMaxPIDController.AccelStrategy.fromInt(settings.getCurve()), TRAPEZOID_SLOT_ID);
    }

    /**
     * Configures the loop's settings.
     *
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        master.restoreFactoryDefaults();
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlType the loop's control type (e.g. voltage, velocity, position...)
     * @param setpoint the loop's target setpoint
     * @param pidSettings the PID constants
     * @param feedForwardSettings the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void pidSet(CANSparkMax.ControlType controlType, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlType);
    }

    @Override
    public void finish() {
        master.stopMotor();
    }
}

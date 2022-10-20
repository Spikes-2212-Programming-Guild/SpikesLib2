package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;

/**
 * A {@link Subsystem} which consists of a master {@link CANSparkMax} motor controller that runs control
 * loops and additional {@link CANSparkMax} motor controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerSubsystem
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
     * Constructs a new instance of {@link SparkMaxGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the master
     */
    public SparkMaxGenericSubsystem(String namespaceName, CANSparkMax master, CANSparkMax... slaves) {
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
        master.getPIDController().setFF(feedForwardSettings.getkV());
        master.getPIDController().setP(pidSettings.getkP());
        master.getPIDController().setI(pidSettings.getkI());
        master.getPIDController().setD(pidSettings.getkD());
    }

    /**
     * Configures the loop's trapezoid profile settings.
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
     * @param controlMode              the loop's control type (e.g. voltage, velocity, position...)
     * @param setpoint                 the loop's target setpoint
     * @param pidSettings              the PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType());
    }

    /**
     * Stops any control loops running on the motor controller.
     */
    @Override
    public void finish() {
        master.stopMotor();
    }

    /**
     * Checks whether the loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value;
        switch (controlMode) {
            case PERCENT_OUTPUT:
                value = master.getAppliedOutput();
                break;
            case VELOCITY:
                value = master.getEncoder().getVelocity();
                break;
            case CURRENT:
                value = master.getOutputCurrent();
                break;
            case VOLTAGE:
                value = master.getBusVoltage();
                break;
            default:
                value = master.getEncoder().getPosition();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }
}

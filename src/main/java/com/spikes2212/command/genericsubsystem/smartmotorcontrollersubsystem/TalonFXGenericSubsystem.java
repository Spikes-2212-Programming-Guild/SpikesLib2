package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;

/**
 * A {@link Subsystem} which consists of a master {@link TalonFX} motor controller that runs control
 * loops and additional {@link TalonFX} motor controllers that follow it.
 *
 * @author
 * @see DashboardedSubsystem
 * @see SmartMotorControllerGenericSubsystem
 */
public class TalonFXGenericSubsystem extends DashboardedSubsystem implements SmartMotorControllerGenericSubsystem {

    /**
     * The slot on the motor controller on which the loop is run.
     */
    private static final int PID_SLOT = 0;

    /**
     * The TalonFX which runs the loops.
     */
    protected final TalonFX master;

    /**
     * Additional TalonFXs that follow the master.
     */
    protected final List<? extends TalonFX> slaves;

    /**
     * Constructs a new instance of {@link TalonFXGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the TalonFX which runs the loops
     * @param slaves        additional TalonFXs that follow the master
     */
    public TalonFXGenericSubsystem(String namespaceName, TalonFX master, TalonFX... slaves) {
        super(namespaceName);
        this.master = master;
        this.slaves = List.of(slaves);
    }
    /**
     * Adds any data or commands to the {@link NetworkTable}s, which can be accessed using the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {
    }

    /**
     * Configures the loop's PID constants and feed forward gains.
     * Updates any loops running on the TalonFX.
     *
     * @param pidSettings         the PID constants
     * @param feedForwardSettings the feed forward gains
     */
    public void pidSet(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = pidSettings.getkP();
        config.Slot0.kI = pidSettings.getkI();
        config.Slot0.kD = pidSettings.getkD();
        config.Slot0.kS = feedForwardSettings.getkS();
        config.Slot0.kV = feedForwardSettings.getkV();
        config.Slot0.kA = feedForwardSettings.getkA();
        config.Slot0.kG = feedForwardSettings.getkG();
        master.getConfigurator().apply(config);
        slaves.get(0).setControl(new Follower(master.getDeviceID(), false));
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
    }

    /**
     * Stops any control loops running on the TalonFX.
     */
    @Override
    public void finish() {
        ((TalonFX) master).stopMotor();
    }

    /**
     * Checks whether the loop is currently on the target setpoint. <br>
     * This method, as is, <b>does not</b> cover every case and should be overridden if necessary.
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
            case VELOCITY:
                value = master.getVelocity().getValue();
                break;
            case PERCENT_OUTPUT:
                value = master.get() * 100;
                break;
            default:
                value = master.getPosition().getValue();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }
}

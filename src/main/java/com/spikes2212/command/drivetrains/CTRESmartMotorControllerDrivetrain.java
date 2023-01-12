package com.spikes2212.command.drivetrains;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
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
 * A {@link Subsystem} which consists of a master CTRE motor controller that can run control loops and additional
 * CTRE motor controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerTankDrivetrain
 * @see BaseMotorController
 * @see MotorController
 */
public class CTRESmartMotorControllerDrivetrain extends TankDrivetrain
        implements SmartMotorControllerTankDrivetrain {

    /**
     * The slot on the motor controller on which the loop is run.
     */
    private static final int LOOP_SLOT = 0;

    /**
     * The motor controller which runs the left side's loops.
     */
    protected final BaseMotorController leftMaster;

    /**
     * The motor controller which runs the right side's loops.
     */
    protected final BaseMotorController rightMaster;

    /**
     * Additional motor controllers that follow the left master.
     */
    protected final List<? extends IFollower> leftSlaves;

    /**
     * Additional motor controllers that follow the right master.
     */
    protected final List<? extends IFollower> rightSlaves;

    /**
     * Constructs a new instance of {@link CTRESmartMotorControllerDrivetrain}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param leftMaster    the motor controller which runs the left side's loops
     * @param rightMaster   the motor controller which runs the right side's loops
     * @param leftSlaves    additional motor controllers that follow the left master
     * @param rightSlaves   additional motor controllers that follow the right master
     */
    public CTRESmartMotorControllerDrivetrain(String namespaceName, BaseMotorController leftMaster,
                                              BaseMotorController rightMaster, List<IFollower> leftSlaves,
                                              List<IFollower> rightSlaves) {
        super(namespaceName, (MotorController) leftMaster, (MotorController) rightMaster);
        this.leftMaster = leftMaster;
        this.leftSlaves = leftSlaves;
        this.leftSlaves.forEach(s -> s.follow(leftMaster));
        this.rightMaster = rightMaster;
        this.rightSlaves = leftSlaves;
        this.rightSlaves.forEach(s -> s.follow(rightMaster));
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
    public void configPIDF(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                           FeedForwardSettings feedForwardSettings) {
        leftMaster.config_kP(LOOP_SLOT, leftPIDSettings.getkP());
        leftMaster.config_kI(LOOP_SLOT, leftPIDSettings.getkI());
        leftMaster.config_kD(LOOP_SLOT, leftPIDSettings.getkD());
        leftMaster.config_kF(LOOP_SLOT, feedForwardSettings.getkV());
        rightMaster.config_kP(LOOP_SLOT, rightPIDSettings.getkP());
        rightMaster.config_kI(LOOP_SLOT, rightPIDSettings.getkI());
        rightMaster.config_kD(LOOP_SLOT, rightPIDSettings.getkD());
        rightMaster.config_kF(LOOP_SLOT, feedForwardSettings.getkV());
    }

    /**
     * Configures the loop's trapezoid profile settings.
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        leftMaster.configMotionAcceleration(settings.getAccelerationRate());
        leftMaster.configMotionCruiseVelocity(settings.getMaxVelocity());
        leftMaster.configMotionSCurveStrength(settings.getCurve());
        rightMaster.configMotionAcceleration(settings.getAccelerationRate());
        rightMaster.configMotionCruiseVelocity(settings.getMaxVelocity());
        rightMaster.configMotionSCurveStrength(settings.getCurve());
    }

    /**
     * Configures the loop's settings.
     */
    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on each side's motor controllers.
     *
     * @param controlMode              the loop's control mode (e.g. voltage, velocity, position...)
     * @param leftSetpoint             the left side's loop's target setpoint
     * @param rightSetpoint            the right side's loop's target setpoint
     * @param leftPIDSettings          the left side's PID constants
     * @param rightPIDSettings         the right side's PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        leftMaster.set(controlMode.getCTREControlMode(), leftSetpoint);
        rightMaster.set(controlMode.getCTREControlMode(), rightSetpoint);
    }

    /**
     * Stops any control loops running on each side's motor controllers.
     */
    @Override
    public void finish() {
        ((MotorController)leftMaster).stopMotor();
        ((MotorController)rightMaster).stopMotor();
    }

    /**
     * Checks whether the left side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean leftOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value;
        switch (controlMode) {
            case PERCENT_OUTPUT:
                value = leftMaster.getMotorOutputPercent();
                break;
            case VELOCITY:
                value = leftMaster.getSelectedSensorVelocity();
                break;
            case CURRENT:
                if (leftMaster instanceof BaseTalon) {
                    value = ((BaseTalon) leftMaster).getStatorCurrent();
                    break;
                }
            case VOLTAGE:
                value = leftMaster.getBusVoltage();
                break;
            default:
                value = leftMaster.getSelectedSensorPosition();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }

    /**
     * Checks whether the right side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean rightOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value;
        switch (controlMode) {
            case PERCENT_OUTPUT:
                value = rightMaster.getMotorOutputPercent();
                break;
            case VELOCITY:
                value = rightMaster.getSelectedSensorVelocity();
                break;
            case CURRENT:
                if (leftMaster instanceof BaseTalon) {
                    value = ((BaseTalon) rightMaster).getStatorCurrent();
                    break;
                }
            case VOLTAGE:
                value = rightMaster.getBusVoltage();
                break;
            default:
                value = rightMaster.getSelectedSensorPosition();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }
}

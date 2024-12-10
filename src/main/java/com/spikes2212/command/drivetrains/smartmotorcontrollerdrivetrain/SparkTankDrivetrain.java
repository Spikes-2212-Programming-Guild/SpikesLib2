package com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.List;

/**
 * A {@link TankDrivetrain}, whose sides each consists of a master {@link CANSparkBase} motor controller that runs
 * control loops and additional {@link CANSparkBase} motor controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see TankDrivetrain
 * @see SmartMotorControllerTankDrivetrain
 */
public class SparkTankDrivetrain extends TankDrivetrain implements SmartMotorControllerTankDrivetrain {

    /**
     * The slot on the motor controller on which the trapezoid profiling configurations are saved.
     */
    private static final int TRAPEZOID_SLOT_ID = 0;

    /**
     * The left motor controller which runs the loops.
     */
    protected final CANSparkBase leftMaster;

    /**
     * The right motor controller which runs the loops.
     */
    protected final CANSparkBase rightMaster;

    /**
     * Additional motor controllers that follow the left master.
     */
    protected final List<? extends CANSparkBase> leftSlaves;

    /**
     * Additional motor controllers that follow the right master.
     */
    protected final List<? extends CANSparkBase> rightSlaves;

    /**
     * Constructs a new instance of {@link SparkTankDrivetrain}.
     *
     * @param namespaceName the name of the drivetrain's namespace
     * @param leftMaster    the {@link CANSparkBase} motor controller which runs the left side's loops
     * @param leftSlaves    additional {@link CANSparkBase} motor controllers that follow the left master
     * @param rightMaster   the {@link CANSparkBase} motor controller which runs the right side's loops
     * @param rightSlaves   additional {@link CANSparkBase} motor controllers that follow the right master
     */
    public SparkTankDrivetrain(String namespaceName, CANSparkBase leftMaster, List<? extends CANSparkBase> leftSlaves,
                               CANSparkBase rightMaster, List<? extends CANSparkBase> rightSlaves) {
        super(namespaceName, leftMaster, rightMaster);
        this.leftMaster = leftMaster;
        this.rightMaster = rightMaster;
        this.leftSlaves = leftSlaves;
        this.rightSlaves = rightSlaves;
        this.leftSlaves.forEach(s -> s.follow(leftMaster));
        this.rightSlaves.forEach(s -> s.follow(rightMaster));
        rightSlaves.forEach(s -> s.setInverted(true));
    }

    /**
     * Constructs a new instance of {@link SparkTankDrivetrain}.
     *
     * @param namespaceName the name of the drivetrain's namespace
     * @param leftMaster    the {@link CANSparkBase} motor controller which runs the left side's loops
     * @param leftSlave     an additional {@link CANSparkBase} motor controller that follows the left master
     * @param rightMaster   the {@link CANSparkBase} motor controller which runs the right side's loops
     * @param rightSlave    an additional {@link CANSparkBase} motor controller that follows the right master
     */
    public SparkTankDrivetrain(String namespaceName, CANSparkBase leftMaster, CANSparkBase leftSlave,
                               CANSparkBase rightMaster, CANSparkBase rightSlave) {
        this(namespaceName, leftMaster, List.of(leftSlave), rightMaster, List.of(rightSlave));
    }

    /**
     * Adds any data or commands to the {@link NetworkTable}s, which can be accessed using the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {
    }

    /**
     * Configures the loops' PID constants and feed forward gains.
     */
    @Override
    public void configPIDF(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                           FeedForwardSettings leftFeedForwardSettings, FeedForwardSettings rightFeedForwardSettings) {
        leftMaster.getPIDController().setFF(leftFeedForwardSettings.getkV());
        leftMaster.getPIDController().setP(leftPIDSettings.getkP());
        leftMaster.getPIDController().setI(leftPIDSettings.getkI());
        leftMaster.getPIDController().setD(leftPIDSettings.getkD());
        rightMaster.getPIDController().setFF(rightFeedForwardSettings.getkV());
        rightMaster.getPIDController().setP(rightPIDSettings.getkP());
        rightMaster.getPIDController().setI(rightPIDSettings.getkI());
        rightMaster.getPIDController().setD(rightPIDSettings.getkD());
    }

    /**
     * Configures the loops' trapezoid profile settings.
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings leftSettings, TrapezoidProfileSettings rightSettings) {
        leftMaster.getPIDController().setSmartMotionMaxAccel(leftSettings.getAccelerationRate(), TRAPEZOID_SLOT_ID);
        leftMaster.getPIDController().setSmartMotionMaxVelocity(leftSettings.getMaxVelocity(), TRAPEZOID_SLOT_ID);
        leftMaster.getPIDController().setSmartMotionAccelStrategy(
                SparkPIDController.AccelStrategy.fromInt((int) leftSettings.getCurve()), TRAPEZOID_SLOT_ID);
        rightMaster.getPIDController().setSmartMotionMaxAccel(rightSettings.getAccelerationRate(), TRAPEZOID_SLOT_ID);
        rightMaster.getPIDController().setSmartMotionMaxVelocity(rightSettings.getMaxVelocity(), TRAPEZOID_SLOT_ID);
        rightMaster.getPIDController().setSmartMotionAccelStrategy(
                SparkPIDController.AccelStrategy.fromInt((int) rightSettings.getCurve()), TRAPEZOID_SLOT_ID);
    }

    /**
     * Configures the loops' settings.
     */
    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        leftMaster.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        rightMaster.setInverted(true);
        rightSlaves.forEach(s -> s.setInverted(true));
        leftSlaves.forEach(s -> s.follow(leftMaster));
        rightSlaves.forEach(s -> s.follow(rightMaster));
    }

    /**
     * Updates any control loops running on the master {@link CANSparkBase} motor controllers.
     *
     * @param controlMode              the loop's control type (e.g. voltage, velocity, position...)
     * @param leftSetpoint             the left side loop's target setpoint
     * @param rightSetpoint            the right side loop's target setpoint
     * @param leftPIDSettings          the left side's PID constants
     * @param rightPIDSettings         the right side's PID constants
     * @param leftFeedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings leftFeedForwardSettings, FeedForwardSettings rightFeedForwardSettings,
                       TrapezoidProfileSettings leftTrapezoidProfileSettings,
                       TrapezoidProfileSettings rightTrapezoidProfileSettings) {
        configPIDF(leftPIDSettings, rightPIDSettings, leftFeedForwardSettings, rightFeedForwardSettings);
        configureTrapezoid(leftTrapezoidProfileSettings);
        FeedForwardController leftFeedForwardController = new FeedForwardController(leftFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        FeedForwardController rightFeedForwardController = new FeedForwardController(rightFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        leftMaster.getPIDController().setReference(leftSetpoint, controlMode.getSparkMaxControlType(), 0,
                leftFeedForwardController.calculate(leftSetpoint));
        rightMaster.getPIDController().setReference(rightSetpoint, controlMode.getSparkMaxControlType(), 0,
                rightFeedForwardController.calculate(rightSetpoint));
    }

    /**
     * Stops any control loops running on each side's master motor controller.
     */
    @Override
    public void finish() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();
    }

    /**
     * Checks whether the left side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the left target to still consider the left loop to be on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean leftOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case PERCENT_OUTPUT -> leftMaster.getAppliedOutput();
            case VELOCITY -> leftMaster.getEncoder().getVelocity();
            case CURRENT -> leftMaster.getOutputCurrent();
            case VOLTAGE -> leftMaster.getBusVoltage() * leftMaster.getAppliedOutput();
            default -> leftMaster.getEncoder().getPosition();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }

    /**
     * Checks whether the right side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the right target to still consider the right loop to be on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean rightOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case PERCENT_OUTPUT -> rightMaster.getAppliedOutput();
            case VELOCITY -> rightMaster.getEncoder().getVelocity();
            case CURRENT -> rightMaster.getOutputCurrent();
            case VOLTAGE -> rightMaster.getBusVoltage() * rightMaster.getAppliedOutput();
            default -> rightMaster.getEncoder().getPosition();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }
}

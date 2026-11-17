package com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SimulatedSmartMotorControllerWrapper;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A simulated {@link TankDrivetrain} that runs closed-loop control using
 * {@link SimulatedSmartMotorControllerWrapper} instances.
 *
 * @author Adi Yekutiel
 * @see TankDrivetrain
 * @see SmartMotorControllerTankDrivetrain
 */
public class SimulatedTankDrivetrain extends TankDrivetrain implements SmartMotorControllerTankDrivetrain {

    private static final double DEFAULT_DT_SECONDS = 0.02;
    private static final boolean UPDATE_PERIODICALLY = true;

    protected final SimulatedSmartMotorControllerWrapper leftMaster;
    protected final SimulatedSmartMotorControllerWrapper rightMaster;

    public SimulatedTankDrivetrain(String namespaceName, SimulatedSmartMotorControllerWrapper leftMaster,
                                   SimulatedSmartMotorControllerWrapper rightMaster) {
        super(namespaceName, leftMaster, rightMaster);
        this.leftMaster = leftMaster;
        this.rightMaster = rightMaster;
    }

    public SimulatedTankDrivetrain(String namespaceName, DCMotor motor, double gearRatio, double moi) {
        this(
                namespaceName,
                new SimulatedSmartMotorControllerWrapper(motor, gearRatio, moi),
                new SimulatedSmartMotorControllerWrapper(motor, gearRatio, moi)
        );
    }

    public void configurePID(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings) {
        leftMaster.configurePID(leftPIDSettings);
        rightMaster.configurePID(rightPIDSettings);
    }

    public void configureFF(FeedForwardSettings leftFeedForwardSettings,
                            FeedForwardSettings rightFeedForwardSettings) {
        leftMaster.configureFF(leftFeedForwardSettings);
        rightMaster.configureFF(rightFeedForwardSettings);
    }

    public void configureFF(FeedForwardSettings feedForwardSettings) {
        configureFF(feedForwardSettings, feedForwardSettings);
    }

    public void configureTrapezoid(TrapezoidProfileSettings leftTrapezoidProfileSettings,
                                   TrapezoidProfileSettings rightTrapezoidProfileSettings) {
        leftMaster.configureTrapezoid(leftTrapezoidProfileSettings);
        rightMaster.configureTrapezoid(rightTrapezoidProfileSettings);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        configureTrapezoid(trapezoidProfileSettings, trapezoidProfileSettings);
    }

    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings leftFeedForwardSettings,
                              FeedForwardSettings rightFeedForwardSettings,
                              TrapezoidProfileSettings leftTrapezoidProfileSettings,
                              TrapezoidProfileSettings rightTrapezoidProfileSettings) {
        leftMaster.configureLoop(leftPIDSettings, leftFeedForwardSettings, leftTrapezoidProfileSettings);
        rightMaster.configureLoop(rightPIDSettings, rightFeedForwardSettings, rightTrapezoidProfileSettings);
    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        configureLoop(
                leftPIDSettings,
                rightPIDSettings,
                feedForwardSettings,
                feedForwardSettings,
                trapezoidProfileSettings,
                trapezoidProfileSettings
        );
    }

    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings leftFeedForwardSettings,
                       FeedForwardSettings rightFeedForwardSettings,
                       TrapezoidProfileSettings leftTrapezoidProfileSettings,
                       TrapezoidProfileSettings rightTrapezoidProfileSettings,
                       boolean updatePeriodically) {
        leftMaster.pidSet(
                controlMode,
                leftSetpoint,
                leftPIDSettings,
                leftFeedForwardSettings,
                leftTrapezoidProfileSettings,
                updatePeriodically
        );
        rightMaster.pidSet(
                controlMode,
                rightSetpoint,
                rightPIDSettings,
                rightFeedForwardSettings,
                rightTrapezoidProfileSettings,
                updatePeriodically
        );
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings feedForwardSettings,
                       TrapezoidProfileSettings trapezoidProfileSettings) {
        pidSet(
                controlMode,
                leftSetpoint,
                rightSetpoint,
                leftPIDSettings,
                rightPIDSettings,
                feedForwardSettings,
                feedForwardSettings,
                trapezoidProfileSettings,
                trapezoidProfileSettings,
                UPDATE_PERIODICALLY
        );
    }

    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings leftFeedForwardSettings,
                       FeedForwardSettings rightFeedForwardSettings,
                       boolean updatePeriodically) {
        leftMaster.pidSet(
                controlMode,
                leftSetpoint,
                leftPIDSettings,
                leftFeedForwardSettings,
                updatePeriodically
        );
        rightMaster.pidSet(
                controlMode,
                rightSetpoint,
                rightPIDSettings,
                rightFeedForwardSettings,
                updatePeriodically
        );
    }

    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings feedForwardSettings) {
        pidSet(
                controlMode,
                leftSetpoint,
                rightSetpoint,
                leftPIDSettings,
                rightPIDSettings,
                feedForwardSettings,
                feedForwardSettings,
                UPDATE_PERIODICALLY
        );
    }

    public void resetPosition() {
        leftMaster.resetPosition();
        rightMaster.resetPosition();
    }

    @Override
    public boolean leftOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        return leftMaster.onTarget(controlMode, tolerance, setpoint);
    }

    @Override
    public boolean rightOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        return rightMaster.onTarget(controlMode, tolerance, setpoint);
    }

    public boolean onTarget(UnifiedControlMode controlMode, double leftTolerance, double rightTolerance,
                            double leftSetpoint, double rightSetpoint) {
        return leftOnTarget(controlMode, leftTolerance, leftSetpoint)
                && rightOnTarget(controlMode, rightTolerance, rightSetpoint);
    }

    public boolean onTarget(UnifiedControlMode controlMode, double tolerance,
                            double leftSetpoint, double rightSetpoint) {
        return onTarget(controlMode, tolerance, tolerance, leftSetpoint, rightSetpoint);
    }

    public void update(double dtSeconds) {
        leftMaster.update(dtSeconds);
        rightMaster.update(dtSeconds);
    }

    @Override
    public void simulationPeriodic() {
        update(DEFAULT_DT_SECONDS);
    }

    public double getLeftPosition() {
        return leftMaster.getPosition();
    }

    public void setLeftPosition(double position) {
        leftMaster.setPosition(position);
    }

    public double getRightPosition() {
        return rightMaster.getPosition();
    }

    public void setRightPosition(double position) {
        rightMaster.setPosition(position);
    }

    public double getLeftVelocity() {
        return leftMaster.getVelocity();
    }

    public double getRightVelocity() {
        return rightMaster.getVelocity();
    }
}

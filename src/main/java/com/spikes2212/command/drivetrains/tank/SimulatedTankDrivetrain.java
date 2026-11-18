package com.spikes2212.command.drivetrains.tank;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.Simulatable;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SimulatedSmartMotorControllerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * A simulated {@link TankDrivetrain} that runs closed-loop control and tracks field pose using
 * {@link SimulatedSmartMotorControllerWrapper} instances.
 *
 * @author Adi Yekutiel
 * @see TankDrivetrain
 * @see SmartMotorControllerTankDrivetrain
 * @see Simulatable
 */
public class SimulatedTankDrivetrain extends TankDrivetrain
        implements SmartMotorControllerTankDrivetrain, Simulatable {

    private static final double DEFAULT_TRACK_WIDTH_METERS = 0.6;
    private static final double DEFAULT_WHEEL_RADIUS_METERS = 0.0762;
    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double CIRCUMFERENCE_MULTIPLIER = 2.0 * Math.PI;
    private static final double DEFAULT_VALUE = 0.0;
    private static final boolean UPDATE_PERIODICALLY = true;

    protected final SimulatedSmartMotorControllerWrapper leftMaster;
    protected final SimulatedSmartMotorControllerWrapper rightMaster;

    private final DifferentialDriveKinematics kinematics;
    private final Field2d field2d;
    private final StructPublisher<Pose2d> posePublisher;
    private final double wheelCircumferenceMeters;

    private Pose2d pose;
    private double yawVelocityDegreesPerSec;

    public SimulatedTankDrivetrain(String namespaceName, SimulatedSmartMotorControllerWrapper leftMaster,
                                   SimulatedSmartMotorControllerWrapper rightMaster,
                                   double trackWidthMeters, double wheelRadiusMeters) {
        super(namespaceName, leftMaster, rightMaster);
        this.leftMaster = leftMaster;
        this.rightMaster = rightMaster;
        this.kinematics = new DifferentialDriveKinematics(trackWidthMeters);
        this.field2d = new Field2d();
        this.wheelCircumferenceMeters = CIRCUMFERENCE_MULTIPLIER * wheelRadiusMeters;
        this.pose = new Pose2d();
        this.yawVelocityDegreesPerSec = DEFAULT_VALUE;
        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(namespaceName + "/pose", Pose2d.struct)
                .publish();
        this.posePublisher.set(this.pose);
        this.namespace.putData("field", field2d);
    }

    public SimulatedTankDrivetrain(String namespaceName, SimulatedSmartMotorControllerWrapper leftMaster,
                                   SimulatedSmartMotorControllerWrapper rightMaster) {
        this(
                namespaceName,
                leftMaster,
                rightMaster,
                DEFAULT_TRACK_WIDTH_METERS,
                DEFAULT_WHEEL_RADIUS_METERS
        );
    }

    public SimulatedTankDrivetrain(String namespaceName, DCMotor motor, double gearRatio, double moi,
                                   double trackWidthMeters, double wheelRadiusMeters) {
        this(
                namespaceName,
                new SimulatedSmartMotorControllerWrapper(motor, gearRatio, moi),
                new SimulatedSmartMotorControllerWrapper(motor, gearRatio, moi),
                trackWidthMeters,
                wheelRadiusMeters
        );
    }

    public SimulatedTankDrivetrain(String namespaceName, DCMotor motor, double gearRatio, double moi) {
        this(
                namespaceName,
                motor,
                gearRatio,
                moi,
                DEFAULT_TRACK_WIDTH_METERS,
                DEFAULT_WHEEL_RADIUS_METERS
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

    @Override
    public void update(double dtSeconds) {
        leftMaster.update(dtSeconds);
        rightMaster.update(dtSeconds);

        double leftVelocityMetersPerSec = (leftMaster.getVelocity() / SECONDS_PER_MINUTE) * wheelCircumferenceMeters;
        double rightVelocityMetersPerSec = (rightMaster.getVelocity() / SECONDS_PER_MINUTE) * wheelCircumferenceMeters;
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
                leftVelocityMetersPerSec,
                rightVelocityMetersPerSec
        );
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
        yawVelocityDegreesPerSec = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);

        Twist2d twist = new Twist2d(
                chassisSpeeds.vxMetersPerSecond * dtSeconds,
                DEFAULT_VALUE,
                chassisSpeeds.omegaRadiansPerSecond * dtSeconds
        );
        pose = pose.exp(twist);
        field2d.setRobotPose(pose);
        posePublisher.set(pose);
    }

    @Override
    public void simulationPeriodic() {
        update();
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

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
        field2d.setRobotPose(pose);
    }

    public double getYawVelocityDegreesPerSec() {
        return yawVelocityDegreesPerSec;
    }

    public Field2d getField2d() {
        return field2d;
    }
}

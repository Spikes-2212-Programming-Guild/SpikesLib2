package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.Simulatable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * A simulated {@link SwerveDrivetrain} that steps {@link SimulatedSwerveModule} instances,
 * tracks field pose, and publishes 2D and 3D telemetry.
 *
 * @author Adi Yekutiel
 * @see SwerveDrivetrain
 * @see Simulatable
 */
public class SimulatedSwerveDrivetrain extends SwerveDrivetrain implements Simulatable {

    private static final double DEFAULT_TRACK_WIDTH_METERS = 0.6;
    private static final double DEFAULT_TRACK_LENGTH_METERS = 0.6;
    private static final double DEFAULT_MAX_VELOCITY_MPS = 4.5;
    private static final double DEFAULT_VALUE = 0.0;

    protected final SimulatedSwerveModule simulatedFrontLeftModule;
    protected final SimulatedSwerveModule simulatedFrontRightModule;
    protected final SimulatedSwerveModule simulatedBackLeftModule;
    protected final SimulatedSwerveModule simulatedBackRightModule;

    private final Field2d field2d;
    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<SwerveModuleState> currentStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;

    private Pose2d pose;
    private Rotation2d gyroOffset;
    private double yawVelocityDegreesPerSec;

    public SimulatedSwerveDrivetrain(String namespaceName,
                                     SimulatedSwerveModule frontLeftModule,
                                     SimulatedSwerveModule frontRightModule,
                                     SimulatedSwerveModule backLeftModule,
                                     SimulatedSwerveModule backRightModule,
                                     double drivetrainTrackWidth,
                                     double drivetrainTrackLength,
                                     double maxPossibleVelocity) {
        super(
                namespaceName,
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
                drivetrainTrackWidth,
                drivetrainTrackLength,
                maxPossibleVelocity
        );
        this.simulatedFrontLeftModule = frontLeftModule;
        this.simulatedFrontRightModule = frontRightModule;
        this.simulatedBackLeftModule = backLeftModule;
        this.simulatedBackRightModule = backRightModule;

        this.field2d = new Field2d();
        this.pose = new Pose2d();
        this.gyroOffset = new Rotation2d();
        this.yawVelocityDegreesPerSec = DEFAULT_VALUE;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        this.posePublisher = nt.getStructTopic(namespaceName + "/pose", Pose2d.struct).publish();
        this.currentStatesPublisher = nt.getStructArrayTopic(
                namespaceName + "/currentStates",
                SwerveModuleState.struct
        ).publish();
        this.targetStatesPublisher = nt.getStructArrayTopic(
                namespaceName + "/targetStates",
                SwerveModuleState.struct
        ).publish();

        this.posePublisher.set(this.pose);
        this.currentStatesPublisher.set(getCurrentModuleStates());
        this.targetStatesPublisher.set(getTargetModuleStates());
        this.namespace.putData("field", field2d);
    }

    public SimulatedSwerveDrivetrain(String namespaceName,
                                     SimulatedSwerveModule frontLeftModule,
                                     SimulatedSwerveModule frontRightModule,
                                     SimulatedSwerveModule backLeftModule,
                                     SimulatedSwerveModule backRightModule) {
        this(
                namespaceName,
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
                DEFAULT_TRACK_WIDTH_METERS,
                DEFAULT_TRACK_LENGTH_METERS,
                DEFAULT_MAX_VELOCITY_MPS
        );
    }

    public SimulatedSwerveDrivetrain(String namespaceName,
                                     PIDSettings drivePIDSettings,
                                     PIDSettings turnPIDSettings,
                                     FeedForwardSettings driveFFSettings,
                                     FeedForwardSettings turnFFSettings) {
        this(
                namespaceName,
                new SimulatedSwerveModule(
                        namespaceName + "/front left",
                        drivePIDSettings,
                        turnPIDSettings,
                        driveFFSettings,
                        turnFFSettings
                ),
                new SimulatedSwerveModule(
                        namespaceName + "/front right",
                        drivePIDSettings,
                        turnPIDSettings,
                        driveFFSettings,
                        turnFFSettings
                ),
                new SimulatedSwerveModule(
                        namespaceName + "/back left",
                        drivePIDSettings,
                        turnPIDSettings,
                        driveFFSettings,
                        turnFFSettings
                ),
                new SimulatedSwerveModule(
                        namespaceName + "/back right",
                        drivePIDSettings,
                        turnPIDSettings,
                        driveFFSettings,
                        turnFFSettings
                )
        );
    }

    @Override
    public void resetAngleSensor() {
        gyroOffset = pose.getRotation();
    }

    @Override
    public Rotation2d getAngle() {
        return pose.getRotation().minus(gyroOffset);
    }

    @Override
    public void update(double dtSeconds) {
        simulatedFrontLeftModule.update(dtSeconds);
        simulatedFrontRightModule.update(dtSeconds);
        simulatedBackLeftModule.update(dtSeconds);
        simulatedBackRightModule.update(dtSeconds);

        ChassisSpeeds chassisSpeeds = getCurrentSpeeds();
        yawVelocityDegreesPerSec = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);

        Twist2d twist = new Twist2d(
                chassisSpeeds.vxMetersPerSecond * dtSeconds,
                chassisSpeeds.vyMetersPerSecond * dtSeconds,
                chassisSpeeds.omegaRadiansPerSecond * dtSeconds
        );
        pose = pose.exp(twist);

        field2d.setRobotPose(pose);
        posePublisher.set(pose);
        currentStatesPublisher.set(getCurrentModuleStates());
        targetStatesPublisher.set(getTargetModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        update();
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
        this.gyroOffset = new Rotation2d();
        field2d.setRobotPose(pose);
        posePublisher.set(pose);
    }

    public double getYawVelocityDegreesPerSec() {
        return yawVelocityDegreesPerSec;
    }

    public Field2d getField2d() {
        return field2d;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("yaw", () -> getAngle().getDegrees());
    }
}

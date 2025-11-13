package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A command that moves a {@link SwerveDrivetrain} to a certain angle using pid to position for each one of the
 * {@link SwerveModule}.
 *
 * @author Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class TurnToDegree extends InstantCommand {

    private final SwerveDrivetrain drivetrain;
    private final double desiredDegrees;

    /**
     * Constructs a new {@link TurnToDegree} command that moves the given {@link SwerveDrivetrain}
     * {@link SwerveModule} to a certain degree.
     *
     * @param drivetrain the swerve drivetrain this command operates on
     */
    public TurnToDegree(SwerveDrivetrain drivetrain, double desiredDegrees) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.desiredDegrees = desiredDegrees;
    }

    /**
     * Moves each {@link SwerveModule} in the given {@link SwerveDrivetrain}
     * to the desired angle in degrees.
     */
    @Override
    public void initialize() {
        drivetrain.resetRelativeEncoders();

        drivetrain.getFrontLeftModule().setTargetAngle(
                Rotation2d.fromDegrees(desiredDegrees));
        drivetrain.getFrontRightModule().setTargetAngle(
                Rotation2d.fromDegrees(desiredDegrees));
        drivetrain.getBackLeftModule().setTargetAngle(
                Rotation2d.fromDegrees(desiredDegrees));
        drivetrain.getBackRightModule().setTargetAngle(
                Rotation2d.fromDegrees(desiredDegrees));
    }
}

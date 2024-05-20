package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.OdometryDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.path.PurePursuitController;
import com.spikes2212.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

public class FollowPath extends Command {

    private static final double PERIODIC_RATE = 0.02;

    private OdometryDrivetrain drivetrain;
    private List<Waypoint> path;
    private double lookaheadDistance;
    private double maxAcceleration;
    private PurePursuitController purePursuitController;
    private FeedForwardController rightFeedForwardController;
    private FeedForwardController leftFeedForwardController;
    private PIDSettings pidSettings;
    private FeedForwardSettings FeedForwardSettings;
    private PIDController leftController;
    private PIDController rightController;

    public FollowPath(OdometryDrivetrain drivetrain, List<Waypoint> path, double lookaheadDistance,
                      PIDSettings pidSettings, FeedForwardSettings feedForwardSettings, double maxAcceleration,
                      boolean inverted) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.pidSettings = pidSettings;
        this.FeedForwardSettings = feedForwardSettings;
        this.maxAcceleration = maxAcceleration;
        drivetrain.setInverted(inverted);
    }

    @Override
    public void initialize() {
        drivetrain.zeroSensors();
        purePursuitController = new PurePursuitController(drivetrain.getHandler(), path,
                lookaheadDistance, maxAcceleration, drivetrain.getWidth());
        purePursuitController.getOdometryHandler().set(0, 0);
        purePursuitController.reset();
        leftFeedForwardController = new FeedForwardController(FeedForwardSettings.getkV(), FeedForwardSettings.getkA(),
                FeedForwardController.DEFAULT_PERIOD);
        rightFeedForwardController = new FeedForwardController(FeedForwardSettings.getkV(), FeedForwardSettings.getkA(),
                FeedForwardController.DEFAULT_PERIOD);
        leftFeedForwardController.reset();
        rightFeedForwardController.reset();
        leftController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        rightController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    @Override
    public void execute() {
        double[] speeds = purePursuitController.getTargetSpeeds();
        double leftSpeed = leftFeedForwardController.calculate(speeds[0], drivetrain.getLeftRate())
                + leftController.calculate(drivetrain.getLeftRate(), speeds[0]);
        double rightSpeed = rightFeedForwardController.calculate(speeds[1], drivetrain.getRightRate())
                + rightController.calculate(drivetrain.getRightRate(), speeds[1]);
        drivetrain.tankDrive(leftSpeed, rightSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return purePursuitController.done();
    }
}

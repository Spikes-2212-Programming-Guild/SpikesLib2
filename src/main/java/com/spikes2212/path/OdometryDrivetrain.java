package com.spikes2212.path;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class OdometryDrivetrain extends TankDrivetrain {

    private OdometryHandler handler;
    private double width;

    public OdometryDrivetrain(SpeedController left, SpeedController right, OdometryHandler handler, double width) {
        super(left, right);
        this.handler = handler;
        this.width = width;
    }

    public OdometryHandler getHandler() {
        return handler;
    }

    public void setHandler(OdometryHandler handler) {
        this.handler = handler;
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    public abstract void resetSensors();

    public abstract double getLeftRate();

    public abstract double getRightRate();
}

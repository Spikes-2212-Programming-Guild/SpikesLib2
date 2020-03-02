package com.spikes2212.command.drivetrains;

import com.spikes2212.path.OdometryHandler;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class OdometryDrivetrain extends TankDrivetrain {

    public OdometryDrivetrain(SpeedController left, SpeedController right) {
        super(left, right);
    }

    public abstract OdometryHandler getHandler();

    public abstract double getWidth();

    public abstract void zeroSensors();

    public abstract double getLeftRate();

    public abstract double getRightRate();

    public abstract void setInverted(boolean inverted);

    @Override
    public void periodic() {
        getHandler().calculate();
    }
}

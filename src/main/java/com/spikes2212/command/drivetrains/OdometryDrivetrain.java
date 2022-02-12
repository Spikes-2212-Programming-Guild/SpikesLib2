package com.spikes2212.command.drivetrains;

import com.spikes2212.path.OdometryHandler;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class OdometryDrivetrain extends TankDrivetrain {

    public OdometryDrivetrain(String namespaceName, MotorController left, MotorController right) {
        super(left, right, namespaceName);
    }

    public abstract OdometryHandler getHandler();

    public abstract double getWidth();

    public abstract void zeroSensors();

    public abstract double getLeftRate();

    public abstract double getRightRate();

    public abstract void setInverted(boolean inverted);

    @Override
    public void periodic() {
        super.periodic();
        getHandler().calculate();
    }
}

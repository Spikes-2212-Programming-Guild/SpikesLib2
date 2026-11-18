package com.spikes2212.util.gyro;

import com.spikes2212.util.Simulatable;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A gyroscope sensor that measures orientation across three axes.
 *
 * @author Adi Yekutiel
 */
public interface Gyro extends Simulatable {

    void reset();

    void calibrate(double yaw);

    void calibrate();

    void update(double dtSeconds);

    double getX();

    double getY();

    double getZ();

    double getYaw();

    void setYaw(double yaw);

    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }
}

package com.spikes2212.command.drivetrains;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class represents a type of Drivetrain that its left and right sides are
 * controlled independently, allowing it to move by giving each side a speed
 * value separately.
 * <p>
 * It can move forwards/backwards by giving both its sides an equal speed or
 * turn by giving the sides different speeds
 *
 * @author Yuval Levy
 */
public class TankDrivetrain extends SubsystemBase {

    protected SpeedController leftController;
    protected SpeedController rightController;
    private DifferentialDrive drive;

    public TankDrivetrain(SpeedController left, SpeedController right) {
        this.leftController = left;
        this.rightController = right;
        drive = new DifferentialDrive(leftController, rightController);
    }

    /**
     * Moves both sides of this drivetrain by the given speeds for each side.
     *
     * @param speedLeft  the speed to set to the left side. Positive values move this side
     *                   forward.
     * @param speedRight the speed to set to the right side. Positive values move this side
     *                   forward.
     */
    public void tankDrive(double speedLeft, double speedRight) {
        drive.tankDrive(speedLeft, speedRight);
    }

    /**
     * Moves both sides of this drivetrain by the given speeds for each side.
     *
     * @param speedLeft  the speed to set to the left side. Positive values move this side
     *                   forward.
     * @param speedRight the speed to set to the right side. Positive values move this side
     *                   forward.
     * @param squareInputs whether to square the given inputs before putting them in the speed controllers
     */
    public void tankDrive(double speedLeft, double speedRight, boolean squareInputs) {
        drive.tankDrive(speedLeft, speedRight, squareInputs);
    }

    /**
     * Moves the drivetrain with the given forward and angular speed.
     *
     * @param moveValue   the forward movement speed.
     * @param rotateValue the angular movement speed.
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        drive.arcadeDrive(moveValue, rotateValue);
    }

    /**
     * Moves both sides of this drivetrain by the given speeds for each side.
     *
     * @param moveValue   the forward movement speed.
     * @param rotateValue the angular movement speed.
     * @param squareInputs whether to square the given inputs before putting them in the speed controllers
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squareInputs) {
        drive.arcadeDrive(moveValue, rotateValue, squareInputs);
    }

    public void curvatureDrive(double speed, double curvature) {
        drive.curvatureDrive(speed, curvature, true);
    }

    /**
     * Moves the left side of this drivetrain by a given speed.
     *
     * @param speedLeft the speed to set to the left side. Positive values move this side
     *                  forward.
     */
    public void setLeft(double speedLeft) {
        leftController.set(speedLeft);
    }

    /**
     * Moves the right side of this drivetrain with a given speed.
     *
     * @param speedRight the speed to set to the right side. Positive values move this side
     *                   forward.
     */
    public void setRight(double speedRight) {
        rightController.set(-speedRight);
    }

    /**
     * Stop the drivetrain's movement completely.
     */
    public void stop() {
        leftController.stopMotor();
        rightController.stopMotor();
    }
}

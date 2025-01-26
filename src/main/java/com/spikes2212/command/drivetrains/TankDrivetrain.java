package com.spikes2212.command.drivetrains;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.MotorControllerGroup;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * This class represents a type of drivetrain that its left and right sides are controlled independently,
 * allowing it to move by giving each side a speed value separately.
 *
 * <p> It can move forwards/backwards by giving both its sides an equal speed or
 * turn by giving the sides different speeds. </p>
 *
 * @author Yuval Levy
 * @see DashboardedSubsystem
 */
public class TankDrivetrain extends DashboardedSubsystem {

    private static final String DEFAULT_NAMESPACE_NAME = "tank drivetrain";

    protected final MotorController leftController;
    protected final MotorController rightController;

    private final DifferentialDrive drive;

    public TankDrivetrain(String namespaceName, MotorController leftController, MotorController rightController) {
        super(namespaceName);
        this.leftController = leftController;
        this.rightController = rightController;
        rightController.setInverted(true);
        drive = new DifferentialDrive(leftController, rightController);
        drive.setSafetyEnabled(false);
    }

    public TankDrivetrain(MotorController leftController, MotorController rightController) {
        this(getClassName(DEFAULT_NAMESPACE_NAME), leftController, rightController);
    }

    public TankDrivetrain(String namespaceName, MotorController leftMaster, MotorController leftSlave,
                          MotorController rightMaster, MotorController rightSlave) {
        this(namespaceName, new MotorControllerGroup(leftMaster, leftSlave),
                new MotorControllerGroup(rightMaster, rightSlave));
    }

    public TankDrivetrain(MotorController leftMaster, MotorController leftSlave,
                          MotorController rightMaster, MotorController rightSlave) {
        this(new MotorControllerGroup(leftMaster, leftSlave),
                new MotorControllerGroup(rightMaster, rightSlave));
    }

    /**
     * Moves both sides of this drivetrain by the given speeds for each side.
     *
     * @param leftSpeed  the speed to set to the left side (-1 to 1). Positive values move this side forward
     * @param rightSpeed the speed to set to the right side (-1 to 1). Positive values move this side forward
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed, false);
    }

    /**
     * Moves both sides of this drivetrain by the given speeds for each side.
     *
     * @param leftSpeed    the speed to set to the left side (-1 to 1). Positive values move this side forward
     * @param rightSpeed   the speed to set to the right side (-1 to 1). Positive values move this side forward
     * @param squareInputs whether to square the given inputs before putting them in the speed controllers
     */
    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        drive.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }

    /**
     * Moves both sides of this drivetrain by the given voltages for each side.
     *
     * @param leftVoltage  the voltage to set to the left side (-12 to 12). Positive values move this side forward
     * @param rightVoltage the voltage to set to the right side (-12 to 12). Positive values move this side forward
     */
    public void tankDriveVoltages(double leftVoltage, double rightVoltage) {
        tankDrive(leftVoltage / RobotController.getBatteryVoltage(),
                rightVoltage / RobotController.getBatteryVoltage(), false);
    }

    /**
     * Moves the drivetrain by the given forward and angular speed.
     *
     * @param moveValue   the forward movement speed (-1 to 1)
     * @param rotateValue the angular movement speed (-1 to 1). Positive values go clockwise
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        drive.arcadeDrive(moveValue, rotateValue, false);
    }

    /**
     * Moves the drivetrain by the given forward and angular speed.
     *
     * @param moveValue    the forward movement speed (-1 to 1)
     * @param rotateValue  the angular movement speed (-1 to 1). Positive values go clockwise
     * @param squareInputs whether to square the given inputs before putting them in the speed controllers
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squareInputs) {
        drive.arcadeDrive(moveValue, rotateValue, squareInputs);
    }

    /**
     * Moves the drivetrain by the given forward and angular voltage.
     *
     * @param moveVoltage   the forward movement voltage (-12 to 12)
     * @param rotateVoltage the angular movement voltage (-12 to 12). Positive values go clockwise
     */
    public void arcadeDriveVoltages(double moveVoltage, double rotateVoltage) {
        arcadeDrive(moveVoltage / RobotController.getBatteryVoltage(),
                rotateVoltage / RobotController.getBatteryVoltage(), false);
    }

    /**
     * Moves the drivetrain while rotating it at a given curvature.
     *
     * @param speed     the forward movement speed (-1 to 1)
     * @param curvature the curvature of the robot's path (-1 to 1). Positive values go clockwise
     */
    public void curvatureDrive(double speed, double curvature) {
        drive.curvatureDrive(speed, curvature, true);
    }

    /**
     * Moves the left side of this drivetrain by a given speed.
     *
     * @param leftSpeed the speed to set to the left side (-1 to 1). Positive values move this side forward
     */
    public void setLeft(double leftSpeed) {
        leftController.set(leftSpeed);
    }

    /**
     * Moves the right side of this drivetrain by a given speed.
     *
     * @param rightSpeed the speed to set to the right side (-1 to 1). Positive values move this side forward
     */
    public void setRight(double rightSpeed) {
        rightController.set(rightSpeed);
    }

    /**
     * Moves the left side of this drivetrain by a given voltage.
     *
     * @param leftVoltage the voltage to set to the left side (-12 to 12). Positive values move this side forward
     */
    public void setLeftVoltage(double leftVoltage) {
        setLeft(leftVoltage / RobotController.getBatteryVoltage());
    }

    /**
     * Moves the right side of this drivetrain by a given voltage.
     *
     * @param rightVoltage the voltage to set to the right side (-12 to 12). Positive values move this side forward
     */
    public void setRightVoltage(double rightVoltage) {
        setRight(rightVoltage / RobotController.getBatteryVoltage());
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        leftController.stopMotor();
        rightController.stopMotor();
    }

    /**
     * Sets the motor safety feature of the speed controllers on/off.
     *
     * @param enabled whether motor safety should be enabled
     */
    public void setMotorSafety(boolean enabled) {
        drive.setSafetyEnabled(enabled);
    }

    /**
     * Adds any commands or data from this subsystem to the {@link NetworkTable}s.
     */
    @Override
    public void configureDashboard() {
    }
}

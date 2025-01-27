package com.spikes2212.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A class that handles the communication between an Xbox 360 or Xbox One controller and the code.
 *
 * @author Tal Sitton
 */
public class XboxControllerWrapper extends Joystick {

    protected final XboxController xbox;

    /**
     * Constructs a new {@link XboxControllerWrapper} using the port of the USB on the {@link DriverStation}.
     *
     * @param port the port on the {@link DriverStation} that the controller is plugged in
     */
    public XboxControllerWrapper(int port) {
        super(port);
        xbox = new XboxController(port);
    }

    public JoystickButton getAButton() {
        return new JoystickButton(this, XboxController.Button.kA.value);
    }

    public JoystickButton getXButton() {
        return new JoystickButton(this, XboxController.Button.kX.value);
    }

    public JoystickButton getBButton() {
        return new JoystickButton(this, XboxController.Button.kB.value);
    }

    public JoystickButton getYButton() {
        return new JoystickButton(this, XboxController.Button.kY.value);
    }

    public JoystickButton getButtonStart() {
        return new JoystickButton(this, XboxController.Button.kStart.value);
    }

    public JoystickButton getButtonBack() {
        return new JoystickButton(this, XboxController.Button.kBack.value);
    }

    public double getRTAxis() {
        return xbox.getRightTriggerAxis();
    }

    public double getLTAxis() {
        return xbox.getLeftTriggerAxis();
    }

    public Trigger getRTButton() {
        return new Trigger(() -> xbox.getRightTriggerAxis() == 1);
    }

    public Trigger getLTButton() {
        return new Trigger(() -> xbox.getLeftTriggerAxis() == 1);
    }

    public JoystickButton getRBButton() {
        return new JoystickButton(this, XboxController.Button.kRightBumper.value);
    }

    public JoystickButton getLBButton() {
        return new JoystickButton(this, XboxController.Button.kLeftBumper.value);
    }

    public JoystickButton getRightStickButton() {
        return new JoystickButton(this, XboxController.Button.kRightStick.value);
    }

    public JoystickButton getLeftStickButton() {
        return new JoystickButton(this, XboxController.Button.kLeftStick.value);
    }

    public double getRightX() {
        return xbox.getRightX();
    }

    public double getRightY() {
        return xbox.getRightY();
    }

    public double getLeftX() {
        return xbox.getLeftX();
    }

    public double getLeftY() {
        return xbox.getLeftY();
    }

    public Trigger getUpButton() {
        return new Trigger(() -> getPOV() == DPAD.UP.value);
    }

    public Trigger getDownButton() {
        return new Trigger(() -> getPOV() == DPAD.DOWN.value);
    }

    public Trigger getLeftButton() {
        return new Trigger(() -> getPOV() == DPAD.LEFT.value);
    }

    public Trigger getRightButton() {
        return new Trigger(() -> getPOV() == DPAD.RIGHT.value);
    }

    public Trigger getUpperRightButton() {
        return new Trigger(() -> getPOV() == DPAD.UPPER_RIGHT.value);
    }

    public Trigger getLowerRightButton() {
        return new Trigger(() -> getPOV() == DPAD.LOWER_RIGHT.value);
    }

    public Trigger getLowerLeftButton() {
        return new Trigger(() -> getPOV() == DPAD.LOWER_LEFT.value);
    }

    public Trigger getUpperLeftButton() {
        return new Trigger(() -> getPOV() == DPAD.UPPER_LEFT.value);
    }

    public void setRumble(double value) {
        xbox.setRumble(RumbleType.kLeftRumble, value);
        xbox.setRumble(RumbleType.kRightRumble, value);
    }

    public void timeRumble(double value, double time) {
        Command command = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    setRumble(value);
                }),
                new WaitCommand(time),
                new InstantCommand(() -> {
                    setRumble(0);
                })
        );
        command.schedule();
    }
}

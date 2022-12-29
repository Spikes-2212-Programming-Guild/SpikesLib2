package com.spikes2212.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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

    public JoystickButton getGreenTrigger() {
        return new JoystickButton(this, XboxController.Button.kA.value);
    }

    public JoystickButton getBlueTrigger() {
        return new JoystickButton(this, XboxController.Button.kX.value);
    }

    public JoystickButton getRedTrigger() {
        return new JoystickButton(this, XboxController.Button.kB.value);
    }

    public JoystickButton getYellowTrigger() {
        return new JoystickButton(this, XboxController.Button.kY.value);
    }

    public JoystickButton getTriggerStart() {
        return new JoystickButton(this, XboxController.Button.kStart.value);
    }

    public JoystickButton getTriggerBack() {
        return new JoystickButton(this, XboxController.Button.kBack.value);
    }

    public double getRTAxis() {
        return xbox.getRightTriggerAxis();
    }

    public double getLTAxis() {
        return xbox.getLeftTriggerAxis();
    }

    public Trigger getRTTrigger() {
        return new Trigger(() -> xbox.getRightTriggerAxis() == 1);
    }

    public Trigger getLTTrigger() {
        return new Trigger(() -> xbox.getLeftTriggerAxis() == 1);
    }

    public JoystickButton getRBTrigger() {
        return new JoystickButton(this, XboxController.Button.kRightBumper.value);
    }

    public JoystickButton getLBTrigger() {
        return new JoystickButton(this, XboxController.Button.kLeftBumper.value);
    }

    public JoystickButton getRightStickTrigger() {
        return new JoystickButton(this, XboxController.Button.kRightStick.value);
    }

    public JoystickButton getLeftStickTrigger() {
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

    public Trigger getUpTrigger() {
        return new Trigger(() -> getPOV() == DPAD.UP.VALUE);
    }

    public Trigger getDownTrigger() {
        return new Trigger(() -> getPOV() == DPAD.DOWN.VALUE);
    }


    public Trigger getLeftTrigger() {
        return new Trigger(() -> getPOV() == DPAD.LEFT.VALUE);
    }


    public Trigger getRightTrigger() {
        return new Trigger(() -> getPOV() == DPAD.RIGHT.VALUE);
    }

    public Trigger getUpperRightTrigger() {
        return new Trigger(() -> getPOV() == DPAD.UPPER_RIGHT.VALUE);
    }

    public Trigger getLowerRightTrigger() {
        return new Trigger(() -> getPOV() == DPAD.LOWER_RIGHT.VALUE);
    }

    public Trigger getLowerLeftTrigger() {
        return new Trigger(() -> getPOV() == DPAD.LOWER_LEFT.VALUE);
    }

    public Trigger getUpperLeftTrigger() {
        return new Trigger(() -> getPOV() == DPAD.UPPER_LEFT.VALUE);
    }
}

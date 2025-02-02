package com.spikes2212.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A class that handles the communication between a PS4 controller and the code.
 *
 * @author Ofri Rosenbaum
 */
public class PlaystationControllerWrapper extends Joystick {

    protected final PS4Controller ps;

    /**
     * Constructs a new {@link PS4Controller} using the port of the USB on the {@link DriverStation}.
     *
     * @param port the port on the {@link DriverStation} that the controller is plugged in
     */
    public PlaystationControllerWrapper(int port) {
        super(port);
        ps = new PS4Controller(port);
    }

    public JoystickButton getCrossButton() {
        return new JoystickButton(this, PS4Controller.Button.kCross.value);
    }

    public JoystickButton getSquareButton() {
        return new JoystickButton(this, PS4Controller.Button.kSquare.value);
    }

    public JoystickButton getCircleButton() {
        return new JoystickButton(this, PS4Controller.Button.kCircle.value);
    }

    public JoystickButton getTriangleButton() {
        return new JoystickButton(this, PS4Controller.Button.kTriangle.value);
    }

    public JoystickButton getOptionsButton() {
        return new JoystickButton(this, PS4Controller.Button.kOptions.value);
    }

    public JoystickButton getShareButton() {
        return new JoystickButton(this, PS4Controller.Button.kShare.value);
    }

    public double getR2Axis() {
        return ps.getR2Axis();
    }

    public double getL2Axis() {
        return ps.getL2Axis();
    }

    public Trigger getR2Button() {
        return new Trigger(ps::getR2Button);
    }

    public Trigger getL2Button() {
        return new Trigger(ps::getL2Button);
    }

    public JoystickButton getR1Button() {
        return new JoystickButton(this, PS4Controller.Button.kR1.value);
    }

    public JoystickButton getL1Button() {
        return new JoystickButton(this, PS4Controller.Button.kL1.value);
    }

    public JoystickButton getRightStickButton() {
        return new JoystickButton(this, PS4Controller.Button.kR3.value);
    }

    public JoystickButton getLeftStickButton() {
        return new JoystickButton(this, PS4Controller.Button.kL3.value);
    }

    public JoystickButton getPlaystationButton() {
        return new JoystickButton(this, PS4Controller.Button.kPS.value);
    }

    public JoystickButton getTouchpadButton() {
        return new JoystickButton(this, PS4Controller.Button.kTouchpad.value);
    }

    public double getRightX() {
        return ps.getRightX();
    }

    public double getRightY() {
        return ps.getRightY();
    }

    public double getLeftX() {
        return ps.getLeftX();
    }

    public double getLeftY() {
        return ps.getLeftY();
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
}

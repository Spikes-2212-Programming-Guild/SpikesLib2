package com.spikes2212.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

public class XboXUID extends Joystick {

    /**
     * Constructs a new {@link XboXUID} using the port of the USB on the driver
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    XboxController xbox;
    public XboXUID(int port) {
        super(port);
        xbox=new XboxController(port);
    }

    /**
     * Returns the green button on the joystick.
     *
     * @return the green button on the joystick.
     */
    public JoystickButton getGreenButton() {
        return new JoystickButton(this, XboxController.Button.kA.value);
    }

    /**
     * Returns the blue button on the joystick.
     *
     * @return the blue button on the joystick.
     */
    public JoystickButton getBlueButton() {
        return new JoystickButton(this, XboxController.Button.kX.value);
    }

    /**
     * Returns the red button on the joystick.
     *
     * @return the red button on the joystick.
     */
    public JoystickButton getRedButton() {
        return new JoystickButton(this, XboxController.Button.kB.value);
    }

    /**
     * Returns the yellow button on the joystick.
     *
     * @return the yellow button on the joystick.
     */
    public JoystickButton getYellowButton() {
		return new JoystickButton(this,XboxController.Button.kY.value);
    }

    /**
     * Returns the start button on the joystick.
     *
     * @return the start button on the joystick.
     */
    public JoystickButton getButtonStart() {
		return new JoystickButton(this,XboxController.Button.kStart.value);
    }

    /**
     * Returns the back button on the joystick.
     *
     * @return the back button on the joystick.
     */
    public JoystickButton getButtonBack() {
		return new JoystickButton(this,XboxController.Button.kBack.value);
    }

    /**
     * Returns the value of the right trigger on the joystick.
     *
     * @return the value of the right trigger on the joystick.
     */
    public double getRTAxis() {
        return xbox.getTriggerAxis(Hand.kRight);
    }

    /**
     * Returns the value of the left trigger on the joystick.
     *
     * @return the value of the left trigger on the joystick.
     */
    public double getLTAxis() {
        return xbox.getTriggerAxis(Hand.kLeft);
    }

    /**
     * Returns the right trigger button on the joystick.
     *
     * @return the right trigger on the joystick.
     */
    public Trigger getRTButton() {
        return new Trigger() {

            @Override
            public boolean get() {
                return xbox.getTriggerAxis(Hand.kRight) == 1;
            }
        };
    }

    /**
     * Returns the left trigger button on the joystick.
     *
     * @return the left trigger button on the joystick.
     */
    public Trigger getLTButton() {
        return new Trigger() {

            @Override
            public boolean get() {
                return xbox.getTriggerAxis(Hand.kLeft) == 1;
            }
        };
    }

    /**
     * Returns the right bumper button on the joystick.
     *
     * @return the right bumper button on the joystick.
     */
    public JoystickButton getRBButton() {
		return new JoystickButton(this,XboxController.Button.kBumperRight.value);
    }

    /**
     * Returns the left bumper button on the joystick.
     *
     * @return the left bumper button on the joystick.
     */
    public JoystickButton getLBButton() {
		return new JoystickButton(this,XboxController.Button.kBumperLeft.value);
    }

    /**
     * Returns the button on the right stick.
     *
     * @return the button on the right stick.
     */
    public JoystickButton getRightStickButton() {
		return new JoystickButton(this,XboxController.Button.kStickRight.value);
    }

    /**
     * Returns the button on the left stick.
     *
     * @return the button on the left stick.
     */
    public JoystickButton getLeftStickButton() {
		return new JoystickButton(this,XboxController.Button.kStickLeft.value);
    }

    /**
     * Get the X axis value of the right stick.
     *
     * @return the X axis value of the right stick.
     */
    public double getRightX() {
        return xbox.getX(Hand.kRight);
    }

    /**
     * Get the Y axis value of the right stick.
     *
     * @return the Y axis value of the right stick.
     */
    public double getRightY() {
        return xbox.getY(Hand.kRight);
    }

    /**
     * Get the X axis value of the left stick.
     *
     * @return the X axis value of the left stick.
     */
    public double getLeftX() {
        return xbox.getX(Hand.kLeft);
    }

    /**
     * Get the Y axis value of the left stick.
     *
     * @return the Y axis value of the left stick.
     */
    public double getLeftY() {
        return xbox.getY(Hand.kLeft);
    }

    /**
     * Returns the up arrow button.
     *
     * @return the up arrow button.
     */
    public Button getUpButton() {
		return new Button() {
			@Override
			public boolean get() {
				return getPOV() == 0;
			}
		};
    }

    /**
     * Returns the down arrow button.
     *
     * @return the down arrow button.
     */
    public Button getDownButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 180;
            }
        };
    }

    /**
     * Returns the left arrow button.
     *
     * @return the left arrow button.
     */
    public Button getLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 270;
            }
        };
    }

    /**
     * Returns the right arrow button.
     *
     * @return the right arrow button.
     */
    public Button getRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 90;
            }
        };
    }

    /**
     * Returns the upper right arrow button.
     *
     * @return the upper right arrow button.
     */
    public Button getUpperRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 45;
            }
        };
    }

    /**
     * Returns the lower right arrow button.
     *
     * @return the lower right arrow button.
     */
    public Button getLowerRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 135;
            }
        };
    }

    /**
     * Returns the lower left arrow button.
     *
     * @return the lower left arrow button.
     */
    public Button getLowerLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 225;
            }
        };
    }

    /**
     * Returns the upper left arrow button.
     *
     * @return the upper left arrow button.
     */
    public Button getUpperLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 315;
            }
        };
    }
}
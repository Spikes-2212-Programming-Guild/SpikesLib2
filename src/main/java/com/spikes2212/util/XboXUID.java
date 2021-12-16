package com.spikes2212.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * a class that handles the communication between an Xbox controller and the code
 * @author Tal Sitton
 */
public class XboXUID extends Joystick {

    private XboxController xbox;

    /**
     * Constructs a new {@link XboXUID} using the port of the USB on the driver
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public XboXUID(int port) {
        super(port);
        xbox = new XboxController(port);
    }

    public JoystickButton getGreenButton() {
        return new JoystickButton(this, XboxController.Button.kA.value);
    }

    public JoystickButton getBlueButton() {
        return new JoystickButton(this, XboxController.Button.kX.value);
    }

    public JoystickButton getRedButton() {
        return new JoystickButton(this, XboxController.Button.kB.value);
    }

    public JoystickButton getYellowButton() {
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
        return new Trigger() {

            @Override
            public boolean get() {
                return xbox.getRightTriggerAxis() == 1;
            }
        };
    }

    public Trigger getLTButton() {
        return new Trigger() {

            @Override
            public boolean get() {
                return xbox.getLeftTriggerAxis() == 1;
            }
        };
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

    public Button getUpButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.UP.value;
            }
        };
    }

    public Button getDownButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.DOWN.value;
            }
        };
    }


    public Button getLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.LEFT.value;
            }
        };
    }


    public Button getRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.RIGHT.value;
            }
        };
    }

    public Button getUpperRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.UPPER_RIGHT.value;
            }
        };
    }

    public Button getLowerRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.LOWER_RIGHT.value;
            }
        };
    }

    public Button getLowerLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.LOWER_LEFT.value;
            }
        };
    }

    public Button getUpperLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == DPAD.UPPER_LEFT.value;
            }
        };
    }
}

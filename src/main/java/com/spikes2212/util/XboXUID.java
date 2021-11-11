package com.spikes2212.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

/**
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
        return xbox.getTriggerAxis(Hand.kRight);
    }

    public double getLTAxis() {
        return xbox.getTriggerAxis(Hand.kLeft);
    }

    public Trigger getRTButton() {
        return new Trigger() {

            @Override
            public boolean get() {
                return xbox.getTriggerAxis(Hand.kRight) == 1;
            }
        };
    }

    public Trigger getLTButton() {
        return new Trigger() {

            @Override
            public boolean get() {
                return xbox.getTriggerAxis(Hand.kLeft) == 1;
            }
        };
    }

    public JoystickButton getRBButton() {
        return new JoystickButton(this, XboxController.Button.kBumperRight.value);
    }

    public JoystickButton getLBButton() {
        return new JoystickButton(this, XboxController.Button.kBumperLeft.value);
    }

    public JoystickButton getRightStickButton() {
        return new JoystickButton(this, XboxController.Button.kStickRight.value);
    }

    public JoystickButton getLeftStickButton() {
        return new JoystickButton(this, XboxController.Button.kStickLeft.value);
    }

    public double getRightX() {
        return xbox.getX(Hand.kRight);
    }

    public double getRightY() {
        return xbox.getY(Hand.kRight);
    }

    public double getLeftX() {
        return xbox.getX(Hand.kLeft);
    }

    public double getLeftY() {
        return xbox.getY(Hand.kLeft);
    }

    public Button getUpButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 0;
            }
        };
    }

    public Button getDownButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 180;
            }
        };
    }


    public Button getLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 270;
            }
        };
    }


    public Button getRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 90;
            }
        };
    }

    public Button getUpperRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 45;
            }
        };
    }

    public Button getLowerRightButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 135;
            }
        };
    }

    public Button getLowerLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 225;
            }
        };
    }

    public Button getUpperLeftButton() {
        return new Button() {
            @Override
            public boolean get() {
                return getPOV() == 315;
            }
        };
    }
}

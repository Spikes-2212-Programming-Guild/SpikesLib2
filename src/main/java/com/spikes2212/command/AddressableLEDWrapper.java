package com.spikes2212.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;

/**
 * Class that represents a LED strip.
 *
 * @author Camellia Lami
 * @see DashboardedSubsystem
 */
public class AddressableLEDWrapper {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public AddressableLEDWrapper(int LEDPort, int numberOfLEDs) {
        led = new AddressableLED(LEDPort);
        ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
    }

    /**
     * Sets the LED strip to a specific color.
     *
     * @param red   The desired red value.
     * @param green The desired green value.
     * @param blue  The desired blue value.
     */
    public void setStripColor(int red, int green, int blue) {
        setColorInRange(red, green, blue, 0, ledBuffer.getLength());
    }

    /**
     * @param color The desired {@link Color}.
     */
    public void setStripColor(Color color) {
        setStripColor(color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * Turns off the strip.
     */
    public void turnOff() {
        setStripColor(0, 0, 0);
    }

    /**
     * Sets a certain range of LEDs to a specific color.
     *
     * @param red   The desired red value.
     * @param green The desired green value.
     * @param blue  The desired blue value.
     * @param start The first LED in the range.
     * @param end   The final LED in the range.
     */
    public void setColorInRange(int red, int green, int blue, int start, int end) {
        for (int i = start; i < end; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    /**
     * @param color The desired {@link Color}.
     * @param start The first LED in the range.
     * @param end   The final LED in the range.
     */
    public void setColorInRange(Color color, int start, int end) {
        setColorInRange(color.getRed(), color.getGreen(), color.getBlue(), start, end);
    }
}

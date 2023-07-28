package com.spikes2212.command;

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

    /**
     * {@link AddressableLED} - The LED strip itself.
     * {@link AddressableLEDBuffer} - The LED strip's data.
     */
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public AddressableLEDWrapper(int ledPort, int numberOfLEDs) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
    }

    /**
     * Sets the LED strip to a specific color.
     *
     * @param red   the red value
     * @param green the green value
     * @param blue  the blue value
     */
    public void setStripColor(int red, int green, int blue) {
        setColorInRange(red, green, blue, 0, ledBuffer.getLength());
    }

    /**
     * Sets the LED strip to a specific color.
     *
     * @param color the desired {@link Color}
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
     * @param red   the red value
     * @param green the green value
     * @param blue  the blue value
     * @param start the first LED in the range
     * @param end   the final LED in the range
     */
    public void setColorInRange(int red, int green, int blue, int start, int end) {
        for (int i = start; i < end; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    /**
     * Sets a certain range of LEDs to a specific color.
     *
     * @param color the desired {@link Color}
     * @param start the first LED in the range
     * @param end   the final LED in the range
     */
    public void setColorInRange(Color color, int start, int end) {
        setColorInRange(color.getRed(), color.getGreen(), color.getBlue(), start, end);
    }

    public void setData() {
        led.setData(ledBuffer);
    }

    /**
     * Activates the LED strip.
     */
    public void startLed() {
        led.start();
    }

    /**
     * Deactivates the LED strip.
     */
    public void stopLed() {
        led.stop();
    }
}

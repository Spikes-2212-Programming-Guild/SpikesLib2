package com.spikes2212.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.Color;

/**
 * A class that handles the communication between a LED strip and the code.
 * This class still requires further testing and an update will be made as soon as testing is finished.
 *
 * @author Camellia Lami
 */
public class AddressableLEDWrapper {

    /**
     * The controlled LED strip.
     */
    private final AddressableLED led;

    /**
     * The controlled LED strip's data.
     */
    private final AddressableLEDBuffer ledBuffer;

    public AddressableLEDWrapper(int ledPort, int numberOfLEDs) {
        this.led = new AddressableLED(ledPort);
        this.ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
    }

    /**
     * Sets the LED strip to a specific color.
     *
     * @param red   the red value from 0 to 255
     * @param green the green value from 0 to 255
     * @param blue  the blue value from 0 to 255
     */
    public void setStripColor(int red, int green, int blue) {
        setColorInRange(0, ledBuffer.getLength() - 1, red, green, blue);
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
     * @param start the first LED in the range
     * @param end   the final LED in the range
     * @param red   the red value from 0 to 255
     * @param green the green value from 0 to 255
     * @param blue  the blue value from 0 to 255
     */
    public void setColorInRange(int start, int end, int red, int green, int blue) {
        for (int i = start; i <= end; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    /**
     * Sets a certain range of LEDs to a specific color.
     *
     * @param start the first LED in the range
     * @param end   the final LED in the range
     * @param color the desired {@link Color}
     */
    public void setColorInRange(int start, int end, Color color) {
        setColorInRange(start, end, color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * Sets a specific LED to a specific color.
     *
     * @param index the index of the LED
     * @param red   the red value from 0 to 255
     * @param green the green value from 0 to 255
     * @param blue  the blue value from 0 to 255
     */
    public void setColorAt(int index, int red, int green, int blue) {
        ledBuffer.setRGB(index, red, green, blue);
    }

    /**
     * Sets a specific LED to a specific color.
     *
     * @param index the index of the LED
     * @param color the desired {@link Color}
     */
    public void setColorAt(int index, Color color) {
        ledBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * Takes the buffer's data and applies it to the LED strip.
     * This method should be called periodically.
     */
    public void update() {
        led.setData(ledBuffer);
    }

    /**
     * Activates the LED strip.
     */
    public void enableStrip() {
        led.start();
    }

    /**
     * Deactivates the LED strip.
     */
    public void disableStrip() {
        led.stop();
    }
}

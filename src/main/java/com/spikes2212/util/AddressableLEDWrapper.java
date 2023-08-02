package com.spikes2212.util;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;

/**
 * A class that handles the communication between an LED strip and the code.
 *
 * @author Camellia Lami
 * @see DashboardedSubsystem
 */
public class AddressableLEDWrapper {

    /**
     * {@link AddressableLED} - The LED strip itself.
     */
    private final AddressableLED led;
    /**
     * {@link AddressableLEDBuffer} - The LED strip's data.
     */
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
     * @param start the first LED in the range
     * @param end   the final LED in the range
     * @param red   the red value
     * @param green the green value
     * @param blue  the blue value
     */
    public void setColorInRange(int start, int end, int red, int green, int blue) {
        for (int i = start; i < end; i++) {
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
        setColorInRange(color.getRed(), color.getGreen(), color.getBlue(), start, end);
    }

    /**
     * Sets a specific LED to a specific color.
     *
     * @param index the index of the LED
     * @param red   the red value
     * @param green the green value
     * @param blue  the blue value
     */
    public void setColorAt(int index, int red, int green, int blue) {
        ledBuffer.setRGB(index, red, green, blue);
    }

    /**
     * Sets a specific LED to a specific color.
     *
     * @param color the desired {@link Color}
     * @param index the index of the LED
     */
    public void setColorAt(int index, Color color) {
        ledBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * Takes the buffer's data and applies it to the LED strip.
     */
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

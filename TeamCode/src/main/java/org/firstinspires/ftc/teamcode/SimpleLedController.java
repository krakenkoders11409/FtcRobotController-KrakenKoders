package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Simple LED Controller for FTC Robot
 * Controls WS2812B LEDs via I2C communication with a D1 Mini
 *
 * Configuration:
 * 1. In Robot Configuration, add a "REV Color/Range Sensor" on I2C Bus 0
 * 2. Name it "ledController"
 * 3. Save and activate configuration
 *
 * D1 Mini Wiring:
 * - D2 (GPIO4) -> I2C Bus 0 SDA (Pin 1)
 * - D1 (GPIO5) -> I2C Bus 0 SCL (Pin 2)
 * - GND -> I2C Bus 0 GND (Pin 3)
 * - 5V -> External 5V supply
 */
public class SimpleLedController {

    private I2cDeviceSynch deviceSynch;
    private static final I2cAddr LED_ADDRESS = I2cAddr.create7bit(0x1C);

    // Command bytes for D1 Mini
    private static final byte CMD_RED = 0x01;
    private static final byte CMD_BLUE = 0x02;
    private static final byte CMD_OFF = 0x03;
    private static final byte CMD_RAINBOW = 0x04;
    private static final byte CMD_BREATHE = 0x05;
    private static final byte CMD_CHASE = 0x06;

    private boolean isRed = true;
    private boolean isConnected = false;

    /**
     * Constructor
     * @param hardwareMap The hardware map from OpMode
     * @param deviceName Name of the device in configuration
     */
    public SimpleLedController(HardwareMap hardwareMap, String deviceName) {
        try {
            // Try to get as I2cDeviceSynch directly
            deviceSynch = hardwareMap.get(I2cDeviceSynch.class, deviceName);
            if (deviceSynch != null) {
                deviceSynch.setI2cAddress(LED_ADDRESS);
                deviceSynch.engage();
                isConnected = testConnection();
                if (isConnected) {
                    setRed(); // Default to red
                }
            }
        } catch (Exception e) {
            // If that fails, try getting as ColorSensor and extract I2C
            try {
                ColorSensor sensor = hardwareMap.get(ColorSensor.class, deviceName);
                // Unfortunately we can't extract I2cDeviceSynch from ColorSensor
                // So we'll try one more approach
                deviceSynch = hardwareMap.tryGet(I2cDeviceSynch.class, deviceName);
                if (deviceSynch != null) {
                    deviceSynch.setI2cAddress(LED_ADDRESS);
                    deviceSynch.engage();
                    isConnected = testConnection();
                }
            } catch (Exception e2) {
                isConnected = false;
            }
        }
    }

    /**
     * Set LEDs to red alliance color
     */
    public void setRed() {
        isRed = true;
        sendCommand(CMD_RED);
    }

    /**
     * Set LEDs to blue alliance color
     */
    public void setBlue() {
        isRed = false;
        sendCommand(CMD_BLUE);
    }

    /**
     * Turn off all LEDs
     */
    public void turnOff() {
        sendCommand(CMD_OFF);
    }

    /**
     * Set rainbow effect
     */
    public void setRainbow() {
        sendCommand(CMD_RAINBOW);
    }

    /**
     * Set breathing effect
     */
    public void setBreathe() {
        sendCommand(CMD_BREATHE);
    }

    /**
     * Set chase effect
     */
    public void setChase() {
        sendCommand(CMD_CHASE);
    }

    /**
     * Set alliance color
     * @param red True for red, false for blue
     */
    public void setAlliance(boolean red) {
        if (red) {
            setRed();
        } else {
            setBlue();
        }
    }

    /**
     * Send command to LED controller
     */
    private void sendCommand(byte command) {
        if (deviceSynch != null && isConnected) {
            try {
                deviceSynch.write8(0x00, command);
            } catch (Exception e) {
                // Ignore errors to prevent crashes
            }
        }
    }

    /**
     * Test I2C connection
     */
    private boolean testConnection() {
        if (deviceSynch == null) return false;
        try {
            deviceSynch.read8(0x00);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Check if connected
     */
    public boolean isConnected() {
        return isConnected;
    }

    /**
     * Get current alliance
     */
    public boolean isRedAlliance() {
        return isRed;
    }

    /**
     * Get status string for telemetry
     */
    public String getStatus() {
        if (!isConnected) {
            return "Not Connected";
        }
        return isRed ? "Red Alliance" : "Blue Alliance";
    }
}

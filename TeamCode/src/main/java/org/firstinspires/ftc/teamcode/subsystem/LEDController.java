package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LEDController {
    // Front LED is a servo-controlled LED module (goBilda RGB Indicator Light)
    private final Servo frontLED;
    // Left and Right LED strips are connected to Rev Blinkin LED modules.
    private final RevBlinkinLedDriver leftLED;
    private final RevBlinkinLedDriver rightLED;
    public enum LEDColor {
        RED(0.285),
        GREEN(0.5),
        BLUE(0.611),
        YELLOW(0.388),
        PURPLE(0.722),
        OFF(0.0);

        private final double position;

        LEDColor(double position) {
            this.position = position;
        }
    }

    // Timers for each LED.
    private final ElapsedTime frontTimer = new ElapsedTime();
    private final ElapsedTime leftTimer = new ElapsedTime();
    private final ElapsedTime rightTimer = new ElapsedTime();

    // Duration to keep each LED on. A value of 0 means “stay on indefinitely.”
    private double frontDuration = 0;
    private double leftDuration = 0;
    private double rightDuration = 0;

    public LEDController(Servo frontLED, RevBlinkinLedDriver leftLED, RevBlinkinLedDriver rightLED) {
        this.frontLED = frontLED;
        this.leftLED = leftLED;
        this.rightLED = rightLED;
        // Initialize all LEDs to OFF.
        setFrontLED(LEDColor.OFF, 0);
        setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK, 0);
        setRightLED(RevBlinkinLedDriver.BlinkinPattern.BLACK, 0);
    }

    // Set the front LED pattern and duration.
    // If durationSeconds is 0, the LED stays on indefinitely.
    public void setFrontLED(LEDColor color, double durationSeconds) {
        // For the front LED, we use a servo command.
        frontLED.setPosition(color.position);
        frontDuration = (durationSeconds > 0) ? durationSeconds : 0;
        frontTimer.reset();
    }
    public void setFrontLED(LEDColor color) {
        // If durationSeconds is not given, call with defaultTimeout.
        setFrontLED(color, Constants.LED_DEFAULT_TIMEOUT);
    }

    // Set the left LED pattern and duration.
    // If durationSeconds is 0, the LED stays on indefinitely.
    public void setLeftLED(RevBlinkinLedDriver.BlinkinPattern pattern, double durationSeconds) {
        leftLED.setPattern(pattern);
        leftDuration = (durationSeconds > 0) ? durationSeconds : 0;
        leftTimer.reset();
    }
    public void setLeftLED(RevBlinkinLedDriver.BlinkinPattern pattern) {
        // If durationSeconds is not given, call with defaultTimeout.
        setLeftLED(pattern, Constants.LED_DEFAULT_TIMEOUT);
    }

    // Set the right LED pattern and duration.
    // If durationSeconds is 0, the LED stays on indefinitely.
    public void setRightLED(RevBlinkinLedDriver.BlinkinPattern pattern, double durationSeconds) {
        rightLED.setPattern(pattern);
        rightDuration = (durationSeconds > 0) ? durationSeconds : 0;
        rightTimer.reset();
    }
    public void setRightLED(RevBlinkinLedDriver.BlinkinPattern pattern) {
        // If durationSeconds is not given, call with defaultTimeout.
        setRightLED(pattern, Constants.LED_DEFAULT_TIMEOUT);
    }

    // Call this method each loop (e.g., in TeleOp or Auton) to check timers and automatically turn off LEDs if needed.
    public void update() {
        if (frontDuration > 0 && frontTimer.seconds() >= frontDuration) {
            setFrontLED(LEDColor.OFF, 0);
        }
        if (leftDuration > 0 && leftTimer.seconds() >= leftDuration) {
            setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK, 0);
        }
        if (rightDuration > 0 && rightTimer.seconds() >= rightDuration) {
            setRightLED(RevBlinkinLedDriver.BlinkinPattern.BLACK, 0);
        }
    }

    // Turn all LEDs off.
    public void allOff() {
        setFrontLED(LEDColor.OFF, 0);
        setLeftLED(RevBlinkinLedDriver.BlinkinPattern.BLACK, 0);
        setRightLED(RevBlinkinLedDriver.BlinkinPattern.BLACK, 0);
    }
}


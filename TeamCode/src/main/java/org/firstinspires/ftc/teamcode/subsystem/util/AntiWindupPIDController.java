package org.firstinspires.ftc.teamcode.subsystem.util;


// Custom PID controller with anti-windup and output saturation control.
public class AntiWindupPIDController {
    private double kp, ki, kd;
    private double maxIntegral;
    private double maxOutput;

    private double integral;
    private double lastError;

    public AntiWindupPIDController(double kp, double ki, double kd, double maxIntegral, double maxOutput) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.maxIntegral = maxIntegral;
        this.maxOutput = maxOutput;
        integral = 0;
        lastError = 0;
    }

    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double current, double target) {
        double error = target - current;
        integral += error;  // You could multiply by the time step if you want a time-based integration.
        // Clamp the integral term.
        if (integral > maxIntegral) {
            integral = maxIntegral;
        } else if (integral < -maxIntegral) {
            integral = -maxIntegral;
        }
        double derivative = error - lastError;
        lastError = error;

        double output = kp * error + ki * integral + kd * derivative;
        // Clamp the output power to within -maxOutput and +maxOutput.
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < -maxOutput) {
            output = -maxOutput;
        }
        return output;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}

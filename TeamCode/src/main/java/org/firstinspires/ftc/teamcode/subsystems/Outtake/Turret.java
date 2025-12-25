package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.RTPAxon;

public class Turret {

    private final RTPAxon turretServo;
    private boolean enabled = true; // allow manual override

    // Constructor
    public Turret(CRServo servo, AnalogInput encoder) {
        turretServo = new RTPAxon(servo, encoder);
        // Optional PID tuning
        turretServo.setPidCoeffs(0.015, 0.0005, 0.0025);
        turretServo.setMaxPower(0.25);
    }

    // Enable or disable automatic turret control
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    // Set target angle (absolute)
    public void setTargetAngle(double degrees) {
        if (enabled) {
            turretServo.setTargetRotation(degrees);
        }
    }

    // Increment target angle (relative)
    public void changeTargetAngle(double deltaDegrees) {
        if (enabled) {
            turretServo.changeTargetRotation(deltaDegrees);
        }
    }

    // Manual power control (bypass PID)
    public void manualPower(double power) {
        turretServo.setRtp(false);
        turretServo.setPower(power);
    }

    // Update loop: must call in OpMode loop
    public void update() {
        turretServo.update();
    }

    // Read current angle (degrees)
    public double getAngle() {
        return turretServo.getTotalRotation();
    }

    // Check if turret is at target
    public boolean isAtTarget() {
        return turretServo.isAtTarget();
    }

    // Reset turret encoder and PID
    public void reset() {
        turretServo.forceResetTotalRotation();
        turretServo.setRtp(true);
    }

    // Optional: telemetry string
    public String telemetryString() {
        return turretServo.log();
    }
}

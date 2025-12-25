package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Turret {

    private final RTPAxon leftServo;
    private final RTPAxon rightServo;
    private boolean enabled = true; // allow manual override

    // Constructor
    public Turret(HardwareMap hwMap) {
        CRServo left = hwMap.get(CRServo.class, "turretLeft");
        CRServo right = hwMap.get(CRServo.class, "turretRight");
        AnalogInput encoder = hwMap.get(AnalogInput.class, "turretEncoder");

        leftServo = new RTPAxon(left, encoder);
        rightServo = new RTPAxon(right, encoder);

        // Optional PID tuning
        leftServo.setPidCoeffs(0.015, 0.0005, 0.0025);
        rightServo.setPidCoeffs(0.015, 0.0005, 0.0025);

        leftServo.setMaxPower(0.8);
        rightServo.setMaxPower(0.8);
    }

    // ---------------- Control ----------------
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setTargetAngle(double degrees) {
        if (enabled) {
            leftServo.setTargetRotation(degrees);
            rightServo.setTargetRotation(-degrees); // opposite direction
        }
    }

    public void changeTargetAngle(double deltaDegrees) {
        if (enabled) {
            leftServo.changeTargetRotation(deltaDegrees);
            rightServo.changeTargetRotation(-deltaDegrees); // opposite
        }
    }

    public void manualPower(double power) {
        leftServo.setRtp(false);
        rightServo.setRtp(false);

        leftServo.setPower(power);
        rightServo.setPower(-power); // opposite
    }

    public void update() {
        leftServo.update();
        rightServo.update();
    }

    public double getAngle() {
        return leftServo.getTotalRotation(); // both share the same encoder
    }

    public boolean isAtTarget() {
        return leftServo.isAtTarget() && rightServo.isAtTarget();
    }

    public void reset() {
        leftServo.forceResetTotalRotation();
        rightServo.forceResetTotalRotation();
        leftServo.setRtp(true);
        rightServo.setRtp(true);
    }

    public String telemetryString() {
        return "L: " + leftServo.log() + " | R: " + rightServo.log();
    }
}

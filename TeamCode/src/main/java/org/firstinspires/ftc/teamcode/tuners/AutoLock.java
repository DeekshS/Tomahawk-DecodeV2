package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp
@Config
public class AutoLock extends LinearOpMode {
    public static double minPower = 0.15;
    public static double maxPower = 0.6;
    public static double p = 0.008;
    public static double i = 0.0;
    public static double d = 0.0003;
    public static double f = 0.0;
    public static double targetAngle = 180;
    public static double tolerance = 5;
    public static double maxIntegral = 0.3;
    public static int servo = 0;
    public static boolean tuning = false;


    private static final double minBound = 0;
    private static final double maxBound = 360;

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double calculateError(double target, double current) {
        double error = target - current;

        // Find shortest path
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        return error;
    }

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        AnalogInput servoEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));

        ElapsedTime timer = new ElapsedTime();
        double lastError = 0;
        double lastTime = 0;
        double integralSum = 0;

        waitForStart();
        if (isStopRequested()) return;

        // Read initial position
        double rawEncoder = (servoEncoder.getVoltage() / 3.3 * 360);
        double initialAngle = normalizeAngle(rawEncoder);

        telemetry.addData("Initial Angle", initialAngle);
        telemetry.addData("0° = Back", "");
        telemetry.addData("90° = Right", "");
        telemetry.addData("180° = Front", "");
        telemetry.addData("270° = Left", "");
        telemetry.addData("Valid Range", "0° to 360°");
        telemetry.update();

        sleep(2000);

        timer.reset();

        while (opModeIsActive()) {
            drive.localizer.update();

            double currentTime = timer.seconds();
            double dt = currentTime - lastTime;

            // Read raw encoder (0-360)
            rawEncoder = (servoEncoder.getVoltage() / 3.3 * 360);
            double currentAngle = normalizeAngle(rawEncoder);

            // Normalize target to 0-360
            double normalizedTarget = normalizeAngle(targetAngle);

            // Calculate error using shortest path
            double error = calculateError(normalizedTarget, currentAngle);

            // Calculate integral
            if (dt > 0 && Math.abs(error) > tolerance) {
                integralSum += error * dt;
                integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
            } else if (Math.abs(error) <= tolerance) {
                integralSum = 0;
            }

            // Calculate derivative
            double derivative = 0;
            if (dt > 0) {
                derivative = (error - lastError) / dt;
            }

            // Calculate power with PIDF control
            double power = 0;

            if (Math.abs(error) > tolerance) {
                double proportional = error * p;
                double integral = integralSum * i;
                double derivativeTerm = derivative * d;
                double feedforward = Math.signum(error) * f;

                power = proportional + integral + derivativeTerm + feedforward;

                // Clamp to min/max power while preserving sign
                if (power > 0) {
                    power = Math.max(minPower, Math.min(maxPower, power));
                } else {
                    power = Math.max(-maxPower, Math.min(-minPower, power));
                }

                // Hard stop at bounds (0° and 360° are the same, so check both)
                // Stop if within 10° of 0°/360° AND moving toward the boundary
                if (((currentAngle < 10 || currentAngle > 350) &&
                        ((currentAngle < 10 && power < 0) || (currentAngle > 350 && power > 0)))) {
                    power = 0;
                }
            } else {
                integralSum = 0;
            }

            if (tuning) {
                if (servo == 1) {
                    left.setPower(power);
                    right.setPower(0);
                }
                else {
                    left.setPower(0);
                    right.setPower(power);
                }
            }
            else {
                left.setPower(power);
                right.setPower(power);
            }

            lastError = error;
            lastTime = currentTime;

            telemetry.addData("Raw Encoder", rawEncoder);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", normalizedTarget);
            telemetry.addData("Error", error);
            telemetry.addData("P Term", error * p);
            telemetry.addData("I Term", integralSum * i);
            telemetry.addData("D Term", derivative * d);
            telemetry.addData("F Term", Math.signum(error) * f);
            telemetry.addData("Integral Sum", integralSum);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.IMUGyro;
public class DriveTrain {


    private final IMUGyro imu;


    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx bl;
    private final DcMotorEx br;
    public double rx;
    private double targetHeading;
    private double currentHeading;
    public State s = State.ROBOTCENTRIC;
    private static boolean isAutoTurning = false;




    public DriveTrain(LinearOpMode mode) {


        fl = mode.hardwareMap.get(DcMotorEx.class, "fl");
        fr = mode.hardwareMap.get(DcMotorEx.class, "fr");
        bl = mode.hardwareMap.get(DcMotorEx.class, "bl");
        br = mode.hardwareMap.get(DcMotorEx.class, "br");






        // Correct IMU instantiation
        imu = new IMUGyro(mode.hardwareMap);




        DcMotorEx[] motors = new DcMotorEx[]{fl, fr, bl, br};
        DcMotorEx[] reversedMotors = new DcMotorEx[]{fl, bl};


        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        for (DcMotorEx motor : reversedMotors) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }






    public void update() {
        if (s.equals(State.ROBOTCENTRIC)) {
            double x = GamepadMappings.strafe;
            double y = -GamepadMappings.drive;
//            double rx;


            if (isAutoTurning) {
                rx = calculateRotationPower();
                if (Math.abs(GamepadMappings.turn) > 0.1) {
                    isAutoTurning = false;
                }
            } else {
                rx = GamepadMappings.turn;
            }


            double flPower = y + x + rx;
            double frPower = y - x - rx;
            double blPower = y - x + rx;
            double brPower = y + x - rx;


            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }
    }


    public void flipState() {
        s = (s.equals(State.ROBOTCENTRIC) ? State.FIELDCENTRIC : State.ROBOTCENTRIC);
    }


    public void addTelemetry() {
        // Add any telemetry here if needed
    }


    public void resetIMU() {
        imu.resetIMU();
    }


    public enum State {
        ROBOTCENTRIC, FIELDCENTRIC
    }
    public double calculateRotationPower() {
        double error = angleWrap(targetHeading - currentHeading);


        if (Math.abs(error) < DriveConstants.tolerance) {
            return 0;
        }


        double power = error * DriveConstants.kP;


        return Math.max(-0.7, Math.min(0.7, power));
    }


    public double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
    public double calculateHeading(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;


        double deltaX = PoseStorage.goalX - robotX;
        double deltaY = PoseStorage.goalY - robotY;
        return Math.toDegrees(Math.atan2(deltaY, deltaX)) - Math.toDegrees(pose.heading.toDouble());
    }


    public void setTargetHeading(double degrees) {
        targetHeading = degrees;
    }
    public void setIsAutoTurning(boolean value) {
        isAutoTurning = value;
    }
    public void setCurrentHeading(double heading) {
        currentHeading = heading;
    }
    public double getCurrentHeading() {
        return currentHeading;
    }
}


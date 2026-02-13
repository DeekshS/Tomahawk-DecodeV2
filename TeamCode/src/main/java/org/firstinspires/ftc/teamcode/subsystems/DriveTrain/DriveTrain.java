package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;




import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;




import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.IMUGyro;
public class DriveTrain {




    private final IMUGyro imu;




    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx bl;
    private final DcMotorEx br;
    private final Limelight3A limelight;
    private GamepadMappings controls;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    public double rx;
    private double targetHeading;
    private double currentHeading;
    private double lastTime;
    private double lastError;
    private double goalX = 0;
    public double result;
    public double error;
    public String text;
    public double testingResult;
    public State s = State.ROBOTCENTRIC;
    private static boolean isAutoTurning = false;








    public DriveTrain(LinearOpMode mode) {




        fl = mode.hardwareMap.get(DcMotorEx.class, "fl");
        fr = mode.hardwareMap.get(DcMotorEx.class, "fr");
        bl = mode.hardwareMap.get(DcMotorEx.class, "bl");
        br = mode.hardwareMap.get(DcMotorEx.class, "br");


        limelight = mode.hardwareMap.get(Limelight3A.class, "limelight");
        gamepad1 = new Gamepad();
        gamepad2 = new Gamepad();
        controls = new GamepadMappings(gamepad1, gamepad2);

        limelight.setPollRateHz(10);
        limelight.pipelineSwitch((PoseStorage.side.equals(PoseStorage.SIDE.BLUE) ? 1 : 0));
//        limelight.pipelineSwitch(1);
        limelight.start();












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












    public void update(double runTime) {
        if (s.equals(State.ROBOTCENTRIC)) {
            double x = GamepadMappings.strafe;
            double y = -GamepadMappings.drive;
            double rx = GamepadMappings.turn;
            testingResult = limelight.getLatestResult().getTx();
            result = (limelight.getLatestResult().isValid() && limelight.getLatestResult() != null) ? limelight.getLatestResult().getTx() : Double.NaN;




            if (true) {
                error = goalX - result;


                if (Math.abs(error) < DriveConstants.tolerance) {
                    rx = 0;
                    text = "pressed, inside of tolerance";

                } else {
                    text = "pressed, below tolerance";
                    double p = error * DriveConstants.kP;


                    double curTime = runTime;
                    double dT = curTime - lastTime;
                    double d = ((error - lastError) / dT) * DriveConstants.kD;


                    rx = -(Range.clip(p + d, -0.4, 0.4));


                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = runTime;
                lastError = 0;
                rx = GamepadMappings.turn;
                text = "NOT pressed";
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




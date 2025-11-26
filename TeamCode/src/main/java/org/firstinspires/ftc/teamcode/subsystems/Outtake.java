package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MiniPID;


@Config
public class Outtake implements Subsystem<Outtake.State> {

    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public State currentState = State.NONE;

    public static int FAR_VELOCITY = 2225;
    public static int CLOSE_VELOCITY = 1750;

    public boolean isFar = false;

    public double SETPOINT = FAR_VELOCITY;
    public double pidOutput;
    double error;

//    public static double P = 8.25, I = 0, D = 0, V = 0.47;
    public static double P = 0.0025, I = 0, D = 0, V = 0.00035;


    MiniPID velocityController;

    LinearOpMode mode;
    IMUGyro imu;

    Telemetry telemetry;

    LimelightCamera camera;


    public Outtake(LinearOpMode mode, LimelightCamera camera) {

        this.mode = mode;
        this.camera = camera;

        motor1 = mode.hardwareMap.get(DcMotorEx.class, "outtake1");
        motor2 = mode.hardwareMap.get(DcMotorEx.class, "outtake2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        velocityController = new MiniPID(P, I, D, V);

        telemetry = mode.telemetry;
        imu = new IMUGyro(mode);

        setState(currentState);

    }

//    public void autoAlign() {
//        // First, tell Limelight which way your robot is facing
//        double robotYaw = imu.getAngularOrientation().firstAngle;
//        limelight.updateRobotOrientation(robotYaw);
//        if (result != null && result.isValid()) {
//            Pose3D botPose = result.getBotpose_MT2();
//            if (botpose_mt2 != null) {
//                double x = botpose_mt2.getPosition().x;
//                double y = botpose_mt2.getPosition().y;
//                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
//            }
//        }
//    }
    public void setState(State state) {
        switch (state) {
            case NONE:
                motor1.setPower(0);
                motor2.setPower(0);
                break;

            case MOVING:
                updateLoop();
                break;

            case REVERSE:
//                setVelocity(-1000);
                motor1.setPower(0);
                motor2.setPower(0);

                break;
        }

        this.currentState = state;
    }

    public State getState() {
        return currentState;
    }

    public void addTelemetry() {

        mode.telemetry.addData("Outtake State", currentState);

        mode.telemetry.addData("Error", error);
        mode.telemetry.addData("Velocity", Math.abs(getVelocity()));
        mode.telemetry.addData("Set point", SETPOINT);
        mode.telemetry.addData("PID Output", pidOutput);
        mode.telemetry.addData("Is Far", isFar);

    }

    public double getVelocity() {
        return motor1.getVelocity();
    }

    public void setVelocity(double velocity) {
        motor1.setVelocity(velocity);
        motor2.setVelocity(-velocity);
    }

    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(-power);
    }

    public void updateLoop() {

//        SETPOINT = isFar ? FAR_VELOCITY : CLOSE_VELOCITY;
        SETPOINT = Math.pow(Math.pow(camera.limelight.getLatestResult().getBotpose().getPosition().x, 2) + Math.pow(camera.limelight.getLatestResult().getBotpose().getPosition().y, 2), 0.5);
        velocityController.setSetpoint(SETPOINT);
        error = SETPOINT - Math.abs(getVelocity());

        pidOutput = velocityController.getOutput(Math.abs(getVelocity()));

        setPower(pidOutput);

        velocityController.setP(P);
        velocityController.setI(I);
        velocityController.setD(D);
        velocityController.setF(V);

    }


    public void toggleIsFar() {
        isFar = !isFar;
    }

    public Action shoot_velocity(int vel) {

        return new Action() {

            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    init = true;
                }


                SETPOINT = vel;
                velocityController.setSetpoint(SETPOINT);
                error = SETPOINT - Math.abs(getVelocity());

                pidOutput = velocityController.getOutput(Math.abs(getVelocity()));

                setPower(pidOutput);

//                setVelocity(1650);


                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));
                telemetryPacket.put("ERROR", error);
                telemetryPacket.put("SETPOINT", SETPOINT);


                return error >= 50;
            }

        };
    }


    public Action shoot_close() {

        return new Action() {

            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    init = true;
                }


                SETPOINT = 1770;
                velocityController.setSetpoint(SETPOINT);
                error = SETPOINT - Math.abs(getVelocity());

                pidOutput = velocityController.getOutput(Math.abs(getVelocity()));

                setPower(pidOutput);

//                setVelocity(1650);


                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));
                telemetryPacket.put("ERROR", error);
                telemetryPacket.put("SETPOINT", SETPOINT);


                return error >= 50;
            }

        };
    }

    public Action shoot_far() {

        return new Action() {

            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    init = true;
                }


                SETPOINT = 2500;
                velocityController.setSetpoint(SETPOINT);
                error = SETPOINT - Math.abs(getVelocity());

                pidOutput = velocityController.getOutput(Math.abs(getVelocity()));

                setPower(pidOutput);

//                setVelocity(1650);


                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));
                telemetryPacket.put("ERROR", error);
                telemetryPacket.put("SETPOINT", SETPOINT);


                return error >= 50;
            }

        };
    }

    public Action shoot_stop() {

        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                setState(State.NONE);

                return false;
            }

        };
    }


    public Action shoot_reverse(double time) {

        return new Action() {

            private boolean init = false;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    init = true;
                    timer.reset();
                    setState(State.REVERSE);
                }

                if (timer.seconds() < time) {
                    return true;
                }
//e
                else if (timer.seconds() >= time) {
                    setState(State.NONE);
                    telemetryPacket.put("Status", "stop");
                    timer.reset();
                    return false;
                }

                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));

                return true;
            }

        };
    }



    public enum State {
        MOVING, NONE, REVERSE
    }
}
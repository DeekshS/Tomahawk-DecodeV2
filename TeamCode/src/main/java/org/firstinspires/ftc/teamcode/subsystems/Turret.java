package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MiniPID;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class Turret implements Subsystem<Turret.State> {

    private double SETPOINT = 0;
    public DcMotorEx turretMotor;
    public double TICKS_PER_DEGREE = 4.57777;

    private State currentState = State.STOP;
    public int pos = 0;

    public int SMALL_RED = 0;
    public int SMALL_BLUE = 90;

    public int LARGE_RED = 0;
    public int LARGE_BLUE = 100;

    public int RED_HP = -122;
    public int BLUE_HP = 117;



    MiniPID controller;

    LinearOpMode mode;

//    LimelightCamera ll;
//    Limelight3A ll;
//    LLResult result;

    public static double P = 0.05, I = 0, D = 0.01, V = 0.04;
    public double error;
    public double pidOutput;
    public double tx;




    public Turret(LinearOpMode mode) {
        this.mode = mode;
        controller = new MiniPID(P, I, D, V);
        MecanumDrive drive = new MecanumDrive(mode.hardwareMap, new Pose2d(1, 1, 0));

        turretMotor = mode.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void addTelemetry() {
        mode.telemetry.addData("Turret State", getState());
        mode.telemetry.addData("Current Position", turretMotor.getCurrentPosition());
        mode.telemetry.addData("Set Position", pos);
    }


    public void updatePID(double tx) {
        this.tx = tx;
        // Get current position from encoder
        double currentPos = pos;


        // Calculate position error
        error = tx*TICKS_PER_DEGREE;

        // Get PID output
        controller.setSetpoint(currentPos - error);
        pidOutput = controller.getOutput(error);

        // Apply power to motor
        pos = (int) pidOutput;
        turretMotor.setTargetPosition((int) pidOutput);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);


        // Update PID gains (if tuning)
        controller.setP(P);
        controller.setI(I);
        controller.setD(D);
        controller.setF(V);


        mode.telemetry.addData("pidOutput", pidOutput);
    }

    public double degToTicks(int deg) {
        return deg * (537.7/360) * 3;
    }

    public void setState(State state) {
        switch (state) {
            case DETECTED:
                break;
            case STOP:
//                updatePositionPID(turretMotor.getCurrentPosition());
//                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                turretMotor.setPower(0);
                break;

            case ZERO:
                turretMotor.setTargetPosition(0);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(0.75);

            case LEFT:
//                speed = 1;
//                updatePID(speed);
//                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                turretMotor.setPower(1);
                break;

            case RIGHT:
//                speed = -1;
//                updatePID(speed);
//                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                turretMotor.setPower(1);
                break;

            case RED_HP:
                pos = (int) degToTicks(RED_HP);
                turretMotor.setTargetPosition(pos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(1);
                break;

            case BLUE_HP:
                pos = (int) degToTicks(BLUE_HP);
                turretMotor.setTargetPosition(pos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(1);
                break;

        }

        currentState = state;
    }

    public State getState() {
        return currentState;
    }



        public Action turretRed() {

            return new Action() {

                private boolean init = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                    if (!init) {
                        setState(State.RED_HP);
                        init = true;
                    }

                    return Math.abs(RED_HP - turretMotor.getCurrentPosition()) >= 50;
                }

            };
        }

    public Action turretBlue() {

        return new Action() {

            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    setState(State.BLUE_HP);
                    init = true;
                }

                return Math.abs(RED_HP - turretMotor.getCurrentPosition()) >= 50;
            }

        };
    }

    public enum State {
        STOP, LEFT, RIGHT, LARGE_RED, LARGE_BLUE, SMALL_BLUE, SMALL_RED, RED_HP, BLUE_HP, ZERO, DETECTED
    }
}






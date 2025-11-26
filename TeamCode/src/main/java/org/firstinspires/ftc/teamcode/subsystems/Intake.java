package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake implements Subsystem<Intake.State> {

    private DcMotorEx intakeMotor;

    private State currentState = State.REST;

    public Intake(LinearOpMode mode) {

        intakeMotor = mode.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setState(currentState);
    }

    public void setState(State state) {

        switch (state) {
            case INTAKE:
                intakeMotor.setPower(1);
                break;
            case REVERSE:
                intakeMotor.setPower(-1);
                break;
            case REST:
                intakeMotor.setPower(0);
                break;
        }

        currentState = state;

    }

    public State getState() {
        return currentState;
    }


    //------------------------ ACTIONS--------------------------------------------------

    public Action intake(double time) {

        return new Action() {

            ElapsedTime timer = new ElapsedTime();

            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    setState(State.INTAKE);
                    timer.reset();
                    init = true;
                }

                if (timer.seconds() < time) {
                    return true;
                }
//e
                else if (timer.seconds() >= time) {
                    setState(State.REST);
                    telemetryPacket.put("Status", "stop");
                    timer.reset();
                    return false;
                }

                telemetryPacket.put("Intake Status", "running");
                telemetryPacket.put("Timer Status", timer.seconds());

                return true;

            }

        };
    }

    public Action intakeReverse(double time) {

        return new Action() {

            ElapsedTime timer = new ElapsedTime();

            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    intakeMotor.setPower(-0.5);
                    timer.reset();
                    init = true;
                }

                if (timer.seconds() < time) {
                    return true;
                }
//
                else if (timer.seconds() >= time) {
                    setState(State.REST);
                    telemetryPacket.put("Status", "stop");
                    timer.reset();
                    return false;
                }

                telemetryPacket.put("Intake Status", "running");
                telemetryPacket.put("Timer Status", timer.seconds());

                return true;

            }

        };
    }

    public Action intakeWithout() {

        return new Action() {

            ElapsedTime timer = new ElapsedTime();

            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    setState(State.INTAKE);
                    init = true;
                }

                return true;

            }

        };
    }


    public Action stop() {

        return new Action() {

            ElapsedTime timer = null;

            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

               setState(State.REST);
                return false;

            }

        };
    }

    public static class intake_start implements Action {

        private ElapsedTime timer = new ElapsedTime();
        private boolean init = false;

        private double time;

        private Intake intake;

        public intake_start(double time, Intake intake) {
            this.time = time;
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!init) {
                intake.setState(State.INTAKE);
                init = true;
            }

            if (timer.seconds() - 3 < time) {
                return true;
            }
//
            else if (timer.seconds() - 3 >= time) {
                intake.setState(State.REST);
                telemetryPacket.put("Status", "stop");
                return false;
            }

            telemetryPacket.put("Intake Status", "running");
            telemetryPacket.put("Timer Status", timer.seconds());

            return true;
        }
    }




    public enum State {
        INTAKE,
        REVERSE,
        REST
    }

}

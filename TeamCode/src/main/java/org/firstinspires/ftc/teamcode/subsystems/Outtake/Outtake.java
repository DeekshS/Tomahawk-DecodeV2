package org.firstinspires.ftc.teamcode.subsystems.Outtake;


import static org.firstinspires.ftc.teamcode.PoseStorage.goalX;
import static org.firstinspires.ftc.teamcode.PoseStorage.goalY;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.math.BigDecimal;
import java.math.RoundingMode;


public class Outtake {
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    public Servo hood;
    public double SETPOINT;
    MultipleTelemetry telemetry;


    // PIDF coefficients for flywheel velocity control
    public static double P = 430, I = 0, D = 1, F = 14.8;


    // Hood control constants (tune these)
    public static double K = 0.05; // saturation rate for hood function
    public static double T = 120;  // distance offset for hood function
    public static double MIN_HOOD = 0.2;  // servo min (0-1)
    public static double MAX_HOOD = 0.8;  // servo max (0-1)
    public static double hoodOffset = 0.25;
    public double autoHoodPos;
    public double autoVelo;
    public double currentHoodPos;
    public double distance;
    public double currentVelocity;


    public Outtake(LinearOpMode mode) {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), mode.telemetry);
        motor1 = mode.hardwareMap.get(DcMotorEx.class, "outtake1");
//        motor1.setDirection(DcMotorEx.Direction.REVERSE);
        motor2 = mode.hardwareMap.get(DcMotorEx.class, "outtake2");
//        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        hood = mode.hardwareMap.get(Servo.class, "hood");

        hood.setDirection(Servo.Direction.REVERSE);


        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor1.setDirection(DcMotorEx.Direction.REVERSE);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));




        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }

    public void bangController(double velocity) {
        double error = velocity - getVelocity();
        if (error > 50) {
            motor1.setPower(10);
            motor2.setPower(10);
        }
        else {
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }


    public void setVelocity(double velocity) {
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }
    public void shootVelocity(int velocity) {
//        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
//        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor1.setVelocityPIDFCoefficients(P, I, D, F);
        motor2.setVelocityPIDFCoefficients(P, I, D, F);

        setVelocity(velocity);
//        if (Math.abs(Math.abs(velocity) - getVelocity()) < 100) setVelocity(velocity);
//        else bangController(velocity);

    }
    public void shootStop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }


    public double getVelocity() {
        return Math.abs(motor1.getVelocity() + motor2.getVelocity()) / 2.0;
    }


    /**
     * Automatic hood calculation based on distance to goal
     * Returns a servo position (0-1)
     */
    public double hoodCalc(double distance) {
//        double rawHood = -0.00000302855 * Math.pow(distance, 3)
//                + 0.000980401 * Math.pow(distance, 2)
//                - 0.10017 * distance
//                + 3.92985;
        double rawHood = 0.0000767983 * Math.pow(distance, 2) - 0.0144138 * distance + 1.34219;
        BigDecimal bd = new BigDecimal(Double.toString(rawHood));
        bd = bd.setScale(2, RoundingMode.HALF_UP);
        autoHoodPos = Math.min(Math.max(0.2, bd.doubleValue()), 0.86);
        return autoHoodPos;
    }


    public double distCalc(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        return Math.sqrt((dx * dx) + (dy * dy));
    }


    /**
     * Calculates flywheel velocity based on distance and hoodvelocity
     * This is a tunable exponential formula
     */
    public int veloCalc(double distance) {
        double velocity = 0.00000128898 * Math.pow(distance, 4)
            + 0.000774931 * Math.pow(distance, 3)
            - 0.302351 * Math.pow(distance, 2)
            + 34.96837 * distance
            + 248.70575;
        return (int) velocity;
    }


    /**
     * Automatic velocity + hood control
     */
    public void autoVelocity(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        distance = distCalc(pose);


//        // Hood control
//        double hoodPos = (hoodCalc(distance))-hoodOffset;
//        hood.setPosition(hoodPos);
//        autoHoodPos = hoodPos;


        if (distance > 100) {//far
            shootVelocity(OuttakeConstants.FAR_VELOCITY1);
            hood.setPosition(hoodCalc(distance));
        }
        else {//close
            shootVelocity(OuttakeConstants.CLOSE_VELOCITY2);
            hood.setPosition(hoodCalc(distance));
        }


        // Velocity control
//        int velocity = veloCalc(distance);
//        shootVelocity(velocity);
//        autoVelo = velocity;


        currentHoodPos = hood.getPosition();
        telemetry.addData("Distance", distance);
    }


    //============== ACTIONS =============
    public Action autoVelocityAction(double robotX, double robotY, double goalX, double goalY) {
        return new Action() {
            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) init = true;


                // Compute distance
                double dx = goalX - robotX;
                double dy = goalY - robotY;
                double distance = distCalc(new Pose2d(robotX, robotY, 0));


                // Hood
                double hoodPos = hoodCalc(distance);
                hood.setPosition(hoodPos);


                // Flywheel
                int velocity = veloCalc(distance);
                shootVelocity(velocity);




                telemetryPacket.put("Hood", hood.getPosition());
                telemetryPacket.put("Velocity", velocity);


                // Return true if flywheel is at target (within 50 rpm)
                double error = Math.abs(getVelocity() - velocity);
                telemetryPacket.put("Error", error);
                return error > 50;
            }
        };
    }


    // Automatic velocity + hood action with time limitsetVeloc
    public Action shootVelocityTimeAction(int velocity, double time) {
        return new Action() {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    init = true;
                    timer.reset();
                }


                if (timer.seconds() < time) {
                    setVelocity(velocity);
                    bangController(velocity);
                } else {
                    shootStop();
                }


                return timer.seconds() <= time;
            }
        };
    }
    public Action shootVelocityAction(int velocity) {
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
                motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

                motor1.setVelocity(velocity);
                motor2.setVelocity(velocity);


                double error = Math.abs(getVelocity() - velocity);
                telemetryPacket.put("Error", error);
                return error > 50;
            }
        };
    }

    public Action shootCloseAction() {
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                setVelocity(OuttakeConstants.CLOSE_VELOCITY2);
                hood.setPosition(OuttakeConstants.CLOSE_HOOD2);

                double error = Math.abs(getVelocity() - Math.abs(OuttakeConstants.CLOSE_VELOCITY2));


                telemetryPacket.put("Error", error);
                return error > 50;
            }
        };
    }

    public Action shootFarAction() {
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                setVelocity(OuttakeConstants.FAR_VELOCITY1);
                hood.setPosition(OuttakeConstants.FAR_HOOD1);

                double error = Math.abs(getVelocity() - Math.abs(OuttakeConstants.FAR_VELOCITY1));


                telemetryPacket.put("Error", error);
                return error > 50;
            }
        };
    }

    public Action autoVelocityTimeAction(double robotX, double robotY, double goalX, double goalY, double timeLimit) {
        return new Action() {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    init = true;
                    timer.reset();
                }


                if (timer.seconds() < timeLimit) {
                    // Compute distance
                    double dx = goalX - robotX;
                    double dy = goalY - robotY;
                    double distance = Math.sqrt(dx*dx + dy*dy);


                    // Hood
                    double hoodPos = hoodCalc(distance);
                    hood.setPosition(hoodPos);


                    // Flywheel
                    int velocity = veloCalc(distance);
                    shootVelocity(velocity);


                    telemetryPacket.put("Distance", distance);
                    telemetryPacket.put("Hood", hoodPos);
                    telemetryPacket.put("Velocity", velocity);
                } else {
                    shootStop();
                }


                return timer.seconds() >= timeLimit;
            }
        };
    }

    public Action transferHold() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (Math.abs(Math.abs(getVelocity()) - Math.abs(currentVelocity)) <= OuttakeConstants.velocityError) {
                    Robot.intake.transferReverseAction(Math.min(1 - 0.2, 0.4));
                } else {
                    Robot.intake.transferInAction(1);
                }
                Robot.intake.transferInAction(0);
                return false;
            }
        };
    }

    public Action transferHoldTime(Robot robot, double time) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                while (timer.seconds() < time) {
                    if (getVelocity() <= currentVelocity - 150) {
                        robot.intake.transferReverseAction(Math.min(1 - 0.2, 0.4));
                    } else {
                        robot.intake.transferInAction(1);
                    }
                    return true;
                }
                return false;
            }
        };
    }




    public Action stopAction() {
        return  new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                motor1.setPower(0);
                motor2.setPower(0);

                return true;
            }
        };
    }

    public Action hoodAction(double pos, double time) {
        ElapsedTime t = new ElapsedTime();
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hood.setPosition(pos);
                return t.seconds() < time;
            }
        };
    }
}




//package org.firstinspires.ftc.teamcode.fsm;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.PoseStorage;
//import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//@Config
//public class EmergencyFSM {
//
//
//    private Intake intake;
//    private Intake transfer;
//    private Outtake outtake;
////    private final Turret turret;
//    private final GamepadMappings controls;
//    private final Telemetry telemetry;
//    private GazelleState gazelleState;
//    public String telemetry1;
//    public static int transferPower = 1;
//    public EmergencyFSM(LinearOpMode mode, Telemetry telemetry, GamepadMappings controls) {
//        Robot robot = new Robot(mode);
//        this.intake = Robot.intake;
//        this.transfer = Robot.transfer;
////        this.turret = Robot.turret;
//        this.outtake = Robot.outtake;
//        this.controls = controls;
//        this.telemetry = telemetry;
////        turret.setTargetAngle(0);
//    }
//
//
//    // ---------------- Main update loop ----------------
//    public void gazelleUpdate() {
//        controls.update();
//        Robot.driveTrain.update();
//        Robot.drive.localizer.update();
//        Robot.driveTrain.setCurrentHeading(Robot.drive.localizer.getPose().heading.toDouble());
//        //turret.update();
//        telemetry.update();
//
//
//
//
//
//
//        // ---------------- Outtake / Flywheel ----------------
//        //close velocities
////        if (controls.flywheelClose1.value()) {
////            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY1);
////            outtake.hood.setPosition(OuttakeConstants.CLOSE_HOOD1);
////            outtake.currentVelocity = OuttakeConstants.CLOSE_VELOCITY1;
////            controls.flywheelClose2.set(false);
////            controls.flywheelClose3.set(false);
////            controls.flywheelFar1.set(false);
////            controls.flywheelFar2.set(false);
////            if (!controls.turretAuto.value()) turret.autoAlign(robot.drive.localizer.getPose()); else turret.setTargetAngle(0);
//        if (controls.flywheelClose2.value()) {
//            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY2);
//            outtake.hood.setPosition(OuttakeConstants.CLOSE_HOOD2);
//            outtake.currentVelocity = OuttakeConstants.CLOSE_VELOCITY2;
//            controls.flywheelClose1.set(false);
//            controls.flywheelClose3.set(false);
//            controls.flywheelFar1.set(false);
//            controls.flywheelFar2.set(false);
////            if (!controls.turretAuto.value()) turret.autoAlign(robot.drive.localizer.getPose());
////            else turret.setTargetAngle(0);
//
//
////        } else if (controls.flywheelClose3.value()) {
////            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY3);
////            outtake.hood.setPosition(OuttakeConstants.CLOSE_HOOD3);
////            outtake.currentVelocity = OuttakeConstants.CLOSE_VELOCITY3;
////            controls.flywheelClose1.set(false);
////            controls.flywheelClose2.set(false);
////            controls.flywheelFar1.set(false);
////            controls.flywheelFar2.set(false);
////            if (!controls.turretAuto.value()) turret.autoAlign(robot.drive.localizer.getPose()); else turret.setTargetAngle(0);
////        }
//
//
//            //far velocities
//        } else if (controls.flywheelFar1.value()) {
//            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY1);
//            outtake.hood.setPosition(OuttakeConstants.FAR_HOOD1);
//            outtake.currentVelocity = OuttakeConstants.FAR_VELOCITY1;
//
//
//            controls.flywheelClose2.set(false);
//
//
////            if (!controls.turretAuto.value()) turret.autoAlign(robot.drive.localizer.getPose());
////            else turret.setTargetAngle(0);
//        }
////        } else if (controls.flywheelFar2.value()) {
////            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY2);
////            outtake.hood.setPosition(OuttakeConstants.FAR_HOOD2);
////            outtake.currentVelocity = OuttakeConstants.FAR_VELOCITY2;
////            controls.flywheelClose1.set(false);
////            controls.flywheelClose2.set(false);
////            controls.flywheelClose3.set(false);
////            controls.flywheelFar1.set(false);
////            if (!controls.turretAuto.value()) turret.autoAlign(robot.drive.localizer.getPose()); else turret.setTargetAngle(0);
////        }
//        else {
//            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
//            outtake.currentVelocity = 0;
////            turret.setTargetAngle(0);
//        }
//
//
////         ---------------------- Turret ----------------------
//
//
//        //Turret Manual
////        if (controls.turretLeft.locked()) {
////            turret.changeTargetAngle(5);
////        }
////        if (controls.turretRight.locked()) {
////            turret.changeTargetAngle(-5);
////        }
//
//
//        // ---------------- Intake / Transfer ----------------
//        if (controls.intake.locked() || controls.intake2.locked()) {
//            intake.intake();
//            transfer.setPower(0);
//        } else if (controls.intakeReverse.locked()) {
//            intake.intakeReverse();
//            transfer.setPower(-1);
//        } else if (controls.transfer.locked()) {
//            intake.intake();
//
//
//            if (Math.abs(outtake.getVelocity()) <= Math.abs(outtake.currentVelocity) - Math.abs(OuttakeConstants.velocityError)) {
//                intake.transferOut(Math.min(transferPower - 0.2, 0.4));
//            } else {
//                intake.transferIn(transferPower);
//            }
//
//
//        } else if (!controls.transfer.locked() && !controls.intakeReverse.locked()) {
//            intake.intakeStop();
//            transfer.setPower(0);
//        }
//
//
////        if (controls.stopper.value()) {
////            intake.stopper.setPosition(IntakeConstants.BLOCKER_OPEN);
////        } else {
////            intake.stopper.setPosition(IntakeConstants.BLOCKER_CLOSED);
////        }
//
//
//
//
//
//        //--------------- Drivetrain ---------------
//
//
//        //reset odo
//        if (controls.reset.changed()) {
//            Robot.drive.localizer.setPose(new Pose2d(PoseStorage.hpX, PoseStorage.hpY, PoseStorage.hpHeading));
////            turret.setTargetAngle(0);
//        }
//
//
//        //auto turn IN TESTING
//        if (controls.autoTurn.changed()) {
//            Robot.driveTrain.setIsAutoTurning(true);
//
//
//            Robot.driveTrain.setTargetHeading(
//                Robot.driveTrain.calculateHeading(
//                    Robot.drive.localizer.getPose()
//                )
//            );
//        }
//
//
//    }
//
//
//    // ---------------- Getters / Setters ----------------
//    public enum GazelleState {
//        BASE_STATE, INTAKING, TRANSFERRING
//    }
//}
//


package org.firstinspires.ftc.teamcode.fsm;




import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;




import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Config
public class EmergencyFSM {


    private LinearOpMode mode;


    private Intake intake;
    private Intake transfer;
    private Outtake outtake;
    //    private final Turret turret;
    private final GamepadMappings controls;
    private final Telemetry telemetry;
    private GazelleState gazelleState;
    public String telemetry1;
    public static int transferPower = 1;
    public EmergencyFSM(LinearOpMode mode, Telemetry telemetry, GamepadMappings controls) {
        this.mode = mode;
        Robot robot = new Robot(mode);
        this.intake = Robot.intake;
        this.transfer = Robot.transfer;
//        this.turret = Robot.turret;
        this.outtake = Robot.outtake;
        this.controls = controls;
        this.telemetry = telemetry;
//        turret.setTargetAngle(0);
    }




    // ---------------- Main update loop ----------------
    public void gazelleUpdate() {
        controls.update();
        Robot.driveTrain.update(mode.getRuntime());
        Robot.drive.localizer.update();
        Robot.driveTrain.setCurrentHeading(Robot.drive.localizer.getPose().heading.toDouble());
        //turret.update();
        telemetry.update();

        // ---------------- Outtake / Flywheel ----------------
        //close velocities
        if (controls.flywheelClose2.value()) {
            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY2);
            outtake.hood.setPosition(OuttakeConstants.CLOSE_HOOD2);
            outtake.currentVelocity = OuttakeConstants.CLOSE_VELOCITY2;

            controls.flywheelFar1.set(false);
        }
        else if (controls.flywheelFar1.value()) {
            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY1);
            outtake.hood.setPosition(OuttakeConstants.FAR_HOOD1);
            outtake.currentVelocity = OuttakeConstants.FAR_VELOCITY1;


            controls.flywheelClose2.set(false);
        }
        else {
            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
            outtake.currentVelocity = 0;
//            turret.setTargetAngle(0);
        }




//         ---------------------- Turret ----------------------




        //Turret Manual
//        if (controls.turretLeft.locked()) {
//            turret.changeTargetAngle(5);
//        }
//        if (controls.turretRight.locked()) {
//            turret.changeTargetAngle(-5);
//        }




        // ---------------- Intake / Transfer ----------------
        if (controls.intake.locked() || controls.intake2.locked()) {
            intake.intake();
            transfer.setPower(0);
        } else if (controls.intakeReverse.locked()) {
            intake.intakeReverse();
            transfer.setPower(-1);
        } else if (controls.transfer.locked()) {
            intake.intake();




            if (Math.abs(outtake.getVelocity()) <= Math.abs(outtake.currentVelocity) - Math.abs(OuttakeConstants.velocityError)) {
                intake.transferOut(Math.min(transferPower - 0.2, 0.4));
            } else {
                intake.transferIn(transferPower);
            }




        } else if (!controls.transfer.locked() && !controls.intakeReverse.locked()) {
            intake.intakeStop();
            transfer.setPower(0);
        }




//        if (controls.stopper.value()) {
//            intake.stopper.setPosition(IntakeConstants.BLOCKER_OPEN);
//        } else {
//            intake.stopper.setPosition(IntakeConstants.BLOCKER_CLOSED);
//        }










        //--------------- Drivetrain ---------------




        //reset odo
        if (controls.reset.changed()) {
            Robot.drive.localizer.setPose(new Pose2d(PoseStorage.hpX, PoseStorage.hpY, PoseStorage.hpHeading));
//            turret.setTargetAngle(0);
        }



        //auto turn IN TESTING
        Robot.driveTrain.isAutoTurning = controls.autoTurn.value();




    }




    // ---------------- Getters / Setters ----------------
    public enum GazelleState {
        BASE_STATE, INTAKING, TRANSFERRING
    }
}








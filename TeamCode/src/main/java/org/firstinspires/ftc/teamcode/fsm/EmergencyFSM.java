package org.firstinspires.ftc.teamcode.fsm;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class EmergencyFSM {

    private Intake intake;
    private Outtake outtake;
    private GamepadMappings controls;
    private Robot robot;
    private Turret turret;
    private Telemetry telemetry;
    private PinpointLocalizer pinpoint;
    private EmergencyFSM.GazelleState gazelleState;
    public EmergencyFSM(Telemetry telemetry, GamepadMappings controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.turret = robot.turret;
        this.pinpoint = robot.pinpoint;
        this.controls = controls;
        this.telemetry = telemetry;

        gazelleState = EmergencyFSM.GazelleState.BASE_STATE;

    }
    public void gazelleUpdate(){
        controls.update();
        robot.driveTrain.update();


        switch (gazelleState){
            case BASE_STATE:
                robot.intake.intakeStop();
                robot.intake.blockerClose();
                robot.outtake.shootStop();
                if (controls.intake.locked()){
                    gazelleState = EmergencyFSM.GazelleState.INTAKING;
                } else if (controls.intakeReverse.locked()){
                    gazelleState = EmergencyFSM.GazelleState.INTAKING;
                }

                //intake off, blocker closed, flywheel off
                break;
            case INTAKING:

                intake.blockerClose();
                if (controls.intake.locked()) {
                    intake.intake();
                } else if (controls.intakeReverse.locked()){
                    intake.intakeReverse();
                } else if (controls.servoBlocker.value()){
                    gazelleState = EmergencyFSM.GazelleState.TRANSFERRING;
                } else if (controls.flywheel.value()){
                    gazelleState = EmergencyFSM.GazelleState.SPINUP;
                } else {
                    intake.intakeStop();
                }
                //intake on, blocker closed
                break;
            case SPINUP:
                outtake.shootVelocity(1770);
                if (controls.servoBlocker.value()) {
                    gazelleState= EmergencyFSM.GazelleState.TRANSFERRING;
                } else if (controls.intake.locked()){
                    gazelleState = EmergencyFSM.GazelleState.INTAKING;
                } else if (controls.intakeReverse.locked()){
                    gazelleState = EmergencyFSM.GazelleState.INTAKING;
                } else if(controls.flywheel.value()){
                    gazelleState = EmergencyFSM.GazelleState.BASE_STATE;
                }
                //flywheel on
                //if done and right trigger, allow transfer
                break;
            case TRANSFERRING:
                intake.blockerOpen();
                if (controls.intake.locked()){
                    intake.intake();
                } else if (controls.intakeReverse.locked()){
                    gazelleState= EmergencyFSM.GazelleState.INTAKING;
                } else if (controls.flywheel.value()){
                    gazelleState = EmergencyFSM.GazelleState.SPINUP;
                }
                else {
                    intake.intakeStop();
                }

                //intake on, blocker up, flywheel on
                break;
        }
    }
    public EmergencyFSM.GazelleState getState(){
        return gazelleState;
    }
    public void setState (EmergencyFSM.GazelleState newState) {
        gazelleState = newState;
    }

    public enum GazelleState {
        BASE_STATE("BASE_STATE"),
        INTAKING ("INTAKING"),
        SPINUP ("SPINUP"),
        TRANSFERRING ("TRANSFERRING");


        private String state;

        GazelleState (String stateName){
            state = stateName;
        }
        public String stateName() {
            return state;
        }
    }
}

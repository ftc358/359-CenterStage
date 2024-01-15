package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoboConstants;

@Config
@TeleOp (name = "V2 Meet 3 TeleOp",group = "TeleOP")
public class NewTeleOP extends LinearOpMode {

    // Dashboard Vars
    public static double diffyDiff = RoboConstants.diffyDiff;

    public static double rapidTrigger_thr = RoboConstants.rapidTrigger_thr;
    public static double extension_sens = RoboConstants.extension_sens;

    public static double flip_lift = RoboConstants.flip_lift;
    public static double flip_intake = RoboConstants.flip_intake;
    public static double flip_half = RoboConstants.flip_half;
    public static double flip_dump = RoboConstants.flip_dump;
    public static double flip_vert = RoboConstants.flip_vert;

    public static double ppGet = RoboConstants.ppGet;
    public static double ppHold = RoboConstants.ppHold;
    public static double ppBoardDrop = RoboConstants.ppBoardDrop;

    public static double liftIdle = RoboConstants.liftIdle;
    public static double liftDown = RoboConstants.liftDown;

    public static double intakeSpeed = RoboConstants.intakeSpeed;

    public static double claw1Grab = RoboConstants.claw1Grab;
    public static double claw2Grab = RoboConstants.claw2Grab;

    // Servo Pos
    double flipPos;
    double claw1Pos = 0;
    double claw2Pos = 0;
    double pp1Pos = 0;
    double pp2Pos = 0;
    double ppAngle = 0.5;
    double planeReleasePos = 0;

    // States
    float ext_past;
    double straight = 0;
    double turn = 0;
    double strafe = 0;
    double extPos;

    double intakePower = 0;
    double liftPower = 0;
    double ratchet = 0;

    double currentVoltage = 12;
    double drivetrainC = 0;
    double driveTrainPower = 0;
    double leftFrontC = 0;
    double leftBackC = 0;
    double rightFrontC = 0;
    double rightBackC = 0;
    double maxPower = 0;
    double fiveC = 0;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    boolean slowflag = false;
    double microAdjust = 0;
    int dropState = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        ext_past = 0;


        int transferState = 0;
        int diffyState = 0;
        double extActualPos, difActualPos;
        boolean flag = false;
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        int speedLimit = 1;
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            straight = (-gamepad1.left_stick_y > 0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.26) : (-gamepad1.left_stick_y < -0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.26) : 0;
            strafe = (gamepad1.left_stick_x > 0.1) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.26) : (gamepad1.left_stick_x < -0.1) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.26) : 0;
            turn = (gamepad1.right_stick_x > 0.1) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.26) : (gamepad1.right_stick_x < -0.1) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.26) : 0;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight * speedLimit, -strafe * speedLimit), -turn * speedLimit));
            drive.updatePoseEstimate();

            //Encoder Position Reading
            difActualPos = drive.difPosEnc.getVoltage() /3.3*360;
            extActualPos = drive.extPosEnc.getVoltage() /3.3*360;
            telemetry.addData("extActualPos",extActualPos);
            telemetry.addData("difActualPos",difActualPos);

            //Intake - Working Prototype
            if ((gamepad2.left_trigger > 0.1) && (flipPos == flip_intake)) {
                intakePower = intakeSpeed;
            } else {
                intakePower = 0;
            }
            if (gamepad2.left_bumper) {
                intakePower = -1;
            }
            drive.intake.setPower(intakePower);
            //END


            liftPower = (currentGamepad2.right_trigger > 0) ? currentGamepad2.right_trigger : (currentGamepad2.right_bumper) ? liftDown : liftIdle;
            telemetry.addData("LiftPower",liftPower);
            //Attack Plan R
            planeReleasePos = (currentGamepad1.options || currentGamepad2.options) ? 1 : 0;

            //Ratchet Power
            ratchet = Math.abs(currentGamepad2.left_stick_y);

            //Extension
            if ((gamepad2.left_trigger >= 0.2)){
                extPos = ((1.0 - extension_sens + currentGamepad2.left_trigger*extension_sens)*1);
            } else{

                if (extActualPos>200) {
                    extPos = 0;
                } else {
                    extPos = 0.3;
                }
            }



            



            //Flipper
            if (extPos<(1.0-extension_sens)||(currentGamepad2.left_trigger <0.2 &&!(extActualPos>220))){//added extPos condition here
                //extension retract
                flipPos = flip_half;}
            else if ((currentGamepad2.left_trigger) > 0.1){
                //extension trigger
                flipPos = flip_intake;}

            if (extActualPos>205){
                flipPos = flip_lift;
            }

            ////Additional Flipper Functions
            if (currentGamepad2.left_bumper) {//shake
                flipPos -= 0.08;
            }else if (currentGamepad2.dpad_down){//dump
                flipPos = flip_dump;
            }

            telemetry.addData("flipPos",flipPos);
            telemetry.addData("extPos",extPos);

            //Transfer Iteratives

            if ((currentGamepad2.cross && !previousGamepad2.cross) && transferState<=3){
                transferState = transferState +1;
            }

            if ((currentGamepad2.dpad_left && !previousGamepad2.dpad_left)&& transferState == 1){
                transferState = 0;
            }
            if ((currentGamepad2.dpad_left && !previousGamepad2.dpad_left)&& transferState >= 3){
                if (transferState < 6){
                    transferState = 6;
                }else {
                    transferState = (transferState + 1) % 8;
                }
            }
            if ((currentGamepad2.dpad_left && !previousGamepad2.dpad_left)&& transferState == 1){
                transferState = 0;
            }


            if (((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) || (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)) && transferState >= 3){
                transferState = (transferState +1)%8;
            }
            if (currentGamepad2.square && diffyState>=1){
                diffyState = 2;
            } else if ((currentGamepad2.triangle && diffyState >=1) || (currentGamepad2.cross && (transferState == 5))){
                diffyState = 1;
            } else if (currentGamepad2.circle && diffyState >=1){
                diffyState = 3;
            }

            if (gamepad2.dpad_right){
                transferState = 0;
                diffyState = 0;
            }


            //claw;

            switch (transferState){
                case 0:
                    pp1Pos = ppGet;
                    pp2Pos = ppGet;
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
                case 1:
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
//                    pp1Pos = ppGet+0.05;
//                    pp2Pos = ppGet+0.05;
                    break;
                case 2:
                    flipPos = 0;
                    extPos = 0.5;
                    if (extActualPos<190) {
                        pp1Pos = ppBoardDrop;
                        pp2Pos = ppBoardDrop;
                        slowflag=true;
                        if (difActualPos>180) {
                            diffyState = 1;
                            transferState = 3;
                        }
                    }
                    break;
                case 3:
                     // At Right Side, dif actual pos is 330+
                                    // At Neutral, dif actual pos is 250 something
                                    //At left positio, dif actual pos is 166
                    break;
                case 4: ////Drops First Pixel
                   claw2Pos = 0;
                   break;
                case 5: //Drops Second Pixel
                    claw1Pos = 0;
                    slowflag = false;
                    break;
                case 6: //Restores
                    diffyState = 1;
                    flipPos = 0;

                    if (drive.placerPivot1.getPosition()==ppBoardDrop){
                        transferState = 7;
                    }
                    break;
                case 7:
                    extPos = 0.45;
                    if (extActualPos<200) {
                        diffyState = 0;
                        pp1Pos = 0.05;
                        pp2Pos = 0.05;
                        if (difActualPos<50) {
                            transferState = 0;
                        }
                    }
            }


            switch (diffyState){
                case 0: break;
                case 1: //middle
                    pp1Pos = ppBoardDrop;
                    pp2Pos = ppBoardDrop;
                    microAdjust = (-gamepad1.left_trigger+gamepad1.right_trigger+gamepad2.right_stick_x)/2*0.15;
                    pp1Pos -= microAdjust;
                    pp2Pos += microAdjust;
                    break;
                case 2: //left
                    pp1Pos = ppBoardDrop +diffyDiff;
                    pp2Pos = ppBoardDrop -diffyDiff;
                    ppAngle = 0.4; //Limit 0.18
                    microAdjust = (-gamepad1.left_trigger+gamepad1.right_trigger+gamepad2.right_stick_x)/2*0.15;
                    pp1Pos -= microAdjust;
                    pp2Pos += microAdjust;
                    break;
                case 3://right
                    pp1Pos = ppBoardDrop -diffyDiff;
                    pp2Pos = ppBoardDrop +diffyDiff;
                    ppAngle = 0.6; //Limit 0.82
                    microAdjust = (-gamepad1.left_trigger+gamepad1.right_trigger+gamepad2.right_stick_x)/2*0.15;
                    pp1Pos -= microAdjust;
                    pp2Pos += microAdjust;
                    break;
            }

            drive.flip1.setPosition(flipPos); //moved flipPos here goodluck
            drive.flip2.setPosition(flipPos);
            drive.placerPivot1.setPosition(pp1Pos);
            drive.placerPivot2.setPosition(pp2Pos);
            drive.claw1.setPosition(claw1Pos);
            drive.claw2.setPosition(claw2Pos);
            drive.ext1.setPosition(extPos);
            drive.ext2.setPosition(extPos);
            drive.planeRelease.setPosition(planeReleasePos);
            drive.lift2.setPower(liftPower);
            drive.climb1.setPower(ratchet);
            drive.climb2.setPower(ratchet);


            telemetry.addData("TransferState",transferState);
            telemetry.addData("DiffyState",diffyState);
            telemetry.update();
        }


    }
}


package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoboConstants;

@Config
@TeleOp (name = "V1 Meet 3 TeleOp",group = "TeleOP")
public class V1TeleOPMeet3 extends LinearOpMode{

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

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        ext_past = 0;

        drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something

        if (currentVoltage < 12) {
            telemetry.addLine("Current Voltage is Below 12v. Please replace battery before match");
            telemetry.update();
        }


        int transferState = 0;


        waitForStart();
        while (opModeIsActive()) {
            //trust me bro - jonathan
            //in switchCase i trust - lucas


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            //Power Systems
            currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something
            leftFrontC = drive.leftFront.getCurrent(CurrentUnit.AMPS);
            leftBackC = drive.leftBack.getCurrent(CurrentUnit.AMPS);
            rightFrontC = drive.rightFront.getCurrent(CurrentUnit.AMPS);
            rightBackC = drive.rightBack.getCurrent(CurrentUnit.AMPS);
            drivetrainC = leftFrontC + leftBackC + rightFrontC + rightBackC;
            fiveC = drive.ch.getCurrent(CurrentUnit.AMPS) +drive.exh.getCurrent(CurrentUnit.AMPS);
            driveTrainPower = currentVoltage * drivetrainC;
            maxPower = (driveTrainPower > maxPower) ? driveTrainPower : maxPower;

            telemetry.addLine("Powertrain Statistics");
            telemetry.addLine();
            telemetry.addData("Drive Current (A)", drivetrainC);
            telemetry.addData("Total Current (A)", fiveC);
            telemetry.addData("Power (W)", driveTrainPower);
            telemetry.addData("Max Drive Power (W)", maxPower);

            if (slowflag){
                straight = (-gamepad1.left_stick_y/2 > 0.1) ? (-gamepad1.left_stick_y/2 + 0.26) : (-gamepad1.left_stick_y < -0.1) ? (-gamepad1.left_stick_y - 0.26) : 0;
                strafe = (gamepad1.left_stick_x > 0.1) ? (gamepad1.left_stick_x/2 + 0.26) : (gamepad1.left_stick_x < -0.1) ? (gamepad1.left_stick_x/2 - 0.26) : 0;
                turn = (gamepad1.right_stick_x > 0.1) ? (gamepad1.right_stick_x/2 + 0.26) : (gamepad1.right_stick_x < -0.1) ? (gamepad1.right_stick_x/2 - 0.26) : 0;
            } else{
                straight = (-gamepad1.left_stick_y > 0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.26) : (-gamepad1.left_stick_y < -0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.26) : 0;
                strafe = (gamepad1.left_stick_x > 0.1) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.26) : (gamepad1.left_stick_x < -0.1) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.26) : 0;
                turn = (gamepad1.right_stick_x > 0.1) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.26) : (gamepad1.right_stick_x < -0.1) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.26) : 0;
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight, -strafe), -turn));
            drive.updatePoseEstimate();

            //Bucket Flip
            telemetry.addData("flipThresThingy",(currentGamepad2.left_trigger - ext_past));
            flipPos = (currentGamepad2.dpad_down)?flip_dump:((currentGamepad2.left_trigger - ext_past) <rapidTrigger_thr)?flip_intake:flip_half;
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){flipPos -= 0.08;} //reversed shake



            //Intake Trigger
            intakePower = (currentGamepad2.left_trigger > 0 && !currentGamepad2.dpad_down) ? intakeSpeed : (currentGamepad2.dpad_down || currentGamepad2.dpad_down) ? -1 : 0;

            //Extension Trigger
//            extPos = 0.5;
            extPos = (currentGamepad2.left_trigger >= 0.1) ? (1.0 - extension_sens + currentGamepad2.left_trigger * extension_sens) * 0.8 : 0;

            //Lift Trigger
            liftPower = (currentGamepad2.right_trigger > 0) ? currentGamepad2.right_trigger : (currentGamepad2.right_bumper) ? liftDown : liftIdle;

            //Attack Plan R
            planeReleasePos = (currentGamepad1.options || currentGamepad2.options) ? 1 : 0;

            //Ratchet Power
            ratchet = Math.abs(currentGamepad2.left_stick_y);
            if (extPos == 0) {flipPos = flip_lift;}



            if (gamepad2.left_stick_y!=0){gamepad2.rumble(100);}

            //Gripping States
            if (((currentGamepad1.left_bumper && !previousGamepad1.left_bumper)||(currentGamepad1.right_bumper && !previousGamepad1.right_bumper)) && (transferState >= 4)){
                dropState = (dropState +1)%5;
            }

            //Transfer States
            if (currentGamepad2.square && !previousGamepad2.square && (transferState<4 || transferState==7)) {
                transferState = (transferState + 1) % 8 ;
            }
            else if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left && ((transferState == 4)||(transferState == 6))){ //turns the diffy left
                transferState = 5;
            } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right && ((transferState == 4)||(transferState==5))){ //turns the diffy right
                transferState = 6;
            } else if ((currentGamepad2.dpad_up && !previousGamepad2.dpad_up)&& (transferState == 5 || transferState ==6) ){
                transferState = 4;
            } else if (currentGamepad2.triangle && transferState == 4) {//Reset Anytime
                transferState = 0;
            } else if (currentGamepad2.triangle){
                transferState = 7;
            }

                            //Transfer Switch-Case
            switch (transferState) {
                case 0: //home
                    pp1Pos = ppGet;
                    pp2Pos = ppGet;
                    claw1Pos = 0;
                    claw2Pos = 0;
                    slowflag = false;
                    break;
                case 1: //bucket in
                    flipPos = flip_lift;
                    transferState = 2;
                    break;
                case 2: //pp up a bit
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    break;
                case 3: //hold and goes to Holding Position
                    extPos = 0.1;
                    flipPos = 0.15;
                    pp1Pos = ppHold;
                    pp2Pos = ppHold;
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    break;

                case 4: //Goes to Board Dropping Position // Diffy Mid
                    pp1Pos = ppBoardDrop;
                    pp2Pos = ppBoardDrop;
                    slowflag=true;

                    microAdjust = (-gamepad1.left_trigger+gamepad1.right_trigger+gamepad2.right_stick_x)/2*0.15;
                    pp1Pos -= microAdjust;
                    pp2Pos += microAdjust;
                    break;

                case 5: //Diffy Left
                    pp1Pos = ppBoardDrop +diffyDiff;
                    pp2Pos = ppBoardDrop -diffyDiff;
                    ppAngle = 0.4; //Limit 0.18
                    microAdjust = (-gamepad1.left_trigger+gamepad1.right_trigger+gamepad2.right_stick_x)/2*0.15;
                    pp1Pos -= microAdjust;
                    pp2Pos += microAdjust;
                    break;
                case 6: //Diffy Right
                    pp1Pos = ppBoardDrop -diffyDiff;
                    pp2Pos = ppBoardDrop +diffyDiff;
                    ppAngle = 0.6; //Limit 0.82
                    microAdjust = (-gamepad1.left_trigger+gamepad1.right_trigger+gamepad2.right_stick_x)/2*0.15;
                    pp1Pos -= microAdjust;
                    pp2Pos += microAdjust;
                    break;

                case 7: //homes
                    slowflag=false;
                    pp1Pos = ppBoardDrop;
                    pp2Pos = ppBoardDrop;
                    break;
            }


            switch (dropState){
                case 0:
                    break;
                case 1:
                    claw2Pos = 0;
                    break;
                case 2:
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
                case 3:
                    if (transferState==4){transferState = 7;
                        dropState = 4;}
                    else {
                        transferState = 7;
                    }
                    break;
                case 4:
                    transferState = 0;
                    dropState = 0;
                    break;

            }


            telemetry.addData("pp1Pos",pp1Pos);
            telemetry.addData("pp2Pos",pp2Pos);



            //Unified Servo Actuation
            drive.ext1.setPosition(extPos);
            drive.ext2.setPosition(extPos);
            drive.flip1.setPosition(flipPos);
            drive.flip2.setPosition(flipPos);
            drive.claw1.setPosition(claw1Pos);
            drive.claw2.setPosition(claw2Pos);
            drive.placerPivot1.setPosition(pp1Pos);
            drive.placerPivot2.setPosition(pp2Pos);
            drive.planeRelease.setPosition(planeReleasePos);

            //Unified External Motor Action
            drive.lift2.setPower(liftPower);
            drive.intake.setPower(intakePower);
            drive.climb1.setPower(ratchet);
            drive.climb2.setPower(ratchet);


            //Debug
            telemetry.addLine("Drive");
            telemetry.addData("Straight", straight);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);


            telemetry.addData("Current Transfer State ", transferState);
            telemetry.addData("Current Grab State ", dropState);

            ext_past = currentGamepad2.left_trigger;
            telemetry.update();


    }
}
}


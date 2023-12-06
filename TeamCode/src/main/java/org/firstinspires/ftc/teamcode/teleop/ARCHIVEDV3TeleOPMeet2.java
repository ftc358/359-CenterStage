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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Disabled
@TeleOp (name = "V2 Mesajdsajdsajet 2 TeleOp",group = "TeleOP")
public class ARCHIVEDV3TeleOPMeet2 extends LinearOpMode{

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private boolean AprilTagInRange(){
        boolean state = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections){
            if (detection.metadata != null || detection.id == 1 ||detection.id == 2 ||detection.id == 3 ||detection.id == 4 ||detection.id == 5 ||detection.id ==6){
                if (detection.ftcPose.range<5) {return true;}
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            }
        }
        return state;
    }

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

    double currentVoltage = 12;
    double drivetrainC = 0;
    double driveTrainPower = 0;
    double leftFrontC = 0;
    double leftBackC = 0;
    double rightFrontC = 0;
    double rightBackC = 0;
    double maxPower = 0;
    double fiveC = 0;


    Gamepad.RumbleEffect batteryCritical, drop1Ready, drop2Ready;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    boolean testingScheme;
    int previousState = 0;



    @Override

    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        ext_past = 0;

        drive.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something

        if (currentVoltage < 12) {
            telemetry.addLine("Current Voltage is Below 12v. Please replace battery before match");
            telemetry.update();
        }


        int transferState = 0;
        initAprilTag();
        while (opModeInInit() && !opModeIsActive()){
            if (gamepad1.ps){
                telemetry.addLine("Control Scheme: Testing");
                testingScheme = true;
                gamepad1.rumble(2000);
            }
        }
        waitForStart();
        while (opModeIsActive()) {
            //trust me bro - jonathan
            //in switchCase i trust - lucas
            if (testingScheme){
                telemetry.addLine("CONTROLLER A ONLY");
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad1);}
            else {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }


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


            //Curving Controls. This is extremely unecessary but it saves space
            straight = (-gamepad1.left_stick_y > 0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.26) : (-gamepad1.left_stick_y < -0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.26) : 0;
            strafe = (gamepad1.left_stick_x > 0.1) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.26) : (gamepad1.left_stick_x < -0.1) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.26) : 0;
            turn = (gamepad1.right_stick_x > 0.1) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.26) : (gamepad1.right_stick_x < -0.1) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.26) : 0;

            if(AprilTagInRange()){
                straight /= 2;
                strafe /= 2;
                turn /= 2;
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight, -strafe), -turn));
            drive.updatePoseEstimate();

            //Bucket Flip
            flipPos = (currentGamepad2.dpad_left)?flip_dump:((currentGamepad2.left_trigger - ext_past) > 0)?flip_intake:flip_vert;

            //Intake Trigger
            intakePower = (currentGamepad2.left_trigger > 0 && !currentGamepad2.dpad_left) ? intakeSpeed : (currentGamepad2.left_bumper || currentGamepad2.dpad_left) ? -1 : 0;

            //Extension Trigger
            extPos = (currentGamepad2.left_trigger >= 0.1) ? (1.0 - extension_sens + currentGamepad2.left_trigger * extension_sens) * 0.8 : 0;

            //Lift Trigger
            liftPower = (currentGamepad2.right_trigger > 0) ? currentGamepad2.right_trigger : (currentGamepad2.right_bumper) ? liftDown : liftIdle;

            //Attack Plan R
            planeReleasePos = (currentGamepad1.options || currentGamepad2.options) ? 0 : 1;


            //Transfer States
            if (currentGamepad2.x && !previousGamepad2.x && (transferState<=4 ||transferState==7 ||transferState ==8)) {
                transferState = (transferState + 1) % 10 ;
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && (transferState == 4 || transferState == 5 || transferState == 6)){
                transferState = 7;
            } else if (currentGamepad2.square && !previousGamepad2.square && (transferState == 4)){ //turns the diffy left
                transferState = 5;
                previousState = 5;
            } else if (currentGamepad2.circle && !previousGamepad2.circle && (transferState == 4)){ //turns the diffy right
                transferState = 6;
                previousState = 6;

            } else if ((currentGamepad1.square && !previousGamepad1.square || currentGamepad1.circle && !previousGamepad1.circle)&&  (previousState == 5 || previousState ==6) ){
                transferState = 4;
                previousState = 0;
            }


            else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && transferState == 8) { //Resets down.
                transferState = 9;
            } else if (currentGamepad2.a) {//Reset Anytime
                transferState = 9;
            }

                            //Transfer Switch-Case
            switch (transferState) {
                case 0: //home
                    pp1Pos = ppGet;
                    pp2Pos = ppGet;
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
                case 1: //bucket in
                    flipPos = flip_lift;
                    break;
                case 2: //pp up a bit
                    pp1Pos = 0.1;
                    pp2Pos = 0.1;
                    if (drive.diffyRightPos()==0.1){//Tune this pos.
                        transferState = 3;
                    }
                    break;
                case 3: //hold and goes to Holding Position
                    pp1Pos = ppHold;
                    pp2Pos = ppHold;
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    break;




                case 4: //Goes to Board Dropping Position // Ready to Drop Right Away
                    pp1Pos = ppBoardDrop;
                    pp2Pos = ppBoardDrop;
                    break;
                case 5: //Diffy Left
                    pp1Pos = ppBoardDrop +diffyDiff;
                    pp2Pos = ppBoardDrop -diffyDiff;
                    ppAngle = 0.4; //Limit 0.18
                    break;
                case 6: //Diffy Right
                    pp1Pos = ppBoardDrop -diffyDiff;
                    pp2Pos = ppBoardDrop +diffyDiff;
                    ppAngle = 0.6; //Limit 0.82
                    break;



                case 7: //Drops the Outermost
                    claw2Pos = 0;
                    break;
                case 8: //Drops the Innermost
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
                case 9:
                    pp1Pos = ppBoardDrop;
                    pp2Pos = ppBoardDrop;
                    if (drive.diffyLeftPos()==drive.diffyRightPos()){transferState = 0;}
                    break;
            }


            telemetry.addData("pp1Pos",pp1Pos);
            telemetry.addData("pp2Pos",pp2Pos);
            telemetry.addData("encoderpp1",drive.diffyLeftPos());
            telemetry.addData("encoderpp2",drive.diffyRightPos());




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
            drive.lift1.setPower(liftPower);
            drive.lift2.setPower(liftPower);
            drive.intake.setPower(intakePower);


            //Debug
            telemetry.addLine("Drive");
            telemetry.addData("Straight", straight);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);


            telemetry.addData("Current Transfer State ", transferState);
            ext_past = currentGamepad2.right_trigger;
            telemetry.update();


            sleep(10);
        }


        visionPortal.close();

    }   // end method doCameraSwitching()
    }


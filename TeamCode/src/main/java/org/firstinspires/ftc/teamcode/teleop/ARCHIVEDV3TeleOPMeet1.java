package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp (name = "V3 Vision Infused Meet 1 TeleOp",group = "TeleOP")
@Disabled
public class ARCHIVEDV3TeleOPMeet1 extends LinearOpMode{
    // saved on codeshare.io/lmaocopeharderpeopleofpaper727

    // Constants Import
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
    public double claw2Pos = 0;
    public double ppPos = 0;

    double planeReleasePos = 0;

    // States
    float ext_past;

    // PS5 Rumble

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



    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TfodProcessor tfod;


    @Override
    public void runOpMode() throws InterruptedException {
        batteryCritical = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 1000)
                .build();

        drop1Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 0.7, 200)
                .addStep(0, 0, 20)
                .build();
        drop2Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 0.7, 100)
                .addStep(0, 0, 100)
                .build();


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

        while (opModeInInit() && !opModeIsActive()){
            if (gamepad1.ps){
                telemetry.addLine("Control Scheme: Testing");
                testingScheme = true;
                gamepad1.rumble(2000);
            }

            initVision();
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
            telemetry.addData("Left Front Current (A)", leftFrontC);
            telemetry.addData("Left Back Current (A)", leftBackC);
            telemetry.addData("Right Front Current (A)", rightFrontC);
            telemetry.addData("Right Back Current (A)", rightBackC);
            telemetry.addLine();
            telemetry.addData("Drive Current (A)", drivetrainC);
            telemetry.addData("Total Current (A)", fiveC);
            telemetry.addData("Power (W)", driveTrainPower);
            telemetry.addData("Max Drive Power (W)", maxPower);


            //Curving Controls. This is extremely unecessary but it saves space
            straight = (-gamepad1.left_stick_y > 0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.26) : (-gamepad1.left_stick_y < -0.1) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.26) : 0;
            strafe = (gamepad1.left_stick_x > 0.1) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.26) : (gamepad1.left_stick_x < -0.1) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.26) : 0;
            turn = (gamepad1.right_stick_x > 0.1) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.26) : (gamepad1.right_stick_x < -0.1) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.26) : 0;


            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight, -strafe), -turn));
            drive.updatePoseEstimate();

            //Bucket Flip
            if (currentGamepad2.dpad_left) {
                flipPos = flip_dump;
            } else if (currentGamepad2.left_trigger == 0 && !currentGamepad2.dpad_left) {
                flipPos = flip_vert;
            } else if (((currentGamepad2.left_trigger - ext_past) > 0) && !currentGamepad2.dpad_left) {
                //extension trigger
                flipPos = flip_intake;
            } else if (((ext_past - currentGamepad2.left_trigger) > rapidTrigger_thr) && !currentGamepad2.dpad_left) {
                //extension retract
                flipPos = flip_half;
            }

            //Intake Trigger
            intakePower = (currentGamepad2.left_trigger > 0 && !currentGamepad2.dpad_left) ? intakeSpeed : (currentGamepad2.left_bumper || currentGamepad2.dpad_left) ? -1 : 0;

            //Extension Trigger
            extPos = (currentGamepad2.left_trigger >= 0.1) ? (1.0 - extension_sens + currentGamepad2.left_trigger * extension_sens) * 0.8 : 0;

            //Lift Trigger
            liftPower = (currentGamepad2.right_trigger > 0) ? currentGamepad2.right_trigger : (currentGamepad2.right_bumper) ? liftDown : liftIdle;

            //Attack Plan R
            planeReleasePos = currentGamepad1.options ? 1.0 : 0;


            //Transfer States
            if (currentGamepad2.x && !previousGamepad2.x && (transferState<=4 ||transferState==6)) {
                transferState = (transferState + 1) % 7 ;
            } else if (currentGamepad1.a && !previousGamepad1.a && (transferState == 4 || transferState == 5 || transferState == 6)){
                transferState = (transferState + 1) % 7;
            }
            else if (currentGamepad1.b && !previousGamepad1.b && transferState == 5) {
                transferState = 0;
            } else if (currentGamepad2.a) {
                transferState = 0;
            }

                            //Transfer Switch-Case
            switch (transferState) {
                case 0:
                    ppPos = ppGet;
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
                case 1:
                    ppPos = 0;
                    flipPos = flip_lift;
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
                case 2:
                    ppPos = 0.1;
                    flipPos = flip_lift;
                    break;
                case 3: //Goes to Holding Position
                    ppPos = ppHold;
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    break;
                case 4: //Goes to Board Dropping Position
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    flipPos = flip_lift;
                    ppPos = ppBoardDrop;
                    gamepad1.runRumbleEffect(drop1Ready);
                    break;
                case 5: //Drops the Outermost
                    claw2Pos = 0;
                    gamepad1.runRumbleEffect(drop2Ready);
                    break;
                case 6: //Drops the Innermost
                    claw1Pos = 0;
                    claw2Pos = 0;
                    break;
            }






            //Unified Servo Actuation
            drive.ext1.setPosition(extPos);
            drive.ext2.setPosition(extPos);
            drive.flip1.setPosition(flipPos);
            drive.flip2.setPosition(flipPos);
            drive.claw1.setPosition(claw1Pos);
            drive.claw2.setPosition(claw2Pos);
            drive.placerPivot1.setPosition(ppPos);
            drive.placerPivot2.setPosition(ppPos);
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
            if (currentVoltage < 7) {
                for (int i = 0; i <= 10; i++) {
                    telemetry.addLine("STOP BATTERY ABUSE >:-(");
                }
                gamepad1.runRumbleEffect(batteryCritical);
                gamepad2.runRumbleEffect(batteryCritical);
            }
            ext_past = currentGamepad2.right_trigger;


            telemetry.update();

        }

        }

    private void initVision() {
        VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);


        // Create the AprilTag processor by using a builder.
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setCameraResolution(new Size(640,480))
//                .addProcessor(aprilTagProcessor)
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .build();
    }
    }


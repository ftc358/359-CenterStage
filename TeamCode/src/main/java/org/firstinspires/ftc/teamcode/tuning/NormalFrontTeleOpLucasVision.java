package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp
public class NormalFrontTeleOpLucasVision extends LinearOpMode {
    //Dashboard Vars
    public static double rapidTrigger_thr = 0.0727;
    public static double extension_sens = 0.727; //tune for extension sensitivity

    public static double flip_lift = 0.05;
    public static double flip_intake = 0.40;
    public static double flip_half = 0.30;
    public static double flip_dump = 0.6;

    public static double ppGet = 0;
    public static double ppHold = 0.21;
    public static double ppBoardDrop = 0.80;

    public static double claw1Grab = 0.38;
    public static double claw1Drop = 0;
    public static double claw2Grab = 0.45;
    public static double claw2Drop = 0;

    public static double liftIdle = 0.1;
    public static double liftDown = -1;

    public static double intake_Power = 0.6;

    public static double scoringDistance = 12;

    public static double GamePadControlSensitivity = 1.0;
    public static double DriveMaxSpeed = 1.0;


    //Servo Pos
    public double flipPos;
    public double claw1Pos = 0;
    public double claw2Pos = 0;
    public double ppPos = 0;


    //States
    public boolean transferTrigger = false;
    public boolean loaded = false;
    public boolean armed = false;
    public boolean armed2 = false;
    public boolean cycle = false;
    public boolean detActive = false;

    //Timers
    public int gravityMagic = 0;
    public int holdTimer = 0;
    public int interDropTimer = 0;
    public int postDropTimer = 0;

    //Looping Stuff
    public float ext_past;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public float ATag_X = 0;
    public float ATag_Y = 0;
    public float ATag_Z = 0;
    public float ATag_Yaw = 0;
    public boolean seeBoard = false;

    public VectorF RobotPos;




    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            initAprilTag();

            ext_past = 0;



            drive.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            waitForStart();
            while (opModeIsActive()) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -applyControlCurve(gamepad1.left_stick_y),
                                -applyControlCurve(gamepad1.left_stick_x)
                        ),
                        -applyControlCurve(gamepad1.right_stick_x)));
                drive.updatePoseEstimate();


                if (detActive){
                    visionPortal.resumeStreaming();
                }else{
                    visionPortal.stopStreaming();
                }



                //extension trigger
                if ((gamepad1.right_trigger >= 0.1)){
                    drive.ext1.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                    drive.ext2.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                }
                else{
                    drive.ext1.setPosition(0);
                    drive.ext2.setPosition(0);
                }

                //bucket flip stuff
                if (gamepad1.options){
                    flipPos = flip_dump;
                } //dumps
                else if (gamepad1.right_trigger == 0 && !gamepad1.options){
                    flipPos = flip_lift;
                }
                else if (((gamepad1.right_trigger - ext_past) > 0)&& !gamepad1.options){
                    //extension trigger
                    flipPos = flip_intake;
                }
                else if (((ext_past - gamepad1.right_trigger) > rapidTrigger_thr)&& !gamepad1.options){
                    //extension retract
                    flipPos = flip_half;
                };
                ext_past = gamepad1.right_trigger;

                //Intake Trigger
                if (gamepad1.right_trigger>0){
                    drive.intake.setPower(intake_Power); // \-0-/ WYSI
                } else if (gamepad1.right_bumper) {
                    drive.intake.setPower(-intake_Power);
                } else{
                    drive.intake.setPower(0);
                }

                //lift stuff
                if (gamepad1.left_trigger > 0){
                    drive.lift1.setPower(gamepad1.left_trigger);
                    drive.lift2.setPower(gamepad1.left_trigger);
                }else if (gamepad1.left_bumper){
                    drive.lift1.setPower(liftDown);
                    drive.lift2.setPower(liftDown);
                }else{
                    drive.lift1.setPower(0);
                    drive.lift2.setPower(0);
                } //Lift motors do not coordinate fix later



                //transfer
                if (!loaded && gamepad1.x){
                    ppPos = ppHold;
                    loaded = true;
                    detActive = true;                       //Activates AprilTag Pipeline
                }
                //Pause for Claw Lock
                else if (loaded && !cycle){
                    gravityMagic++;
                    if (gravityMagic > 10){
                        claw1Pos = claw1Grab;//0.38
                        claw2Pos = claw2Grab;//0.45
                        cycle = true;
                        gravityMagic = 0;
                    }
                }
                else if (loaded && cycle){
                    holdTimer++;
                    if ((holdTimer > 50)&&gamepad1.x) {
                        ppPos = ppBoardDrop;
                        armed = true;
                        armed2 = true;
                        loaded = false;
                    }
                }

                if (detActive && seeBoard){
                    if (gamepad1.dpad_down){
                        //Set Target Location
                        //

                        MecanumDrive score = new MecanumDrive(hardwareMap, new Pose2d(RobotPos.get(0), RobotPos.get(1), -ATag_Yaw));
                        Actions.runBlocking(
                                drive.actionBuilder(score.pose)
                                        .turnTo(0)
                                        .lineToY(scoringDistance)
                                        .build()
                        );
                    }



                }


                //First Drop
                if (!loaded && armed && armed2){
                    if (gamepad1.a) {//Drop First
                        claw2Pos = claw2Drop;
                        armed2 = false;
                    }
                    else if (gamepad1.b){//Drop Everything
                        claw2Pos = claw2Drop;
                        claw1Pos = claw1Drop;
                        armed2 = false;
                        armed = false;
                    }
                }

                //Second Drop
                else if (!loaded && armed && !armed2){
                    interDropTimer++;
                    if ((interDropTimer>30)&&gamepad1.a) {
                        claw1Pos = claw1Drop;
                        armed = false;
                        loaded = false;
                        interDropTimer = 0;
                    }
                }

                //Reset After Drop
                else if (!loaded && !armed && cycle){
                    postDropTimer++;
                    if (postDropTimer >40) {
                        ppPos = 0;
                        cycle = false;
                        postDropTimer = 0;
                        flipPos = flip_lift;
                        detActive = false;
                    }
                }




                //Actuate Servos
                drive.flip1.setPosition(flipPos);
                drive.flip2.setPosition(flipPos);
                drive.claw1.setPosition(claw1Pos);
                drive.claw2.setPosition(claw2Pos);
                drive.placerPivot1.setPosition(ppPos);
                drive.placerPivot2.setPosition(ppPos);


                //Debug

                //AprilTag:
                telemetryAprilTag();

                telemetry.addLine("Servos");
                telemetry.addData("flipPos", flipPos);
                telemetry.addData("clawPos1", claw1Pos);
                telemetry.addData("clawPos2", claw2Pos);
                telemetry.addData("ppPos", ppPos);

                telemetry.addLine("States");
                telemetry.addData("transfer",transferTrigger);
                telemetry.addData("armed",armed);
                telemetry.addData("armed2",armed2);
                telemetry.addData("loaded",loaded);
                telemetry.addData("cycle",cycle);

                telemetry.addLine("Time");
                telemetry.addData("gravity", gravityMagic);
                telemetry.addData("holdTimer",holdTimer);
                telemetry.addData("interDropWait", interDropTimer);
                telemetry.addData("postDropWait", postDropTimer);
                telemetry.update();

                telemetry.addLine("ATag");
                telemetry.addData("X", ATag_X);
                telemetry.addData("Y",ATag_Y);
                telemetry.addData("Z", ATag_Z);
                telemetry.addData("Yaw", ATag_Yaw);
                telemetry.update();

                sleep(5);
            }

        } else {
            throw new AssertionError();
        }

    }

    private double applyControlCurve(double input) {
        double output = GamePadControlSensitivity * Math.pow(input, 3) * DriveMaxSpeed;
        return output;
    }

    private void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(578.272,578.272,402.145,221.506)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "BoardCam"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("AprilTags Detected", currentDetections.size());
        

        for (AprilTagDetection detection : currentDetections){
            if (detection.id == 1 || detection.id == 2 || detection.id == 3 || detection.id == 4 || detection.id == 5 || detection.id == 6){
                seeBoard = true;
                if (detection.metadata != null){

                    ATag_X = (float) detection.ftcPose.x;
                    ATag_Y = (float) detection.ftcPose.y;
                    ATag_Z = (float) detection.ftcPose.z;
                    ATag_Yaw = (float) detection.ftcPose.yaw;

                    VectorF TagPos = new VectorF(ATag_X, ATag_Y,ATag_Z);
                    VectorF Pos = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;
                    RobotPos = Pos.subtracted(TagPos);




                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


                    //Necessary for Alignment: Y-> 5in or whatever distance until board, Yaw -> 0;
                    //Necessary for Board Detection: X,Y,Z, Localize Bottom Corners for Reference. Pitch: Correct OpenCV Board

                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }else{
                seeBoard = false;
            }


            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }




    }
}

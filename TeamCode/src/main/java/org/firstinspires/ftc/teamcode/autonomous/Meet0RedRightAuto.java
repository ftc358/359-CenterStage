package org.firstinspires.ftc.teamcode.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static java.lang.Math.toRadians;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.vision.RedDetectionRight;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous
public class Meet0RedRightAuto extends LinearOpMode {
    VisionPortal.Builder VPBuilder;
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    private AprilTagProcessor aprilTagProcessor;
    public RedDetectionRight detector = new RedDetectionRight(telemetry);
//    public BlueDetectBoardTest detector = new BlueDetectBoardTest(telemetry);
    public int color = 0;

    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, toRadians(270)));

        initVision();
        //drive.clawGrab();
        sleep(500);
        //drive.ppHold();
        sleep(500);
       // drive.extendZero();


//        drive.placerPivot1.setPosition(RoboConstants.ppGet);
//        drive.placerPivot2.setPosition(RoboConstants.ppGet);
//        sleep(500);
//        drive.bucketTransfer();
//        sleep(500);
//        drive.clawGrab();
//        sleep(500);
//        drive.placerPivot1.setPosition(0.1);
//        drive.placerPivot2.setPosition(0.1);
//        sleep(500);
//        drive.bucketVertical();
//        sleep(500);
//        drive.ppHold();
        while(opModeInInit() &&!isStarted()){
            color = RedDetectionRight.getReadout();

            telemetry.addData("Useless sees ",color);
            telemetry.addData("LeftValue",RedDetectionRight.leftValue);
            telemetry.addData("CenterValue",RedDetectionRight.centerValue);
            telemetry.addData("RightValue",RedDetectionRight.rightValue);
            telemetry.update();
        }


        waitForStart();

        if (color == 1) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(11, -33, toRadians(0)), 90)
                            .build()));

        } else if (color ==2){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(11,-36.5, toRadians(270)),0)
                            .build()));

        } else if (color == 3 || color == 0) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(15.5, -47, toRadians(-92)),0)
                            .build()));
        }
        //drive.ppGround();
        sleep(2000);
        //drive.dropOut();
        sleep(1000);
        //drive.ppBoard();

        if (color == 1) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(46, -23, toRadians(178)), toRadians(0))
                            .build()
            ));
        } else if (color ==2){
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(46, -32, toRadians(178)), toRadians(0))
                            .build()
            ));
        } else if (color == 3 || color == 0) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(46, -39, toRadians(178)), toRadians(0))
                            .build()
            ));
        }


        sleep(1000);
        //drive.dropAll();
        sleep(2000);
        //drive.bucketVertical();
        sleep(1000);
        //drive.ppZero();
        sleep(1000);
               runBlocking( new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(53,-55))
                                //.strafeTo(new Vector2d(-55,-35))

                                .build()
                ));


    }


    private void initVision() {
        // Create the AprilTag processor by using a builder.
        List myPortalsList;

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }




    @Override
    public void waitForStart() {
        super.waitForStart();
    }}

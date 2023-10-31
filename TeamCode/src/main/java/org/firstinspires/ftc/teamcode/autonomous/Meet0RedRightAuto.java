package org.firstinspires.ftc.teamcode.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static java.lang.Math.toRadians;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.RedDetectionRight;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "Meet 0 Autonomous Red Right",group = "Autonomous")
public class Meet0RedRightAuto extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    public RedDetectionRight detector = new RedDetectionRight(telemetry);
//    public BlueDetectBoardTest detector = new BlueDetectBoardTest(telemetry);
    public int color = 0;

    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, toRadians(90)));

        drive.extendZero();
        initVision();

        drive.bucketTransfer();
        sleep(500);
        drive.ppHold();
        sleep(500);
        drive.clawGrab();

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
                            .splineToLinearHeading(new Pose2d(7, -32, toRadians(0)), 90)
                            .build()));

        } else if (color ==2){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(12,-32.5, toRadians(270)),0)
                            .build()));

        } else if (color == 3 || color == 0) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(17.4, -34, toRadians(-92)),0)
                            .build()));
        }
        drive.ppGround();
        sleep(2000);
        drive.dropOut();
        sleep(1000);
        drive.ppBoard();

        if (color == 1) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49, -23.5, toRadians(178)), toRadians(0))
                            .build()
            ));
        } else if (color ==2){
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49, -32, toRadians(178)), toRadians(0))
                            .build()
            ));
        } else if (color == 3 || color == 0) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49, -37, toRadians(178)), toRadians(0))
                            .build()
            ));
        }


        //.strafeTo(new Vector2d(-60, -36))
                sleep(1000);
                drive.dropAll();
                sleep(1000);
                drive.bucketVertical();
                drive.ppZero();

               runBlocking( new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(53,-55))
                                .build()
                ));


    }


    private void initVision() {
        // Create the AprilTag processor by using a builder.
        visionPortal = new VisionPortal.Builder()
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

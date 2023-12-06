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

        drive.extendZero();
        drive.ppHold();
        sleep(500);
        drive.clawGrab();

        while(opModeInInit() &&!isStarted()){
            color = RedDetectionRight.getReadout();

            telemetry.addData("Me sees ",color);
            telemetry.addData("LeftValue",RedDetectionRight.leftValue);
            telemetry.addData("CenterValue",RedDetectionRight.centerValue);
            telemetry.addData("RightValue",RedDetectionRight.rightValue);
            telemetry.update();
        }


        waitForStart();
        if (color == 1 || color == 0) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(-50)
                            .splineToLinearHeading(new Pose2d(15, -32, toRadians(0)), 90)
                            .build()));

        } else if (color ==2){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(16.2,-36.1))
                            //.lineToYConstantHeading(-36.5)

                            .build()));

        } else if (color == 3 ){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(-50)
                            .splineToLinearHeading(new Pose2d(23.6, -47, toRadians(-92)),0)
                            .build()));
        }
        drive.ppGround();
        sleep(2000);
        drive.dropOut();
        sleep(1000);
        drive.ppBoard();

        if (color == 1 || color == 0) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49.1, -27, toRadians(180)), toRadians(0))
                            .build()
            ));
        } else if (color ==2){
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49.6, -33, toRadians(180)), toRadians(0))
                            .build()
            ));
        } else if (color == 3) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(48.7, -41.5, toRadians(180)), toRadians(0))
                            .build()
            ));
        }


        sleep(1000);
        drive.dropAll();
        sleep(2000);
        drive.bucketVertical();
        sleep(1000);
        drive.ppZero();
        sleep(1000);
               runBlocking( new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(53,-58))
                                .build()
                ));


    }


    private void initVision() {

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

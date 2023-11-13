package org.firstinspires.ftc.teamcode.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static java.lang.Math.toRadians;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BlueDetectionLeft;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@Autonomous(name = "Meet 0 Autonomous Blue Far Left",group = "Autonomous")
public class Meet0BlueFarLeftAuto extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
//    public RedDetectionRight detector = new RedDetectionRight(telemetry);
    public BlueDetectionLeft detector = new BlueDetectionLeft(telemetry);
    public int color = 0;

    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 60, toRadians(270)));

        drive.extendZero();
        initVision();

        drive.bucketTransfer();
        sleep(500);
        drive.ppHold();
        sleep(500);
        drive.clawGrab();

        while(opModeInInit() &&!isStarted()){
            color = BlueDetectionLeft.getReadout();

            telemetry.addData("Useless sees ",color);
            telemetry.addData("LeftValue",BlueDetectionLeft.leftValue);
            telemetry.addData("CenterValue",BlueDetectionLeft.centerValue);
            telemetry.addData("RightValue",BlueDetectionLeft.rightValue);
            telemetry.update();
        }


        waitForStart();

        if (color == 1) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(17-48, 44, toRadians(90)),0)
                            .build()));

        } else if (color ==2){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(12-48,32, toRadians(90)),toRadians(300))
                            .build()));

        } else if (color == 3 || color == 0) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(8-48, 33.5, toRadians(0)),toRadians(200))
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
                            .strafeTo(new Vector2d(-38,0))
                            .splineToLinearHeading(new Pose2d(47, 38,Math.toRadians(180)), Math.toRadians(0))
                            .build()
            ));
        } else if (color ==2){
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(-38,0))
                            .splineToLinearHeading(new Pose2d(47, 34,Math.toRadians(180)), Math.toRadians(0))
                            .build()
            ));
        } else if (color == 3 || color == 0) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(-38,0))
                            .splineToLinearHeading(new Pose2d(47, 27,Math.toRadians(180)), Math.toRadians(0))
                            .build()
            ));
        }


        //.strafeTo(new Vector2d(-60, -36))
        sleep(1000);
        drive.dropAll();
        sleep(1000);
        drive.bucketVertical();
        sleep(1000);
        drive.ppZero();
        sleep(1000);
        runBlocking( new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(53,55))
                        //.strafeTo(new Vector2d(-55,37))
                        .build()
        ));
//

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



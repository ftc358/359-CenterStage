package org.firstinspires.ftc.teamcode.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static java.lang.Math.toRadians;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BlueDetectionLeft;
import org.firstinspires.ftc.teamcode.vision.RedDetectionRight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous
public class Meet0BlueLeftAuto extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagProcessor aprilTagProcessor;
    public BlueDetectionLeft detector = new BlueDetectionLeft(telemetry);
    public int color = 0;

    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, 60, toRadians(90)));

        initVision();

        drive.clawGrab();
        sleep(500);
        drive.ppHold();
       sleep(500);
       drive.extendZero();

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
                            .lineToYConstantHeading(50)
                            .splineToLinearHeading(new Pose2d(23, 46, toRadians(90)),0)
                            .build()));

        } else if (color ==2 || color == 0){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(16.3,36.1))
                            .build()));

        } else if (color == 3) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(50)
                            .splineToLinearHeading(new Pose2d(14, 33.4, toRadians(0)), 90)
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
                            .splineToLinearHeading(new Pose2d(46, 35.5, Math.toRadians(180)), Math.toRadians(4))
                            .build()
            ));
        } else if (color ==2|| color == 0){
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(46.9, 33,Math.toRadians(180)), Math.toRadians(0))
                            .build()
            ));
        } else if (color == 3 ) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(46.6, 26,Math.toRadians(180)), Math.toRadians(0))
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
        aprilTag = new AprilTagProcessor.Builder()
        //        <Calibration
        //size="640 480"
//        focalLength="622.001f, 622.001f"
//        principalPoint="319.803f, 241.251f"
//        distortionCoefficients="0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0"
//                />
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(detector,aprilTag)
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }




    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()




    @Override
    public void waitForStart() {
        super.waitForStart();
    }}



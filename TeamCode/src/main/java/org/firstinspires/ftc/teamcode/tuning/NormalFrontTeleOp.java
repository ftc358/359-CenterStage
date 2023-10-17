package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class NormalFrontTeleOp extends LinearOpMode {
    public static double flip_zer = 0;
    public static double flip_one = 0.15;
    public static double flip_two = 0.3;

    public float ext_past;
    public float rapidTrigger_thr = 0.0727f;
    public float extension_sens = 0.727f; //tune for extension sensitivity

    public float dead_zone_prelim = 0.1f;
    public float dead_zone_second = 0.3f;


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;


    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


//            initAprilTag();

            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();



            ext_past = 0;

            waitForStart();
//
//            webcam1 = hardwareMap.get(WebcamName.class, "webcam1"); //This is duplicated in comp. to init(). Please dont crash.
//            webcam2 = hardwareMap.get(WebcamName.class, "webcam2");

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);

                //extension stuff
                if ((gamepad1.right_trigger >= 0.1)){
                    drive.ext1.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                    drive.ext2.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                }
                else{
                    drive.ext1.setPosition(0);
                    drive.ext2.setPosition(0);
                }

                //bucket flip stuff
                if (gamepad1.right_trigger <= dead_zone_prelim){
                    drive.flip1.setPosition(flip_zer);
                    drive.flip2.setPosition(flip_zer);
                }

                else if (gamepad1.right_trigger <= dead_zone_second){
                    drive.flip1.setPosition(flip_two);
                    drive.flip2.setPosition(flip_two);
                }


                else if ((gamepad1.right_trigger - ext_past) > 0){
                    //extension trigger
                    drive.flip1.setPosition(flip_one);
                    drive.flip2.setPosition(flip_one);
                }
                else if ((ext_past - gamepad1.right_trigger) > rapidTrigger_thr){
                    //extension retract
                    drive.flip1.setPosition(flip_zer);
                    drive.flip2.setPosition(flip_zer);
                }
                ext_past = gamepad1.right_trigger;

                if (gamepad1.right_trigger>0){
                    drive.intake.setPower(0.8);
                } else if (gamepad1.right_bumper) {
                    drive.intake.setPower(-1);
                } else{
                    drive.intake.setPower(0);
                }




                telemetry.update();
                sleep(5);
            }
        } else {
            throw new AssertionError();
        }
    }


//
//    private void initAprilTag() {
//
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        webcam1 = hardwareMap.get(WebcamName.class, "webcam1");
//        webcam2 = hardwareMap.get(WebcamName.class, "webcam2");
//
//
//
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
//
//        // Create the vision portal by using a builder.
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(switchableCamera)
//                .addProcessor(aprilTag)
//                .build();
//
//    }   // end method initAprilTag()




//    private void telemetryCameraSwitching() {
//        if (visionPortal.getActiveCamera().equals(webcam1)) {
//            telemetry.addData("activeCamera", "Webcam 1");
//            telemetry.addData("Press RightBumper", "to switch to Webcam 2");
//        } else {
//            telemetry.addData("activeCamera", "Webcam 2");
//            telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
//        }
//
//    }   // end method telemetryCameraSwitching()


//
//
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }   // end method telemetryAprilTag()

//
//    private void doCameraSwitching() {
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            // If the left bumper is pressed, use Webcam 1.
//            // If the right bumper is pressed, use Webcam 2.
//            boolean newLeftBumper = gamepad1.left_bumper;
//            boolean newRightBumper = gamepad1.right_bumper;
//            if (newLeftBumper && !oldLeftBumper) {
//                visionPortal.setActiveCamera(webcam1);
//            } else if (newRightBumper && !oldRightBumper) {
//                visionPortal.setActiveCamera(webcam2);
//            }
//            oldLeftBumper = newLeftBumper;
//            oldRightBumper = newRightBumper;
//        }
//
//    }




    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1,"BlueAllianceLeft",
                        2,new VectorF(60.25f, 41.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(2,"BlueAllianceCenter",
                        2,new VectorF(60.25f, 35.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(3,"BlueAllianceRight",
                        2,new VectorF(60.25f, 29.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(4,"BlueAllianceLeft",
                        2,new VectorF(60.25f, -29.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(5,"BlueAllianceCenter",
                        2,new VectorF(60.25f, -35.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(6,"BlueAllianceRight",
                        2,new VectorF(60.25f, -41.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(7,"RedAudienceWallLarge",
                        2,new VectorF(-70.25f, -40.625f,5.5f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(8,"RedAudienceWallSmall",
                        2,new VectorF(-70.25f, -35.125f,4f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(9,"BlueAudienceWallSmall",
                        2,new VectorF(-70.25f, 35.125f,4f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(10,"BlueAudienceWallLarge",
                        2,new VectorF(-70.25f, 40.625f,5.5f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .build();

    }
}


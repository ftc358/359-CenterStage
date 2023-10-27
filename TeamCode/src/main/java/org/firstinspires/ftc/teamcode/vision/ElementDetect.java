//package org.firstinspires.ftc.teamcode.tuning;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//import org.
//
//@Autonomous(name="Skystone Detecotor", group="AutoTest")
//public class SkystoneAutoMode extends LinearOpMode {
//    OpenCvCamera phoneCam;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext
//                .getResources().getIdentifier("cameraMonitorViewId",
//                        "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance()
//                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        AutoBlueDetect detector = new AutoBlueDetect(telemetry);
//        phoneCam.setPipeline(detector);
//        phoneCam.openCameraDeviceAsync(
//                () -> phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
//        );
//
//        waitForStart();
//        switch (detector.getLocation()) {
//            case LEFT:
//                // ...
//                break;
//            case RIGHT:
//                // ...
//                break;
//            case CENTER:
//                // ..
//                break;
//            case DNE:
//                break;
//        }
//        phoneCam.stopStreaming();
//    }
//}
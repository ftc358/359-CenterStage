package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp
public class NormalFrontTeleOp extends LinearOpMode {
    public static double flip_zer = 0;
    public static double flip_one = 0.15;
    public static double flip_two = 0.3;

    public float ext_past;
    public float rapidTrigger_thr = 0.0727f;
    public float extension_sens = 0.727f; //tune for extension sensitivity

    public float dead_zone_prelim = 0.1f;
    public float dead_zone_second = 0.3f;


    public static float rapidTrigger_thr = 0.0727f;
    public static float extension_sens = 0.727f; //tune for extension sensitivity
    public static double flip_lift = 0;
    public static double flip_intake = 0.25;
    public static double flip_half = 0.17;
    public static double flip_dump = 0.5;

    public float ext_past;

    public static double sensitivity = 1.0;
    public static double maxSpeed = 1.0;




    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            ext_past = 0;

            waitForStart();
            while (opModeIsActive()) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -applyControlCurve(gamepad1.left_stick_y),
                                -applyControlCurve(gamepad1.left_stick_x)
                        ),
                        -applyControlCurve(gamepad1.right_stick_x)));

                drive.updatePoseEstimate();

//                telemetry.addData("x", drive.pose.position.x);
//                telemetry.addData("y", drive.pose.position.y);
//                telemetry.addData("heading", drive.pose.heading);

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




                //extension trigger
                if ((gamepad1.right_trigger >= 0.1)){
                    drive.ext1.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                    drive.ext2.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                }
                else{
                    drive.ext1.setPosition(0);
                    drive.ext2.setPosition(0);
                }

                if (gamepad1.options){
                    drive.flip1.setPosition(flip_dump);
                    drive.flip2.setPosition(flip_dump);
                }
                //bucket flip stuff
                else if (gamepad1.right_trigger == 0 && !gamepad1.options){
                    drive.flip1.setPosition(flip_lift);
                    drive.flip2.setPosition(flip_lift);
                }
                else if (((gamepad1.right_trigger - ext_past) > 0)&& !gamepad1.options){
                    //extension trigger
                    drive.flip1.setPosition(flip_intake);
                    drive.flip2.setPosition(flip_intake);
                }
                else if (((ext_past - gamepad1.right_trigger) > rapidTrigger_thr)&& !gamepad1.options){
                    //extension retract
                    drive.flip1.setPosition(flip_half);
                    drive.flip2.setPosition(flip_half);
                };
                ext_past = gamepad1.right_trigger;

                if (gamepad1.right_trigger>0){
                    drive.intake.setPower(0.8);
                } else if (gamepad1.right_bumper) {
                    drive.intake.setPower(-1);
                } else{
                    drive.intake.setPower(0);
                }

                telemetry.update();


                drive.flip1.setPosition(gamepad1.left_trigger);
                drive.flip2.setPosition(gamepad1.left_trigger);

            }




        } else {
            throw new AssertionError();
        }
    }

    private double applyControlCurve(double input) {
        // Apply sensitivity and ensure the output is between -maxSpeed and maxSpeed
        double output = sensitivity * Math.pow(input, 3) * maxSpeed;


        return output;
    }


}


package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp
public class NormalFrontTeleOp extends LinearOpMode {


    public static float rapidTrigger_thr = 0.0727f;
    public static float extension_sens = 0.727f; //tune for extension sensitivity

    public static double flip_lift = 0.05;
    public static double flip_intake = 0.27;
    public static double flip_half = 0.20;
    public static double flip_dump = 0.5;

    public static double claw1Grab = 0;
    public static double claw1Drop = 0.2;
    public static double claw2Grab = 0;
    public static double claw2Drop = 0.2;

    public static double ppGet = 0.1;
    public static double ppPut = 0.5;
    public static double ppGround = 1.0;

    public static double liftIdle = 0.1;
    public static double liftDown = -0.5;

    public float ext_past;
    public boolean liftHomed;

    public static double GamePadControlSensitivity = 1.0;
    public static double DriveMaxSpeed = 1.0;

    //flags
    public boolean transferTrigger = false;



    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            ext_past = 0;

            //lift homing
            ///INSERT HOMING CODE
            drive.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            liftHomed = true;


            waitForStart();
            while (opModeIsActive()) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -applyControlCurve(gamepad1.left_stick_y),
                                -applyControlCurve(gamepad1.left_stick_x)
                        ),
                        -applyControlCurve(gamepad1.right_stick_x)));

                drive.updatePoseEstimate();

                //extension trigger
                if ((gamepad1.right_trigger >= 0.1)){
                    drive.ext1.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                    drive.ext2.setPosition((1.0 - extension_sens + gamepad1.right_trigger*extension_sens)*0.359);
                    //lift down

                    //home lift
                    ///INSERT HOMING CODE
                    //lift goes back to
                    liftHomed = true;

                }
                else{
                    drive.ext1.setPosition(0);
                    drive.ext2.setPosition(0);
                }

                //bucket flip stuff
                if (gamepad1.options){
                    drive.flip1.setPosition(flip_dump);
                    drive.flip2.setPosition(flip_dump);
                }
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
                }

                //transfer
                if (gamepad1.y == true && transferTrigger == false){
                    drive.flip1.setPosition(0);
                    drive.flip2.setPosition(0);
                    //drive.placerPivot1.setPosition(0.5);
                    //drive.placerPivot2.setPosition(0.5);
                    transferTrigger = true;
                }
                else if (gamepad1.x = true && transferTrigger == true){
                    //drive.claw1.setPosition(0.2);
                    //drive.claw1.setPosition(0.2);
                    transferTrigger = false;
                }





                //Intake Trigger
                if (gamepad1.right_trigger>0){
                    drive.intake.setPower(0.8);
                } else if (gamepad1.right_bumper) {
                    drive.intake.setPower(-1);
                } else{
                    drive.intake.setPower(0);
                }

                telemetry.update();
            }

        } else {
            throw new AssertionError();
        }

    }

    private double applyControlCurve(double input) {
        double output = GamePadControlSensitivity * Math.pow(input, 3) * DriveMaxSpeed;
        return output;
    }

}

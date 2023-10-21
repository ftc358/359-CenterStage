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
public class NormalFrontTeleOpLucas extends LinearOpMode {

    //Dashboard Vars
    public static float rapidTrigger_thr = 0.0727f;
    public static float extension_sens = 0.727f; //tune for extension sensitivity

    public static double flip_lift = 0.05;
    public static double flip_intake = 0.40;
    public static double flip_half = 0.30;
    public static double flip_dump = 0.6;

    public static double ppGet = 0;
    public static double ppHold = 0.21;
    public static double ppBoardDrop = 0.80;

    public static double liftIdle = 0.1;
    public static double liftDown = -1;

    public float ext_past;

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

    //Timers
    public int gravityMagic = 0;
    public int holdTimer = 0;
    public int interDropTimer = 0;
    public int postDropTimer = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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
                    ppPos = 0.21;
                    loaded = true;
                }
                //Pause for Claw Lock
                else if (loaded && !cycle){
                    gravityMagic++;
                    if (gravityMagic > 10){
                        claw1Pos = 0.38;
                        claw2Pos = 0.45;
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


                //First Drop
                if (!loaded && armed && armed2){
                    if (gamepad1.a) {//Drop First
                        claw2Pos = 0;
                        armed2 = false;
                    }
                    if (gamepad1.b){//Drop Everything
                        claw2Pos = 0;
                        claw1Pos = 0;
                        armed2 = false;
                        armed = false;
                    }
                }

                //Second Drop
                else if (!loaded && armed && !armed2){
                    interDropTimer++;
                    if ((interDropTimer>30)&&gamepad1.a) {
                        claw1Pos = 0;
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
                        flipPos = 0.05;
                    }
                }




                //Intake Trigger
                if (gamepad1.right_trigger>0){
                    drive.intake.setPower(0.727); // \-0-/ WYSI
                } else if (gamepad1.right_bumper) {
                    drive.intake.setPower(-1);
                } else{
                    drive.intake.setPower(0);
                }


                //Actuate Servos
                drive.flip1.setPosition(flipPos);
                drive.flip2.setPosition(flipPos);
                drive.claw1.setPosition(claw1Pos);
                drive.claw2.setPosition(claw2Pos);
                drive.placerPivot1.setPosition(ppPos);
                drive.placerPivot2.setPosition(ppPos);


                //Debug
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

}

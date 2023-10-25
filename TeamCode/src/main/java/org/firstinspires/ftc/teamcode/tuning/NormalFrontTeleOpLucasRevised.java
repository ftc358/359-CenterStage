package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp
public class NormalFrontTeleOpLucasRevised extends LinearOpMode {

    //Dashboard Vars
    public static float rapidTrigger_thr = 0.0727f;
    public static float extension_sens = 0.727f; //tune for extension sensitivity

    public static double flip_lift = 0.05;
    public static double flip_intake = 0.40;
    public static double flip_half = 0.30;
    public static double flip_dump = 0.6;
    public static double flip_vert = 0.15;

    public static double ppGet = 0;
    public static double ppHold = 0.21;
    public static double ppBoardDrop = 0.80;

    public static double liftIdle = 0.1;
    public static double liftDown = -1;



    public static double GamePadControlSensitivity = 1.0;
    public static double DriveMaxSpeed = 1.0;

    public static double claw1Grab = 0.38;
    public static double claw2Grab = 0.45;



    //Servo Pos
    double flipPos;
    double claw1Pos = 0;
    public double claw2Pos = 0;
    public double ppPos = 0;

    //States
    boolean bucketPrime = false;
    boolean loadPP = false;
    boolean rd2drop = false;
    boolean drop1 = false;
    boolean drop2 = false;
    boolean cycleFinished = false;



    boolean gp1ANow = false; //debounce for repeated button
    boolean gp2ALast = false;;
    boolean dBouncegp1a = false;
    float ext_past;


    //PS5 Rumble
    Gamepad.RumbleEffect batteryCritical, drop1Ready, drop2Ready;

    double straight = 0;
    double turn = 0;
    double strafe = 0;

    double currentVoltage = 16;
    double lowestVoltage = 11;



    @Override
    public void runOpMode() throws InterruptedException {
        batteryCritical = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0,1.0,1000)
                .addStep(0,0,1000)
                .build();

        drop1Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.3,0.3,200)
                .addStep(0,0,200)
                .addStep(0.7,0.7,200)
                .addStep(0,0,200)
                .build();
        drop2Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.3,0.3,100)
                .addStep(0,0,100)
                .addStep(0.7,0.7,100)
                .addStep(0,0,100)
                .build();



            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            ext_past = 0;

            drive.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            waitForStart();
            while (opModeIsActive()) {
                currentVoltage = drive.voltageSensor.getVoltage();
                if (currentVoltage<lowestVoltage){
                    lowestVoltage = currentVoltage;
                }
                if (lowestVoltage<10){
                    gamepad1.runRumbleEffect(batteryCritical);
                    gamepad2.runRumbleEffect(batteryCritical);
                }



                if (gamepad1.left_stick_y<-0.1){
                    straight = -applyControlCurve(gamepad1.left_stick_y) +0.1;
                }
                else if (gamepad1.left_stick_y>0.1){
                    straight = -applyControlCurve(gamepad1.left_stick_y) -0.1;
                } else{
                    straight = 0;
                }

                if (gamepad1.left_stick_x<-0.1){
                    strafe = -applyControlCurve(gamepad1.left_stick_x) +0.2;
                }
                else if (gamepad1.left_stick_x>0.1){
                    strafe = -applyControlCurve(gamepad1.left_stick_x) -0.2;
                } else{
                    strafe = 0;
                }

                if (gamepad1.right_stick_x<0){
                    turn = -applyStrongerControlCurve(gamepad1.right_stick_x) +0.11;
                }
                else if (gamepad1.right_stick_x>0){
                    turn = -applyStrongerControlCurve(gamepad1.right_stick_x) -0.11;
                } else{
                    turn = 0;
                }
//                +0.1
//                +0.2
//                +0.11


                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                straight,
                                strafe
                        ),
                                turn));

                drive.updatePoseEstimate();


                //Intake Trigger
                if (gamepad2.right_trigger>0){
                    drive.intake.setPower(0.727); // \-0-/ WYSI
                } else if (gamepad2.right_bumper) {
                    drive.intake.setPower(-1);
                } else{
                    drive.intake.setPower(0);
                }


                //extension trigger
                if ((gamepad2.right_trigger >= 0.1)){
                    drive.ext1.setPosition((1.0 - extension_sens + gamepad2.right_trigger*extension_sens)*0.359);
                    drive.ext2.setPosition((1.0 - extension_sens + gamepad2.right_trigger*extension_sens)*0.359);
                }
                else{
                    drive.ext1.setPosition(0);
                    drive.ext2.setPosition(0);
                }

                //bucket flip stuff
                if (gamepad2.options&&!gamepad2.x){
                    flipPos = flip_dump;
                } //dumps
                else if (gamepad2.right_trigger == 0 && !gamepad2.options &&!gamepad2.x){
                    flipPos = flip_vert;
                }
                else if (((gamepad2.right_trigger - ext_past) > 0)&& !gamepad2.options &&!gamepad2.x){
                    //extension trigger
                    flipPos = flip_intake;
                }
                else if (((ext_past - gamepad2.right_trigger) > rapidTrigger_thr)&& !gamepad2.options &&!gamepad2.x){
                    //extension retract
                    flipPos = flip_half;
                };



                //lift stuff
                if (gamepad2.left_trigger > 0){
                    drive.lift1.setPower(1);
                    drive.lift2.setPower(1);
                }else if (gamepad2.left_bumper){
                    drive.lift1.setPower(liftDown);
                    drive.lift2.setPower(liftDown);
                }else{
                    drive.lift1.setPower(0);
                    drive.lift2.setPower(0);
                } //Lift motors do not  coordinate fix later



                //transfer
                gp1ANow = gamepad1.a;
                //Debouncing for Gamepad
                if (gp1ANow && !gp2ALast) {
                    dBouncegp1a = true;
                } else {
                    dBouncegp1a = false;
                }

                if (gamepad2.x && !bucketPrime){
                    bucketPrime = true; //Move this to below if jake wants on one key, add timer if necc.
                } else if (gamepad2.y && bucketPrime){
                    loadPP = true;
                } else if (gamepad2.b && loadPP){
                    rd2drop = true;
                } else if (dBouncegp1a && rd2drop){ //could switch this out for gamepad1.a and gamepad1.b if needed.
                    drop1 = true;
                } else if (dBouncegp1a && drop1){
                    drop2 = true;
                }
                else if (gamepad2.dpad_left) { //Reset: Drops both
                    drop1 = true;
                    drop2 = true;
                }else if (cycleFinished){
                    bucketPrime = false;
                    loadPP = false;
                    rd2drop = false;
                    drop1 = false;
                    drop2 = false;
                    cycleFinished = false;
                }



                if (bucketPrime){
                    ppPos = 0;
                    flipPos = flip_lift;
                }
                if (loadPP){
                    ppPos = ppHold;
                    claw1Pos = claw1Grab;//No Pause for Claw lock, add timer if issue
                    claw2Pos = claw2Grab;
                }
                if (rd2drop){
                    flipPos = flip_vert;
                    ppPos = ppBoardDrop;
                    if (!drop1) {
                        gamepad1.runRumbleEffect(drop1Ready);
                    }
                }
                if (drop1){
                    claw2Pos = 0;
                    if (!drop2) {
                        gamepad2.runRumbleEffect(drop2Ready);
                    }
                }
                if (drop2){
                    claw1Pos = 0;
                    ppPos = ppGet;
                    cycleFinished = true;
                }


                //Actuate Servos
                drive.flip1.setPosition(flipPos);
                drive.flip2.setPosition(flipPos);
                drive.claw1.setPosition(claw1Pos);
                drive.claw2.setPosition(claw2Pos);
                drive.placerPivot1.setPosition(ppPos);
                drive.placerPivot2.setPosition(ppPos);


                //Debug
                telemetry.addLine("Drive");
                telemetry.addData("Straight",straight);
                telemetry.addData("Strafe", strafe);
                telemetry.addData("Turn",turn);


//                telemetry.addLine("Servos");
//                telemetry.addData("flipPos", flipPos);
//                telemetry.addData("clawPos1", claw1Pos);
//                telemetry.addData("clawPos2", claw2Pos);
//                telemetry.addData("ppPos", ppPos);

                telemetry.addLine("States In Order");
                telemetry.addData("BucketPrime",bucketPrime);
                telemetry.addData("loadPP",loadPP);
                telemetry.addData("rd2drop",rd2drop);
                telemetry.addData("drop1",drop1);
                telemetry.addData("drop2", drop2);
                telemetry.addData("cycleFinished",cycleFinished);

                telemetry.update();


                //Variables updated per run;
                gp2ALast = gp1ANow;// debounce
                ext_past = gamepad2.right_trigger;
            }



    }

    private double applyControlCurve(double input) {
        double output = GamePadControlSensitivity * Math.pow(input, 3) * DriveMaxSpeed;
        return output;
    }

    private double applyStrongerControlCurve(double input) {
        double output = GamePadControlSensitivity * Math.pow(input, 5) * DriveMaxSpeed;
        return output;
    }

}

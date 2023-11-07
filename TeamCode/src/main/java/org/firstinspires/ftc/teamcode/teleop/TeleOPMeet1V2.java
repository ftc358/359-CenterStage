package org.firstinspires.ftc.teamcode.teleop;

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
@TeleOp (name = "V2 Meet 1 TeleOp",group = "TeleOP")
public class TeleOPMeet1V2 extends LinearOpMode {
 // saved on codeshare.io/lmaocopeharderpeopleofpaper727


    //Dashboard Vars
    public static float rapidTrigger_thr = 0.1f;
    public static float extension_sens = 0.727f; //tune for extension sensitivity

    public static double flip_lift = 0.07;
    public static double flip_intake = 0.40;
    public static double flip_half = 0.30;
    public static double flip_dump = 0.6;
    public static double flip_vert = 0.15;

    public static double ppGet = 0;
    public static double ppHold = 0.21;
    public static double ppBoardDrop = 0.77;

    public static double liftIdle = 0.001;
    public static double liftDown = -1;

    public static double intakeSpeed = 0.727;


    public static double GamePadControlSensitivity = 1.0;
    public static double DriveMaxSpeed = 1.0;

    public static double claw1Grab = 0.38;
    public static double claw2Grab = 0.47;


    public static double ax1_home = 0.0;
    public static double ax2_home = 0.0;
    public static double ax1_limit = 1.0;
    public static double ax2_limit = 1.0;



    //Servo Pos
    double flipPos;
    double claw1Pos = 0;
    public double claw2Pos = 0;
    public double ppPos = 0;

    //States
    int state = 0;

    boolean bucketPrime = false;
    boolean loadPP = false;
    boolean flingup = false;
    boolean dropOnBoard = false;
    boolean dropOnGround = false;
    boolean drop1 = false;
    boolean drop2 = false;
    boolean cycleFinished = false;
    boolean afterDrop = false;


    float ext_past;


    //PS5 Rumble
    Gamepad.RumbleEffect batteryCritical, drop1Ready, drop2Ready;

    double straight = 0;
    double turn = 0;
    double strafe = 0;

    double currentVoltage = 16;
    double lowestVoltage = 11;




    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void runOpMode() throws InterruptedException {
        batteryCritical = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0,1.0,1000)
                .build();

        drop1Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7,0.7,200)
                .addStep(0,0,20)
                .build();
        drop2Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7,0.7,100)
                .addStep(0,0,100)
                .build();


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ext_past = 0;

        drive.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something

        if (currentVoltage<12) {
            telemetry.addLine("Current Voltage is Below 12v. Please replace battery before match");
            telemetry.update();
        }


        waitForStart();
        int bucketState = 0;
        int transferState = 0;
        while (opModeIsActive()) {
            //trust me bro - jonathan



            //Gamepad States
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //C
            currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something




                //Curving Controls.
            if (gamepad1.left_stick_y<-0.1){
                straight = -applyStrongerControlCurve(gamepad1.left_stick_y) +0.26;
            } else if (gamepad1.left_stick_y>0.1){
                straight = -applyStrongerControlCurve(gamepad1.left_stick_y) -0.26;
            } else{
                straight = 0;
            }

            if (gamepad1.left_stick_x<-0.1){
                strafe = -applyStrongerControlCurve(gamepad1.left_stick_x) +0.26;
            } else if (gamepad1.left_stick_x>0.1){
                strafe = -applyStrongerControlCurve(gamepad1.left_stick_x) -0.26;
            } else{
                strafe = 0;
            }

            if (gamepad1.right_stick_x<0){
                turn = -applyStrongerControlCurve(gamepad1.right_stick_x) +0.26;
            } else if (gamepad1.right_stick_x>0){
                turn = -applyStrongerControlCurve(gamepad1.right_stick_x) -0.26;
            } else{
                turn = 0;
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight,strafe),turn));
            drive.updatePoseEstimate();


            //Intake Trigger
            if (currentGamepad2.left_trigger>0 && !currentGamepad2.dpad_left){
                drive.intake.setPower(intakeSpeed);
            } else if (currentGamepad2.left_bumper || currentGamepad2.dpad_left) {
                drive.intake.setPower(-1);
            } else{
                drive.intake.setPower(0);
            }


            //Extension Trigger
            if ((currentGamepad2.left_trigger >= 0.1)){
                drive.ext1.setPosition((1.0 - extension_sens + currentGamepad2.left_trigger*extension_sens)*0.5);
                drive.ext2.setPosition((1.0 - extension_sens + currentGamepad2.left_trigger*extension_sens)*0.5);
            } else{
                drive.ext1.setPosition(0);
                drive.ext2.setPosition(0);
            }


            //Bucket Flip
            if (currentGamepad2.dpad_left){
                flipPos = flip_dump;
            } else if (currentGamepad2.left_trigger == 0 && !currentGamepad2.dpad_left){
                flipPos = flip_vert;
            } else if (((gamepad2.left_trigger - ext_past) > 0)&& !gamepad2.dpad_left){
                //extension trigger
                flipPos = flip_intake;
            } else if (((ext_past - gamepad2.left_trigger) > rapidTrigger_thr)&& !gamepad2.dpad_left){
                //extension retract
                flipPos = flip_half;
            }

            //Lift
            if (currentGamepad2.right_trigger > 0){
                drive.lift1.setPower(currentGamepad2.right_trigger);
                drive.lift2.setPower(currentGamepad2.right_trigger);
            }else if (currentGamepad2.right_bumper){
                drive.lift1.setPower(liftDown);
                drive.lift2.setPower(liftDown);
            }else{
                drive.lift1.setPower(liftIdle);
                drive.lift2.setPower(liftIdle);
            } //This idle power overheats the motor. Caution.

            //transfer stuff


            if ((currentGamepad2.x && !previousGamepad2.x) && !bucketPrime){
                bucketPrime = true;
            }
            else if ((currentGamepad2.x && !previousGamepad2.x) && bucketPrime &&!loadPP){
                loadPP = true;
            }
            else if ((currentGamepad2.x && !previousGamepad2.x) && loadPP &&!flingup){
                flingup = true;
            }
            else if ((currentGamepad2.y && !previousGamepad2.y) && flingup &&!dropOnGround){
                dropOnGround = true;
                dropOnBoard = false;
            }
            else if ((currentGamepad2.x && !previousGamepad2.x) && flingup &&!dropOnBoard){
                dropOnBoard = true;
                dropOnGround = false;
            }
            else if ((currentGamepad1.b && !previousGamepad1.b) && (dropOnBoard||dropOnGround) &&!drop1){ //Drop the Outermost Pixel
                drop1 = true;
            }
            else if ((currentGamepad1.b && !previousGamepad1.b) && drop1 &&!drop2){ //Drop the Innermost Pixel
                drop2 = true;
            }
            else if (gamepad2.a) {
                drop1 = true;
                drop2 = true;
                afterDrop = true;
            }
            else if ((currentGamepad1.b && !previousGamepad1.b)&&drop2&&!afterDrop){
                afterDrop = true;
            }


            //transfer stuff end



            if (cycleFinished){
                bucketPrime = false;
                flingup = false;
                loadPP = false;
                dropOnBoard = false;
                drop1 = false;
                drop2 = false;
                afterDrop = false;
                cycleFinished = false;
            }

            if (bucketPrime){
                ppPos = 0;
                flipPos = flip_lift;
            }
            if (loadPP){
                ppPos = 0.1;

                //flipPos = flip_lift;
                //You probably wanna either kill the servo or make it retract more here
                //Time the recession of the flip bucket to coordinate with the rotational x component of pp.
            }
            if (flingup){
                ppPos = ppHold;
            }
            if (dropOnGround){
                ppPos = 1.0;
                claw1Pos = claw1Grab;//No Pause for Claw lock, add timer if issue
                claw2Pos = claw2Grab;
                flipPos = flip_vert;
                if (!drop1) {
                    gamepad1.runRumbleEffect(drop1Ready);
                }
            }
            if (dropOnBoard){
                claw1Pos = claw1Grab;//No Pause for Claw lock, add timer if issue
                claw2Pos = claw2Grab;
                flipPos = flip_vert;
                ppPos = ppBoardDrop;
                if (!drop1) {
                    gamepad1.runRumbleEffect(drop1Ready);
                }
            }
            if (drop1){
                claw2Pos = 0;
                if (!drop2) {
                    gamepad1.runRumbleEffect(drop2Ready);
                }
            }
            if (drop2){
                claw1Pos = 0;
            }
            if (afterDrop) {
                ppPos = ppGet;
                flipPos = flip_vert;
                cycleFinished = true;
            }

            //Attack Plan R
            if (currentGamepad1.options){
                drive.planeRelease.setPosition(1.0);
            }else{
                drive.planeRelease.setPosition(0);
            }



            //Unified Servo Actuation
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
//                telemetry.addData("gp1b current",currentGamepad1.b);
//                telemetry.addData("gp1b prev",previousGamepad1.b);
//                telemetry.addData("gp2x current",currentGamepad2.x);
//                telemetry.addData("gp2x current",currentGamepad2.x);
//
//                telemetry.addData("BucketPrime",bucketPrime);
//                telemetry.addData("loadPP",loadPP);
//                telemetry.addData("rd2drop",rd2drop);
//                telemetry.addData("drop1",drop1);
//                telemetry.addData("drop2", drop2);
//                telemetry.addData("afterDrop",afterDrop);
//                telemetry.addData("cycleFinished",cycleFinished);
            if (currentVoltage<7) {
                for (int i = 0; i<=10; i++) {telemetry.addLine("STOP BATTERY ABUSE >:-(");}
                gamepad1.runRumbleEffect(batteryCritical);
                gamepad2.runRumbleEffect(batteryCritical);
            }
                telemetry.update();
                ext_past = currentGamepad2.right_trigger;
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

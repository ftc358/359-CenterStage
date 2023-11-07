package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp (name = "V2 Meet 1 TeleOp",group = "TeleOP")
public class HelpTeleOPMeet1 extends LinearOpMode {
    // saved on codeshare.io/lmaocopeharderpeopleofpaper727

    // Dashboard Vars
    public static double rapidTrigger_thr = 0.1;
    public static double extension_sens = 0.727;

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



    public static double claw1Grab = 0.38;
    public static double claw2Grab = 0.47;



    // Servo Pos
    double flipPos;
    double claw1Pos = 0;
    public double claw2Pos = 0;
    public double ppPos = 0;

    double planeReleasePos = 0;

    // States
    float ext_past;

    // PS5 Rumble

    double straight = 0;
    double turn = 0;
    double strafe = 0;
    double extPos;

    double intakePower = 0;
    double liftPower = 0;

    double currentVoltage = 12;
    double drivetrainC = 0;
    double driveTrainPower = 0;
    double leftFrontC = 0;
    double leftBackC = 0;
    double rightFrontC = 0;
    double rightBackC = 0;
    double maxPower = 0;


    Gamepad.RumbleEffect batteryCritical, drop1Ready, drop2Ready;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void runOpMode() throws InterruptedException {
        batteryCritical = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 1000)
                .build();

        drop1Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 0.7, 200)
                .addStep(0, 0, 20)
                .build();
        drop2Ready = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 0.7, 100)
                .addStep(0, 0, 100)
                .build();


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));



        ext_past = 0;

        drive.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something

        if (currentVoltage < 12) {
            telemetry.addLine("Current Voltage is Below 12v. Please replace battery before match");
            telemetry.update();
        }

        int transferState = 0;
        waitForStart();

        while (opModeIsActive()) {
            //trust me bro - jonathan
            //in case i trust - lucas


            //Gamepad States
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //Power Systems
            currentVoltage = drive.voltageSensor.getVoltage();//Could Use for Something
            leftFrontC = drive.leftFront.getCurrent(CurrentUnit.AMPS);
            leftBackC = drive.leftBack.getCurrent(CurrentUnit.AMPS);
            rightFrontC = drive.rightFront.getCurrent(CurrentUnit.AMPS);
            rightBackC = drive.rightBack.getCurrent(CurrentUnit.AMPS);
            drivetrainC = leftFrontC+leftBackC+rightFrontC+rightBackC;
            driveTrainPower = currentVoltage*drivetrainC;

            driveTrainPower = currentVoltage * drivetrainC;
            maxPower = (driveTrainPower > maxPower)?driveTrainPower:maxPower;

            telemetry.addLine("Powertrain Statistics");
            telemetry.addData("Left Front Current (A)", leftFrontC);
            telemetry.addData("Left Back Current (A)", leftBackC);
            telemetry.addData("Right Front Current (A)", rightFrontC);
            telemetry.addData("Right Back Current (A)", rightBackC);
            telemetry.addLine();
            telemetry.addData("Drive Current (A)",drivetrainC);
            telemetry.addData("Power (W)",driveTrainPower);
            telemetry.addData("Max Power (W)",maxPower);






            //Curving Controls. This is extremely unecessary but it saves cycles in true Lucas Useless fashion
            straight = (-gamepad1.left_stick_y > 0.1) ? (Math.pow(-gamepad1.left_stick_x, 3) + 0.26) : (-gamepad1.left_stick_x < -0.1) ? (Math.pow(-gamepad1.left_stick_x, 3) - 0.26) : 0;
            strafe = (gamepad1.left_stick_x > 0.1) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.26) : (gamepad1.left_stick_x < -0.1) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.26) : 0;
            turn = (gamepad1.right_stick_x > 0.1) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.26) : (gamepad1.right_stick_x < -0.1) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.26) : 0;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight, strafe), turn));
            drive.updatePoseEstimate();


            //Intake Trigger
            intakePower = (currentGamepad2.left_trigger >0 && !currentGamepad2.dpad_left) ? intakeSpeed : (currentGamepad2.left_bumper || currentGamepad2.dpad_left) ? -1 : 0;

            //Extension Trigger
            extPos = (currentGamepad2.left_trigger >= 0.1) ? (1.0 - extension_sens + currentGamepad2.left_trigger * extension_sens) * 0.5 : 0;

            //Lift Trigger
            liftPower = (currentGamepad2.right_trigger >0)? currentGamepad2.right_trigger : (currentGamepad2.right_bumper) ? liftDown : liftIdle;

            //Transfer States
            if (currentGamepad2.x && !previousGamepad2.x) {
                transferState = (transferState + 1) % 8;
            } else if (currentGamepad1.b && !previousGamepad1.b && transferState == 5) {
                transferState = 7;
            } else if (gamepad2.a) {
                transferState = 7;
            }

            //Transfer Switch-Case
            switch (transferState) {
                case 0:
                    break;
                case 1:
                    ppPos = 0;
                    flipPos = flip_lift;
                    break;
                case 2:
                    ppPos = 0.1;
                    break;
                case 3: //Goes to Holding Position
                    ppPos = ppHold;
                    break;
                case 4: //Goes to Board Dropping Position
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    flipPos = flip_vert;
                    ppPos = ppBoardDrop;
                    gamepad1.runRumbleEffect(drop1Ready);
                    break;
                case 5: //Drops the Outermost
                    claw2Pos = 0;
                    gamepad1.runRumbleEffect(drop2Ready);
                    break;
                case 6: //Drops the Innermost
                    claw1Pos = 0;
                    break;
                case 7: //Resets
                    ppPos = ppGet;
                    flipPos = flip_vert;
                    break;
            }

            //Attack Plan R
            planeReleasePos = currentGamepad1.options ? 1.0 : 0;


            //Bucket Flip
            if (currentGamepad2.dpad_left) {
                flipPos = flip_dump;
            } else if (currentGamepad2.left_trigger == 0 && !currentGamepad2.dpad_left) {
                flipPos = flip_vert;
            } else if (((gamepad2.left_trigger - ext_past) > 0) && !gamepad2.dpad_left) {
                //extension trigger
                flipPos = flip_intake;
            } else if (((ext_past - gamepad2.left_trigger) > rapidTrigger_thr) && !gamepad2.dpad_left) {
                //extension retract
                flipPos = flip_half;
            }




            //Unified Servo Actuation
            drive.ext1.setPosition(extPos);
            drive.ext2.setPosition(extPos);
            drive.flip1.setPosition(flipPos);
            drive.flip2.setPosition(flipPos);
            drive.claw1.setPosition(claw1Pos);
            drive.claw2.setPosition(claw2Pos);
            drive.placerPivot1.setPosition(ppPos);
            drive.placerPivot2.setPosition(ppPos);
            drive.planeRelease.setPosition(planeReleasePos);

            //Unified External Motor Action
            drive.lift1.setPower(liftPower);
            drive.lift2.setPower(liftPower);
            drive.intake.setPower(intakePower);


            //Debug
            telemetry.addLine("Drive");
            telemetry.addData("Straight", straight);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);


            telemetry.addData("Current Transfer State ", transferState);
            if (currentVoltage < 7) {
                for (int i = 0; i <= 10; i++) {
                    telemetry.addLine("STOP BATTERY ABUSE >:-(");
                }
                gamepad1.runRumbleEffect(batteryCritical);
                gamepad2.runRumbleEffect(batteryCritical);
            }
            telemetry.update();
            ext_past = currentGamepad2.right_trigger;
        }

    }
}

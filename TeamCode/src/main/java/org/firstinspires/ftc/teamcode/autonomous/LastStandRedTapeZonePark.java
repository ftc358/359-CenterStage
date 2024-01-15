package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.RoboConstants.claw1Grab;
import static org.firstinspires.ftc.teamcode.RoboConstants.claw2Grab;
import static org.firstinspires.ftc.teamcode.RoboConstants.ppBoardDrop;
import static org.firstinspires.ftc.teamcode.RoboConstants.ppHold;


import android.util.Size;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vision.BlueDetectionLeft;
import org.firstinspires.ftc.teamcode.vision.RedDetectionRight;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class LastStandRedTapeZonePark extends LinearOpMode {
    //Hardware Class Inserts
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx leftBack= null;

    private Servo pp1 = null;
    private Servo pp2 = null;

    private Servo claw1 = null;

    private Servo claw2 = null;

    private Servo ext1 = null;
    private Servo ext2 = null;


    private BHI260IMU imu = null;




    public double globalHeading;
    public double relativeHeading;
    public double offset;


    //Vision portal inserts
    private VisionPortal visionPortal;
    public RedDetectionRight detector = new RedDetectionRight(telemetry);
    public int color = 0;


    ///JONATHANS INSERTS
    //Turning
    private double Kp = 0.15; // Proportional constant TUNE THIS
    private double Ki = 0.01; // Integral constant TUNE THIS
    private double Kd = 0.08; // Derivative constant TUNE THIS

    private double previousError = 0;
    private double integral = 0;

    //Driving
    public static double Kp_dist = 0.1; // Proportional constant for distance TUNE
    public static double Ki_dist = 0.01; // Integral constant for distance TUNE
    public static double Kd_dist = 0.05; // Derivative constant for distance TUNE

    // PID constants for heading
    private double Kp_heading = 0.1; // Proportional constant for heading TUNE
    private double Ki_heading = 0.01; // Integral constant for heading TUNE
    private double Kd_heading = 0.05; // Derivative constant for heading TUNE

    private double encoder_tick_scale = 1/0.00054;

    private double previousErrorDist = 0;
    private double integralDist = 0;

    private double previousErrorHeading = 0;
    private double integralHeading = 0;

    IMU.Parameters myParameters;



    public void runOpMode(){
        //Hardwareclass Init, nothing to do with Roadrunner
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ext1 = hardwareMap.get(Servo.class,"ext1");
        ext2 = hardwareMap.get(Servo.class,"ext2");
        ext1.setDirection(Servo.Direction.REVERSE);

        pp1 = hardwareMap.get(Servo.class,"placerPivot1");
        pp2 = hardwareMap.get(Servo.class,"placerPivot2");
        pp2.setDirection(Servo.Direction.REVERSE);

        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");


        imu = hardwareMap.get(BHI260IMU.class,"imu");

        myParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myParameters);

//        leftEncoder = new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
//        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));
//        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);




        //https://www.youtube.com/watch?v=cqs5sdCPm8E&list=WL&index=21&ab_channel=testsubjec
        //you got this bro - jonathan

        initVision();

        ext1.setPosition(0.06);//idle no stress
        ext2.setPosition(0.06);
        claw1.setPosition(claw1Grab);
        claw2.setPosition(claw2Grab);
        sleep(1000);
        pp1.setPosition(ppHold);
        pp2.setPosition(ppHold);

        while(opModeInInit() &&!isStarted()){
             color = RedDetectionRight.getReadout();
            telemetry.addLine("In Initialization");
             telemetry.addData("Useless sees ",color);
             telemetry.addData("LeftValue",RedDetectionRight.leftValue);
             telemetry.addData("CenterValue",RedDetectionRight.centerValue);
             telemetry.addData("RightValue",RedDetectionRight.rightValue);
            telemetry.update();
        }

        //waitForStart(); //Extra Redundant Precaution
        if (opModeIsActive()) {

            ///AUTO CODE EXECUTED BELOW
            //ALL ROBOT POWER WILL BE LARGER THAN 0.3
            telemetry.addData("First Sequence: Drive to Tape",0);

            //First Move
            robot_move(-36, 0.4);
            sleep(1000);

            if (color == 0 || color == 1){
                robot_move(-2,0.4);
                robot_turn(27,0.4);
                sleep(1000);

            }else if (color == 2){
                robot_move(-6,0.4);
                sleep(1000);
                robot_turn(-10,0.4);

            }else if (color == 3){
                //Turn to 3:

                robot_move(6,0.4);
                robot_turn(-29,0.4);
                sleep(1000);
            }





            //Drops
            pp1.setPosition(0.85);
            pp2.setPosition(0.85);
            sleep(1000);
            claw2.setPosition(0);
            sleep(1000);
            pp1.setPosition(ppBoardDrop);
            pp2.setPosition(ppBoardDrop);

            if (color == 0 || color == 1){
                robot_turn(-112,0.4);
                sleep(1000);
            }else if (color == 2){
                robot_turn(-80,0.4);
                sleep(1000);

            }else if (color == 3){
                //Turn to 3:
                robot_turn(-60,0.4);
                sleep(1000);
            }


            //Turn to board from 3:


            //drive to board:
            robot_move(-48,0.4); // possibly insert apriltag before this
            sleep(2000);


            //Drops 2nd
            claw1.setPosition(0);
            sleep(1000);
            pp1.setPosition(ppHold);
            pp2.setPosition(ppHold);
            sleep(1000);
            /////END


            sleep(1000);

            telemetry.addData("Done", 3);
            telemetry.update();
        }

    }



//TURN STUFF ------ TURN STUFF ------ TURN STUFF ------ TU18RN STUFF ------ TURN STUFF ------ TURN STUFF ------ TURN STUFF ------
//MAKE SURE TO CHECK FUNCTIONS. most SHOUDL BE CORRECT BUT THERE ARE TEMP FUNCTION


//to call the function just use




    // Assuming you have a method to set power for left and right motors
    public void setLeftMotorPower(double power) {
        leftFront.setPower(power); //CHANGE MOTOR NAME
        leftBack.setPower(power); //CHANGE MOTOR NAME
    }

    public void setRightMotorPower(double power) {
        rightFront.setPower(power); //CHANGE MOTOR NAME
        rightBack.setPower(power); //CHANGE MOTOR NAME
    }

    public void robot_turn(double degrees, double maxPower) {
        double targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + degrees;
        double error = targetHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        long start = System.currentTimeMillis();
        long elapsed = 0;
        while ((Math.abs(error) > 0.5) && elapsed<10000) { // 0.5 degree tolerance, adjust as needed TUNE THIS LATER NO WORRIES RN, JUST LIKE HOW ACCURATE YOU NEED IT
            elapsed = System.currentTimeMillis()-start;
            error = targetHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double derivative = error - previousError;
            integral += error;

            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // clamp output to the maxPower
            output = Math.min(Math.max(output, -maxPower), maxPower);

            // cet motor powers
            setLeftMotorPower(-output);
            setRightMotorPower(output);

            previousError = error;

//            sleep(10);

        }

        imu.resetYaw();
        // Stop motors after turning
        setLeftMotorPower(0);
        setRightMotorPower(0);
    }

//TURN STUFF ------ TURN STUFF ------ TURN STUFF ------ TURN STUFF ------ TURN STUFF ------ TURN STUFF ------ TURN STUFF ------




//DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:)
//MAKE SURE TO CHANGE THE ENCODER NAMES

//to call just use this:
    ; //12 = distance (no units just trial error it), 1.0 = max motor power

    public void robot_move(double distance, double maxPower) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double targetDistanceTicks =  (distance * encoder_tick_scale /2); //somehow divided by 4 who knows
        double averageEncoder =  ((rightFront.getCurrentPosition() + leftFront.getCurrentPosition()) / 2.0);

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorDist, errorHeading;

        while (Math.abs(targetDistanceTicks - averageEncoder) > 1000) {

            // Tolerance in ticks, adjust as needed
            averageEncoder = ((rightFront.getCurrentPosition() + leftFront.getCurrentPosition()) / 2.0);
            errorDist = targetDistanceTicks - averageEncoder;

            telemetry.addData("avg Encoder",averageEncoder);
            telemetry.addData("errorDist",errorDist);
            telemetry.addData("heading",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            // Distance PID
            double derivativeDist = errorDist - previousErrorDist;
            integralDist += errorDist;
            double outputDist = (Kp_dist * errorDist) + (Ki_dist * integralDist) + (Kd_dist * derivativeDist);

            // Heading PID
            errorHeading = startHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double derivativeHeading = errorHeading - previousErrorHeading;
            integralHeading += errorHeading;
            double outputHeading = (Kp_heading * errorHeading) + (Ki_heading * integralHeading) + (Kd_heading * derivativeHeading);

            // Combine distance and heading outputs and clamp to maxPower
            double leftPower = Math.min(Math.max(outputDist - outputHeading, -maxPower), maxPower);
            double rightPower = Math.min(Math.max(outputDist + outputHeading, -maxPower), maxPower);

            setLeftMotorPower(leftPower);
            setRightMotorPower(rightPower);
            telemetry.addData("leftPower",leftPower);
            telemetry.addData("rightPower",rightPower);

            previousErrorDist = errorDist;
            previousErrorHeading = errorHeading;
            telemetry.update();

        }
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Stop motors
        setLeftMotorPower(0);
        setRightMotorPower(0);
    }


    private void initVision() {

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }






}
//DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:) DRIVE STUFF ------- (;-:)






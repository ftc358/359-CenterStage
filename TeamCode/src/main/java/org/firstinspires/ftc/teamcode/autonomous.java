package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuning.NormalFrontTeleOpLucasRevised;

/*
public class robot(){
        public Robot(HardwareMap hardwareMap){

        }
        public Action bucketTransfer(){

        }
        public Action ppHold(){
            drive.ppHold
        }
        flip1.setPosition(NormalFrontTeleOpLucasRevised.flip_intake);
        flip2.setPosition(NormalFrontTeleOpLucasRevised.flip_intake);
        }

 */
@Autonomous
public class autonomous extends LinearOpMode {
public int color = 0;

    public void runOpMode(){
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, toRadians(90)));
       // drive.setPoseEstimate(startPose);
        drive.bucketTransfer();
        drive.extendZero();
        drive.ppZero();
        sleep(500);
        color = 1;

        drive.ppHold();
        waitForStart();
        if (color == 1) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(8.4, -30, Math.toRadians(0)), 90)
                            .build()));

        } else if (color ==2){
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(12,-32.5,Math.toRadians(270)),0)
                            .build()));

        } else if (color == 3) {
            runBlocking(new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(17.4, -34, Math.toRadians(-92)),0)
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
                            .splineToLinearHeading(new Pose2d(49, -23,Math.toRadians(178)), Math.toRadians(0))
                            .build()
            ));
        } else if (color ==2){
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49, -32,Math.toRadians(178)), Math.toRadians(0))
                            .build()
            ));
        } else if (color == 3) {
            runBlocking(       new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(49, -37,Math.toRadians(178)), Math.toRadians(0))
                            .build()
            ));
        }


        //.strafeTo(new Vector2d(-60, -36))
                sleep(1000);
                drive.dropAll();
                sleep(1000);
                drive.bucketVertical();
                drive.ppZero();

               runBlocking( new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(53,-55))
                                .build()
                ));


    }
}

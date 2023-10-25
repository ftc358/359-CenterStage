package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;





@Autonomous
public class autonomous extends LinearOpMode {

    public void runOpMode(){

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, toRadians(90)));
       // drive.setPoseEstimate(startPose);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)

                        .splineToLinearHeading(new Pose2d(24, -30  ,Math.toRadians(-90)),0)
                        // new SequentialAction(
                          //

                        //)
                        .splineToLinearHeading(new Pose2d(44, -40,Math.toRadians(178)), Math.toRadians(0))
                        .lineToY(-36)
                        .lineToX(-60)
                        .build());


    }
}

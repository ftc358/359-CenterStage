package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoboConstants;

@Config
@TeleOp (name = "DODODODOOD",group = "TeleOP")
public class Doodoodoo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        while (opModeIsActive()) {
            drive.placerPivot1.setPosition(gamepad1.left_trigger*0.1);
            drive.placerPivot2.setPosition(gamepad1.left_trigger*0.1);

            drive.ext1.setPosition(gamepad1.right_trigger*0.1);
            drive.ext2.setPosition(gamepad1.right_trigger*0.1);

            telemetry.addData("ext",gamepad1.right_trigger*0.1);//0.06
            telemetry.addData("pppos",gamepad1.left_trigger*0.1);//0.05
            telemetry.update();
        }


    }
}


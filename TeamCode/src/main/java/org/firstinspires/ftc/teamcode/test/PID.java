package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@Config
public class PID extends LinearOpMode {

    DcMotorEx motor;
    DcMotorEx motor2;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double intSum = 0;
    public static double targetPosition = 5000;
    ElapsedTime timer = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double lastError = 0;
    @Override
    public void runOpMode() throws InterruptedException{

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        motor = hardwareMap.get(DcMotorEx.class, "lift1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()){
            double power = PID(targetPosition, motor.getCurrentPosition());
            packet.put("power", power);
            packet.put("position", motor.getCurrentPosition());
            packet.put("error", lastError);
            motor.setPower(power);
            motor2.setPower(power);
            dashboard.sendTelemetryPacket(packet);
            //goToPositionIntakeSlides(timer,Kp,Ki,Kd,targetPosition);
        }
    }
    public double PID(double reference, double state) {
        double error = reference - state;
        intSum += error * timer.seconds();
        double derivative = (error-lastError)/timer.seconds();
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (intSum * Ki);
        return output;
    }

}


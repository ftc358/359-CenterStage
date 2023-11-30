/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoboConstants;

@TeleOp(name = "Diffy Test PP", group = "Testing")
public class DiffyTestBasic extends LinearOpMode {

    Servo ppAngle, pp1, pp2, grab1, grab2;
    AnalogInput diffyLeftEnc, diffyRightEnc;
    double ppPos = 0.5;
    double pp1Pos = 0;
    double pp2Pos = 0;
    double claw1Grab = 0;
    double claw2Grab = 0;
    double diffyDiff = 0.3;



    public double diffyLeftPos(){
        return diffyLeftEnc.getVoltage()/3.3*(360/355);
    }
    public double diffyRightPos(){
        return diffyRightEnc.getVoltage()/3.3*(360/355);
    }
    @Override
    public void runOpMode() {
        pp1 = hardwareMap.get(Servo.class,"placerPivot1");
        pp2 = hardwareMap.get(Servo.class,"placerPivot2");
        grab1 = hardwareMap.get(Servo.class, "claw1");
        grab2 = hardwareMap.get(Servo.class, "claw2");
        ppAngle = hardwareMap.get(Servo.class, "ppAngle");
        diffyLeftEnc = hardwareMap.get(AnalogInput.class, "diffyLeftEnc");
        diffyRightEnc = hardwareMap.get(AnalogInput.class,"diffyRightEnc");



        telemetry.addData("Servo Pos", ppPos );
        telemetry.update();
        waitForStart();
        int transferState = 0;
        while(opModeIsActive()){
            switch (transferState){
                case 0:
                    pp1Pos = 0;
                    pp2Pos = 0;
                    claw1Grab = 0;
                    claw2Grab = 0;
                    ppPos = 0.5;
                    break;
                case 1:
                    claw1Grab = RoboConstants.claw1Grab;
                    claw2Grab = RoboConstants.claw2Grab;
                    break;
                case 2:
                    pp1Pos = RoboConstants.ppBoardDrop;
                    pp2Pos = RoboConstants.ppBoardDrop;
                    break;
                case 3:
                    pp1Pos = RoboConstants.ppBoardDrop+gamepad1.left_stick_x;
                    pp2Pos = RoboConstants.ppBoardDrop-gamepad1.left_stick_x;
                    ppPos = 0.5+gamepad1.left_stick_x;
                    break;
                case 4:
                    claw1Grab = 0;
                    claw2Grab = 0;
                case 5:
                    pp1Pos = RoboConstants.ppBoardDrop;
                    pp2Pos = RoboConstants.ppBoardDrop;
                    ppPos = 0.5;
                    if (diffyLeftPos() == diffyRightPos()){
                        transferState = 0;
                    }
                    break;
            }

            pp1.setPosition(pp1Pos);
            pp2.setPosition(pp2Pos);
            grab1.setPosition(claw1Grab);
            grab2.setPosition(claw2Grab);
            ppAngle.setPosition(ppPos);

            telemetry.addData("Left ENC", diffyLeftPos());
            telemetry.addData("Right ENC", diffyRightPos());

            telemetry.addData("pp1Pos",pp1Pos);
            telemetry.addData("pp2Pos",pp2Pos);
            telemetry.addData("claw1Grab",claw1Grab);
            telemetry.addData("claw2Grab",claw2Grab);
            telemetry.addData("ppPos",ppPos);
            telemetry.update();
            }





        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

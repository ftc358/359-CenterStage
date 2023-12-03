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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Testing")
@Disabled
public class ServoTest extends LinearOpMode {

    Servo   servo1, servo2;
    double servo1Pos = 0;
    double servo2Pos = 0;


    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class,"placerPivot1");
        servo2 = hardwareMap.get(Servo.class, "placerPivot2");
        servo2.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Servo1 Pos", servo1Pos);
        telemetry.addData("Servo2 Pos", servo2Pos);

        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.square){
                servo1Pos = 0;
                servo2Pos = 0;
            }
            else if (gamepad1.circle){
                servo1Pos = 1.0;
                servo2Pos = 1.0;
            }
            else {
                servo1Pos = 0.5;
                servo2Pos = 0.5;
                servo1Pos +=gamepad1.left_stick_x;
                servo2Pos -=gamepad1.left_stick_x;
            }
            telemetry.addData("Servo Pos", servo1Pos);
            telemetry.addData("2", servo2Pos);
            telemetry.update();
            servo1.setPosition(servo1Pos);
            servo2.setPosition(servo2Pos);
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

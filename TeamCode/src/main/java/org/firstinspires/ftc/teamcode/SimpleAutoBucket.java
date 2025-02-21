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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Mecanum: Score Park", group="Mecanum")
//@Disabled
public class SimpleAutoBucket extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();
    private ElapsedTime     runtime = new ElapsedTime();

    void drive(double drive, double strafe) {

        // Forward and right is POSITIVE
        robot.frontLeftDrive.setPower(drive + strafe);
        robot.frontRightDrive.setPower(drive - strafe);
        robot.backLeftDrive.setPower(drive - strafe);
        robot.backRightDrive.setPower(drive + strafe);
    }

    void spin(double power) {

        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(-power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(-power);
    }


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Do this thing
        //int lift1Pos = robot.liftMotor1.getCurrentPosition();
        //int lift2Pos = robot.liftMotor2.getCurrentPosition();

        //double clawPosition              = robot.CLAW_HOME;          // Servo's position
        //final double CLAW_SPEED          = 0.10;                    // Sets rate to move servo

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//how goon????
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Drive the robot forward
        drive(0,1);
        sleep(750);

        drive(0,0);

        spin(-1);
        sleep(200);

        spin(0);

        drive(1,0);
        sleep(300);

        drive(0,0);

        // Rise lifts
        robot.arm.setPower(1);
        sleep(4500);

        drive(0,0);

        robot.elbow.setPosition(0.4);

        robot.claw.setPosition(0.7);

        robot.elbow.setPosition(0.6);

        robot.claw.setPosition(1);

        robot.arm.setPower(-1);
        sleep(4000);

        robot.arm.setPower(0);

        spin(1);
        sleep(200);

        spin(0);

        drive(0,1);
        sleep(300);

        drive(0,0);

        drive(-1,0);
        sleep(4600);

        drive(0,0);

        drive(0,-1);
        sleep(1000);

        drive(0,0);



        /*


        // Turn the robot right
        //spin(0.5);
        //sleep(600);

        //Park, stop the motors
        drive(0,0);

        //Drive the robot forward
        drive(0.5,0);
        sleep(100);

        //Park, stop the motors
        drive(0,0);
        sleep(700);

        robot.clawServo.setPosition(180);
        sleep(1000);

        robot.clawServo.setPosition(0);
        sleep(500);

        //Drive the robot backward
        drive(-0.5,0);
        sleep(100);

        // Turn the robot left
        spin(-0.5);
        sleep(600);

        //Park, stop the motors
        drive(0,0);
        sleep(500);

        //Lower lifts
        robot.liftMotor1.setPower(-1);
        robot.liftMotor2.setPower(-1);
        sleep(3000);
        robot.liftMotor1.setPower(0);
        robot.liftMotor2.setPower(0);

        //Drive the robot backward
        drive(-0.5,0);
        sleep(1300);

        //Park, stop the motors
        drive(0,0);
        sleep(500);

        // Drive the robot sideways
        drive(0,0.5);
        sleep(3000);

        //Park, stop the motors
        drive(0,0);*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}

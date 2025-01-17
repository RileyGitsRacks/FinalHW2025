/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class HardwareMecanum {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;
    public DcMotor climb = null;
    public DcMotor climbBend = null;
    public DcMotor arm = null;
    public Servo elbow = null;
    public Servo wrist = null;
    public Servo claw = null;

//how goon????



    public static final double ELBOW_HOME      = 0.30; // Starting position for Servo Twerk
    public static final double ELBOW_MIN_RANGE = 0.30; // Smallest number value allowed for servo position
    public static final double ELBOW_MAX_RANGE = 0.60; // Largest number value allowed for servo position*/

    public static final double WRIST_HOME      = 0.8; // Starting position for Servo Twerk
    public static final double WRIST_MIN_RANGE = 0.60; // Smallest number value allowed for servo position
    public static final double WRIST_MAX_RANGE = 1.00; // Largest number value allowed for servo position*/

    public static final double CLAW_HOME      = 1.00; // Starting position for Servo Claw
    public static final double CLAW_MIN_RANGE = 0.70; // Smallest number value allowed for servo position
    public static final double CLAW_MAX_RANGE = 1.00; // Largest number value allowed for servo position

    // local Opmode members
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // constructor
    public HardwareMecanum(){

    }

//how goon????
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public HardwareMecanum(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap) {
        //save reference to hardwaremap
        hwMap = ahwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        frontLeftDrive = hwMap.get(DcMotor.class, "lf");
        frontRightDrive = hwMap.get(DcMotor.class, "rf");
        backLeftDrive = hwMap.get(DcMotor.class, "lb");
        backRightDrive = hwMap.get(DcMotor.class, "rb");
        climb = hwMap.get(DcMotor.class, "climb");
        climbBend = hwMap.get(DcMotor.class, "climbBend");
        arm = hwMap.get(DcMotor.class, "arm");

//how goon????
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        climb.setDirection(DcMotorSimple.Direction.REVERSE);
        climbBend.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//how goon????

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        climb.setPower(0);
        climbBend.setPower(0);
        arm.setPower(0);
//how goon????
//how goon????
        //Define and initialize ALL installed servos.
        elbow = hwMap.get(Servo.class, "elbow"); // set equal to name of the servo motor on driver hub
        elbow.setPosition(ELBOW_HOME); // setPosition actually sets the servos position and moves it

        wrist = hwMap.get(Servo.class, "wrist");
        wrist.setPosition(WRIST_HOME);

        claw = hwMap.get(Servo.class, "claw"); // set equal to name of the servo motor on driver hub
        claw.setPosition(CLAW_HOME); // setPosition actually sets the servos position and moves it

    }
}
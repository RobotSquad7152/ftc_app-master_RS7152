/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class RSSimpleTeleOp extends OpMode
{

	/*
     * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
    // TETRIX VALUES.

    DcMotor motor1;
    DcMotor motor2;
    Servo servo;
    Servo servoHopper;
    Servo servoChurro;
    Servo servoClimber;
    Servo servoRightZip;
    Servo servoLeftZip;
    Servo servoTape;
    Servo servoClutch;


    double servoPosRightZip = .5;//in .08 //out .77
    double servoPosLeftZip = .5;  //in 0.868 out 0.1

    double leftPower = 0;
    double rightPower = 0;
    double armPower = 0;
    double deadZone = .1;
    double servoDoorOpen = 1.0;
    double servoDoorClose = 0;
    double servoHopperPos = 0.5;//stopped continuous
    int bucketTarget;
    int bucketPos = 0;
    double maxSpinnerCollectPow = 1.0;
    double maxSpinnerDispensePow = 0.75;
    boolean continuousSpin = false;
    boolean j1aPressed = false;
    boolean j1bPressed = false;
    double servoChurroUp = 0.15;
    double servoChurroDown = 0.75;
    boolean drivingForward = true;
    boolean harvesterMoving = false;
    double servoClimberClose = 0;
    double servoClimberOpen = 1;
    double servoClimberPos = 0.5;//in .086 out .54
    //double servoRightZipOut = 0.77;
    //double servoRightZipIn = 0.08;
    //double servoLeftZipOut = 0.01;
    //double servoLeftZipIn = 0.868;
    double servoTapePos = .5;  //max up 0.98  down 0.5
    double servoClutchPos = .6;
    double servoChurroPos = 0.1;
    double servoDoor = 0;

    //double servoPos = 0.5;

    /**
     * Constructor
     */
    public RSSimpleTeleOp()
    {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()
    {
        /*Controller		Config Name		Robot
        Front Drive Port 1	motor_1			Front Right
		Front Drive Port 2	motor_2			Front Left
		Rear Drive Port 1	motor_3			Back Right
		Rear Drive Port 2	motor_4			Back Left
		Extend/hang			motor_5			Slide motor
		Harvester port 1	motor_8			Harvester bucket
		Harvester port 2	motor_7			Harvester spinner*/

//        motor1 = hardwareMap.dcMotor.get("motor_1");
//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor2 = hardwareMap.dcMotor.get("motor_2");
//        motor2.setDirection(DcMotor.Direction.FORWARD);

//        motorSlide = hardwareMap.dcMotor.get("motor_slide");
//        motorSlide.setDirection(DcMotor.Direction.FORWARD);
//
//        motorTape = hardwareMap.dcMotor.get("motor_tape");
//        motorTape.setDirection(DcMotor.Direction.FORWARD);
//
//        motorBucket = hardwareMap.dcMotor.get("motor_harv_arm");
//        motorBucket.setDirection(DcMotor.Direction.FORWARD);
//
//        motorSpinner = hardwareMap.dcMotor.get("motor_spinner");
//        motorSpinner.setDirection(DcMotor.Direction.FORWARD);
//
//        servoDoor = hardwareMap.servo.get("servo_door");
//        servoDoor.setPosition(servoDoorClose);
//
        servoHopper = hardwareMap.servo.get("servo_hopper");
        servoHopper.setPosition(servoHopperPos);
//
        servoChurro = hardwareMap.servo.get("servo_churro");
        servoChurro.setPosition(servoChurroPos);
//
//        servoClimber = hardwareMap.servo.get("servo_climber");
//        servoClimber.setPosition(servoClimberClose);

        servoRightZip = hardwareMap.servo.get("servo_rightzip");
        servoRightZip.setPosition(servoPosRightZip);

        servoLeftZip = hardwareMap.servo.get("servo_leftzip");
        servoLeftZip.setPosition(servoPosLeftZip);

        servoTape = hardwareMap.servo.get("servo_tape");
        servoTape.setPosition(servoTapePos);

        servoClimber = hardwareMap.servo.get("servo_climber");
        servoClimber.setPosition(servoClimberPos);
//
        servoClutch = hardwareMap.servo.get("servo_clutch");
        servoClutch.setPosition(servoClutchPos);

        //servoChurro = hardwareMap.servo.get("servo_churro");
        //servoChurro.setPosition(.5);
//        bucketThread =  BucketThreadSingleton.getBucketThread();
//
//        bucketThread.InitializeBucket(motorBucket);
//
//        bucketTarget = motorBucket.getCurrentPosition();
//

        //	motorHarvester.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        //	motorHarvester.setMode(DcMotorController.RunMode.RESET_ENCODERS);

    }

    @Override
    public void start()
    {
        //motorBucket.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    //    bucketThread.startBucketThread();

      //  bucketTarget = motorBucket.getCurrentPosition();
    }


    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop()
    {



        //	telemetry.addData("y1 ", j1y1);
        //	telemetry.addData("x2 ", j1x2);
        telemetry.addData("servoClimber", servoClimberPos);

        //telemetry.addData("servoLeftZip", servoPosLeftZip);
        telemetry.addData("servoTape", servoTapePos);

//        if( gamepad1.a)
//            motor1.setPower(0.5);
//        else if( gamepad1.x)
//            motor1.setPower(-0.5);
//        else
//            motor1.setPower(0);
//
//        if( gamepad1.b)
//            motor2.setPower(0.5);
//        else if( gamepad1.y)
//            motor2.setPower(-0.5);
//        else
//            motor2.setPower(0);
//
//        if( gamepad1.a)
//            motor1.setPower(0.5);
//        else if( gamepad1.x)
//            motor1.setPower(-0.5);
//        else
//            motor1.setPower(0);



        if(gamepad1.a)
            servoChurroPos = servoChurroPos + 0.01;
        else if(gamepad1.b)
            servoChurroPos = servoChurroPos - 0.01;
        servoChurro.setPosition(Range.clip(servoChurroPos, 0, 1));

        /*if(gamepad1.x)
            servoPosLeftZip = servoPosLeftZip +0.01;
        else if( gamepad1.y)
            servoPosLeftZip = servoPosLeftZip -0.01;
        servoLeftZip.setPosition(Range.clip(servoPosLeftZip, 0, 1));
        */
        if( gamepad1.dpad_up)
            servoClimberPos += 0.002;
        else if( gamepad1.dpad_down)
            servoClimberPos -= 0.002;

        servoClimber.setPosition(Range.clip(servoClimberPos, 0, 1));
        telemetry.addData("servoCrurroPos", "CurroPos, " + servoChurroPos);
        //telemetry.addData("servoPosLeftZip", "LeftPos, " + servoPosLeftZip);
//
//
//
        if( gamepad1.right_bumper)
            servoTapePos += 0.002;
        else if( gamepad1.left_bumper)
            servoTapePos -= 0.002;

        servoTape.setPosition(Range.clip(servoTapePos, 0.5, .98));
//
//        if( gamepad1.dpad_up)
//            servoTapePos += 0.002;
//        else if( gamepad1.dpad_down)
//            servoTapePos -= 0.002;
//
//        servoTape.setPosition(Range.clip(servoTapePos, 0, 1));
    }


    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop()
    {
        //bucketThread.stopBucketThread();
        Log.d("DEBUG", "RSTeleOp.Stop");
    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0)
        {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16)
        {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        }
        else
        {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}

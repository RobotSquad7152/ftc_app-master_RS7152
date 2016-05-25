/* Copyright (c) 2015 Qualcomm Technologies Inc

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

import RobotSquad.RSRobot;


public class RSTestAuto extends RSLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        InitHardware();
        robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = true;

        waitForStart();


        robot.DeliverClimber();

//        robot.stallDetectionOn = false;
//        robot.DriveToUSDistance(20);
//        while (true)
//            telemetry.addData("Ultrasonic value ", robot.readUltrasonic());

        /*
        //robot.DriveForward(.5,7);
        robot.SpinLeft(.7, 45);
        robot.DriveForwardToTape(.2, 100, RSRobot.Alliance.WHITE);
        robot.DriveBackward(.5, 15);
        robot.DeliverClimberRight();
        */

        //robot.DeliverClimberRight();
        // robot.startHarvester();

  //      robot.DriveForward(.7, 200);

        //Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinLeft(.7, 90);
//
//        sleep(500);
//        robot.SpinLeft(.7, 90);
//
//        sleep(500);
//        robot.SpinLeft(.7, 90);
//
//        sleep(500);
//        robot.SpinLeft(.7, 90);
//
//        sleep(500);

       // robot.DriveForward(.5,50);

        //Log.d("RSTESTAUTOIIIIIIIIIII", "DONE!!!!!!!!!!!!!");


       //robot.SpinRight(.7, 360);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinRight(.7, 90);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinRight(.7, 90);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinRight(.7, 90);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);

        //    robot.SpinRight(1, 90);
        //  sleep(1000);

        //   robot.SpinLeft(1,90);


//    //robot.DriveBackward(.1, 1000);
//      for( int i=0; i<3000; i++)
//      {
//          sleep(10);
//          telemetry.addData("curr heading ", robot.GetCurrentHeading());
//      }
//    robot.stopHarvester();


        //     for (int count = 0; count < 8; count ++)
        //    {
//            robot.DriveBackward(.5, 100);
//            sleep(500);
//
//            robot.DriveForward(.5, 100);
//            sleep(500);

//robot.CrazyDrive();

        // robot.DriveForwardWall(.3, 100, 10);


    }
}

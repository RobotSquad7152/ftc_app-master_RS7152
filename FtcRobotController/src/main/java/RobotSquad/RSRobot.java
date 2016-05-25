package RobotSquad;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.opmodes.RSLinearOpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Calendar;

//import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Tony on 10/4/2015.
 */

public class RSRobot
{

    RSLinearOpMode opMode;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorHarvester;
    DcMotor motorSpinner;
    DcMotor motorSlide;
    Servo servoChurro;
    Servo servoHopper;
    Servo servoClimberRight;
    Servo servoClimberLeft;
    Servo servoDoor;
    DcMotorController motorControllerFrontDrive;
    DcMotorController motorControllerRearDrive;
    // private MRGyroThread gyrothread;
    private ModernRoboticsI2cGyro gyro;
    private double targetHeading;
    ColorSensor sensorRGB;
    OpticalDistanceSensor sensorODS;
    UltrasonicSensor sensorUltrasonic;
    public boolean stallDetectionOn = false;

    private BucketThreadSingleton bucketThread;
    double turnError = 0;

    public enum Alliance
    {
        BLUE(1),
        UNKNOWN(0),
        RED(-1),
        WHITE(2);
        private final int alliance;

        private Alliance(int alliance)
        {
            this.alliance = alliance;
        }

    }

    static final int forward = 1;
    static final int backward = -1;
    static final int right = 1;
    static final int left = -1;


    Alliance myAlliance;

    public void setMyAlliance(Alliance myAlliance)
    {
        this.myAlliance = myAlliance;
    }

    //defines drive wheel diameter -- a 2 inch sprocket this year
    final double wheeldiacm = 4 * 2.54;
    //defines drive wheel circumference
    final double wheelcirccm = wheeldiacm * 3.141592653589793;
    //defines how many teeth on gear attached to motor (no gearing this year)
    final double motorgearteeth = 1;
    //defines how many teeth on gear attached to wheel (no gearing this year)
    final double wheelgearteeth = 1;
    //encoder counts per rotation of the motor
    final double motorclicksperrotation = 1120;
    //calculates how far the robot will drive for each motor encoder click
    final double onemotorclick = ((motorgearteeth / wheelgearteeth) * wheelcirccm) / motorclicksperrotation;

    final public int park = 0;
    final public int defenseBeacon = 1;

    public void Initialize()
    {
        /*if (gyro != null)
        {
            gyrothread = new MRGyroThread(gyro);

            gyrothread.calibrategyro();

            gyrothread.start();
        }
*/

        gyro.calibrate();

        while (gyro.isCalibrating())
        {
            try
            {
                opMode.sleep(50);
            } catch (InterruptedException e)
            {

            }
        }

        opMode.telemetry.addData("Gyro READY!", "Yee Haa");

        targetHeading = 0;

        if (motorBackRight != null)
            motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        if (motorBackLeft != null)
            motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public RSRobot(ModernRoboticsI2cGyro gyro)
    {
        this.gyro = gyro;
    }

    public double GetCurrentHeading()
    {
        return -gyro.getIntegratedZValue();
    }

    public double GetDeltaHeading()
    {
        double currentHeading = gyro.getHeading();
        double deltaHeading = targetHeading - currentHeading;
        if (deltaHeading > 180)
        {
            //MR gyro returns 0 through 359.  We need it to return -179 through 180
            deltaHeading = deltaHeading - 360;
        }
        else if (deltaHeading < -180)
        {
            deltaHeading = deltaHeading + 360;
        }
        return deltaHeading;
    }

    public void ChangeTargetHeading(double degrees)
    {
        targetHeading = targetHeading + degrees;
        if (targetHeading > 359)
        {
            targetHeading = targetHeading - 360;
        }
        else if (targetHeading < 0)
        {
            targetHeading = targetHeading + 360;
        }
    }

    public void ResetCurrentHeading()
    {
        gyro.resetZAxisIntegrator();

        while (gyro.getIntegratedZValue() != 0)
        {
            try
            {
                opMode.sleep(50);
            } catch (InterruptedException e)
            {

            }
        }
    }

    public void setOpMode(RSLinearOpMode opMode)
    {
        this.opMode = opMode;
    }

    public void SetFrontRightMotor(DcMotor motor)
    {
        motorFrontRight = motor;
    }

    public void SetFrontLeftMotor(DcMotor motor)
    {
        motorFrontLeft = motor;
    }

    public void SetBackRightMotor(DcMotor motor)
    {
        motorBackRight = motor;
    }

    public void SetBackLeftMotor(DcMotor motor)
    {
        motorBackLeft = motor;
    }

    public void SetHarvesterMotor(DcMotor motor)
    {
        motorHarvester = motor;
    }

    public void SetSpinnerMotor(DcMotor motor)
    {
        motorSpinner = motor;
    }

    public void SetSlideMotor(DcMotor motor)
    {
        motorSlide = motor;
    }

    public void SetChurroServo(Servo servo)
    {
        servoChurro = servo;
    }

    public void SetHopperServo(Servo servo)
    {
        servoHopper = servo;
    }

    public void SetClimberRightServo(Servo servo)
    {
        servoClimberRight = servo;
    }

    public void SetClimberLeftServo(Servo servo)
    {
        servoClimberLeft = servo;
    }

    public void SetColorSensor(ColorSensor color)
    {
        sensorRGB = color;
    }

    public void SetODS(OpticalDistanceSensor ods)
    {
        sensorODS = ods;
    }

    public void SetUltrasonic(UltrasonicSensor ultrasonic)
    {
        sensorUltrasonic = ultrasonic;
    }

    public void setMotorControllerFrontDrive(DcMotorController motorControllerFrontDrive)
    {
        this.motorControllerFrontDrive = motorControllerFrontDrive;
    }

    public void setMotorControllerRearDrive(DcMotorController motorControllerRearDrive)
    {
        this.motorControllerRearDrive = motorControllerRearDrive;
    }

    double LeftPowerCalc(double calculatedPow, double direction, double currentHeading)
    {
        double powerFactor = 2;
        double steerRatio = powerFactor / calculatedPow;

        double leftCalcPow = Range.clip((calculatedPow * direction) - (currentHeading / steerRatio), -1, 1);
        if ((calculatedPow * direction < 0 && leftCalcPow > 0) || (calculatedPow * direction > 0 && leftCalcPow < 0))
        {
            leftCalcPow = 0;
        }
        return (leftCalcPow);
    }

    double RightPowerCalc(double calculatedPow, double direction, double currentHeading)
    {
        double powerFactor = 2;
        double steerRatio = powerFactor / calculatedPow;

        double rightCalcPow = Range.clip((calculatedPow * direction) + (currentHeading / steerRatio), -1, 1);
        if ((calculatedPow * direction < 0 && rightCalcPow > 0) || (calculatedPow * direction > 0 && rightCalcPow < 0))
        {
            rightCalcPow = 0;
        }
        return (rightCalcPow);
    }


    private long Drive(double power, long distance, double direction) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos = 0;
        int rightMotorPos = 0;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;

        Log.d("@@@@@@@@@@@@@@@Drive: ", "" + distance * direction);
        Log.d("@@@Drive: Target head ", "" + targetHeading);

        //set current heading to zero
        ResetCurrentHeading();
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        int counter = 0;
        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
            opMode.telemetry.addData("Waiting left ", motorBackLeft.getCurrentPosition());
            opMode.telemetry.addData("Waiting right ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Power left ", motorBackLeft.getPower());
            opMode.telemetry.addData("Power right ", motorBackRight.getPower());
            counter++;
            if (counter % 100 == 0)
            {
                motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                counter = 0;
            }
        }
        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorBackLeft.getCurrentPosition()) < encoderTarget &&
                Math.abs(motorBackRight.getCurrentPosition()) < encoderTarget &&
                !IsRobotStalled(isLeftMotorStalled, isRightMotorStalled))
        {

            currentHeading = GetCurrentHeading();
            //      Log.d("@@@Heading ", "" + currentHeading);

            calculatedPow = calculateDrivePow(distance, Math.abs(motorBackLeft.getCurrentPosition()) * onemotorclick, power);

            //       Log.d("@@@Calc Pww ", "" + calculatedPow);

            leftCalculatedPow = LeftPowerCalc(calculatedPow, direction, currentHeading);

            rightCalculatedPow = RightPowerCalc(calculatedPow, direction, currentHeading);

            //       Log.d("@@@Left Calc Pow ", "" + leftCalculatedPow);
            //       Log.d("@@@Right Calc Pow ", "" + rightCalculatedPow);

            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);


            leftMotorPos = motorBackLeft.getCurrentPosition();
            rightMotorPos = motorBackRight.getCurrentPosition();

//			nxtDisplayBigTextLine( 5, "L %d", leftMotorPos );
            //		nxtDisplayTextLine( 7, "R %d", rightMotorPos );

            //if trying to move and not successfully moving
            if ((leftCalculatedPow != 0) && (Math.abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
            {
                //if stalling for 50 rounds through the loop (.5 second)
                if (++leftStallCount == 20)
                {
                    //left motor has stalled.
                    isLeftMotorStalled = true;

                }
            }
            else
            {
                // not stalled, reset stall counter
                leftStallCount = 0;

                isLeftMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            leftStallPos = leftMotorPos;

            //if trying to move and not successfully moving
            if ((rightCalculatedPow != 0) && (Math.abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
            {
                //if stalling for 50 rounds through the loop
                if (++rightStallCount == 20)
                {
                    //right motor has stalled.
                    isRightMotorStalled = true;


                }
            }
            else
            {
                // not stalled, reset stall counter
                rightStallCount = 0;

                isRightMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            rightStallPos = rightMotorPos;

            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        if (stallDetectionOn && (isLeftMotorStalled || isRightMotorStalled))
        {
            if (leftMotorPos > rightMotorPos)

                distance = (long) Math.abs(leftMotorPos * onemotorclick);
            else
                distance = (long) Math.abs(rightMotorPos * onemotorclick);
        }
        Log.d("@@@Distance ", "" + distance);

        return (distance);
    }

    private boolean IsRobotStalled(boolean isLeftMotorStalled, boolean isRightMotorStalled)
    {
        if (stallDetectionOn)
        {
            return (isLeftMotorStalled || isRightMotorStalled);
        }
        else
        {
            return false;
        }

    }

    public long DriveForward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (Drive(power, distance, 1));
    }

    public long DriveBackward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (Drive(power, distance, -1));
    }


    public long DriveToUSDistance(long distance) throws InterruptedException
    {
        boolean correctDistance = false;

        while (!correctDistance)
        {
            double usDistance = readUltrasonic();
            if (usDistance >= distance - 1 && usDistance <= distance + 1)
            {
                correctDistance = true;
            }
            else if (usDistance == 0)
            {
               // return (-1);
            }
            else if (usDistance >= distance + 1)
            {
                DriveBackward(.25, (long) usDistance - distance);
            }
            else if (usDistance <= distance - 1)
            {
                DriveForward(.25, distance - (long) usDistance);
            }
        }
            //else we're in the right spot
            return (0);
    }


    private long DriveToTape(double power, long distance, Alliance tapeColor, double direction) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos = 0;
        int rightMotorPos = 0;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;
        boolean tapeFound = false;

        Log.d("@@@@@@@@@@@@@@@Drive: ", "" + distance * direction);
        Log.d("@@@Drive: Target head ", "" + targetHeading);

        //set current heading to zero
        ResetCurrentHeading();
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        int counter = 0;
        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
            opMode.telemetry.addData("Waiting left ", motorBackLeft.getCurrentPosition());
            opMode.telemetry.addData("Waiting right ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Power left ", motorBackLeft.getPower());
            opMode.telemetry.addData("Power right ", motorBackRight.getPower());
            counter++;
            if (counter % 100 == 0)
            {
                motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                counter = 0;
            }
        }
        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorBackLeft.getCurrentPosition()) < encoderTarget &&
                Math.abs(motorBackRight.getCurrentPosition()) < encoderTarget &&
                !IsRobotStalled(isLeftMotorStalled, isRightMotorStalled) && !tapeFound)
        {

            currentHeading = GetCurrentHeading();
            //    Log.d("@@@Heading ", "" + currentHeading);

            calculatedPow = calculateDrivePow(distance, Math.abs(motorBackLeft.getCurrentPosition()) * onemotorclick, power);

            //    Log.d("@@@Calc Pww ", "" + calculatedPow);

            leftCalculatedPow = LeftPowerCalc(calculatedPow, direction, currentHeading);

            rightCalculatedPow = RightPowerCalc(calculatedPow, direction, currentHeading);

            //     Log.d("@@@Left Calc Pow ", "" + leftCalculatedPow);
            //    Log.d("@@@Right Calc Pow ", "" + rightCalculatedPow);

            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);


            leftMotorPos = motorBackLeft.getCurrentPosition();
            rightMotorPos = motorBackRight.getCurrentPosition();

//			nxtDisplayBigTextLine( 5, "L %d", leftMotorPos );
            //		nxtDisplayTextLine( 7, "R %d", rightMotorPos );

            //if trying to move and not successfully moving
            if ((leftCalculatedPow != 0) && (Math.abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
            {
                //if stalling for 50 rounds through the loop (.5 second)
                if (++leftStallCount == 20)
                {
                    //left motor has stalled.
                    isLeftMotorStalled = true;

                }
            }
            else
            {
                // not stalled, reset stall counter
                leftStallCount = 0;

                isLeftMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            leftStallPos = leftMotorPos;

            //if trying to move and not successfully moving
            if ((rightCalculatedPow != 0) && (Math.abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
            {
                //if stalling for 50 rounds through the loop
                if (++rightStallCount == 20)
                {
                    //right motor has stalled.
                    isRightMotorStalled = true;


                }
            }
            else
            {
                // not stalled, reset stall counter
                rightStallCount = 0;

                isRightMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            rightStallPos = rightMotorPos;

            if (GetColorRGB() == tapeColor)
            {
                Log.d("XXXXXXXXXXXXXXXXXXXX", "TapeFound");
                tapeFound = true;
            }

            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        if (stallDetectionOn && (isLeftMotorStalled || isRightMotorStalled))
        {

            Log.d("@@@STALLED!!!! ", "");

            if (leftMotorPos > rightMotorPos)

                distance = (long) Math.abs(leftMotorPos * onemotorclick);
            else
                distance = (long) Math.abs(rightMotorPos * onemotorclick);
        }
        Log.d("@@@Distance ", "" + distance);

        return (distance);
    }

    public long DriveForwardToTape(double power, long distance, Alliance tapeColor) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (DriveToTape(power, distance, tapeColor, 1));
    }

    public long DriveBackwardToTape(double power, long distance, Alliance tapeColor) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (DriveToTape(power, distance, tapeColor, -1));
    }

    public double readUltrasonic() throws InterruptedException
    {
        double ultrasonicValue = 0;
        double numberOfGoodValues = 0;
        double sumOfGoodValues = 0;
        double averageUltrasonicValue = 0;
        int counter = 0;

        while (counter < 5)
        {
            ultrasonicValue = sensorUltrasonic.getUltrasonicLevel();
            Log.d("ULTRASONIC", " " + ultrasonicValue);
            opMode.sleep(50);
            counter++;
            if (ultrasonicValue < 200 && ultrasonicValue > 0)
            {
                numberOfGoodValues++;
                sumOfGoodValues = sumOfGoodValues + ultrasonicValue;
            }
        }
        if (numberOfGoodValues > 0)
        {
            //average values
            averageUltrasonicValue = sumOfGoodValues / numberOfGoodValues;
        }
        return (averageUltrasonicValue);
    }


    private long DriveWall(double power, long distance, double targetDistance, double direction) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        double steerConsant = 40;
        double wallDistance;

        //    Log.d("@@@@@@@@@@@@@@@Drive:", "" + distance * direction);
        //   Log.d("@@@@@@@@Drive: Target", "" + targetHeading);
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        int counter = 0;
        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
            opMode.telemetry.addData("Waiting left ", motorBackLeft.getCurrentPosition());
            opMode.telemetry.addData("Waiting right ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Power left ", motorBackLeft.getPower());
            opMode.telemetry.addData("Power right ", motorBackRight.getPower());
            counter++;
            if (counter % 100 == 0)
            {
                motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                counter = 0;
            }
        }

        opMode.telemetry.addData("Waiting left ", "Done");
        opMode.telemetry.addData("Waiting right ", "Done");

        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorBackLeft.getCurrentPosition()) < encoderTarget &&
                Math.abs(motorBackRight.getCurrentPosition()) < encoderTarget)
        {

            wallDistance = sensorUltrasonic.getUltrasonicLevel();
            //currentHeading = GetCurrentHeading();
            calculatedPow = calculateDrivePow(distance, Math.abs(motorBackLeft.getCurrentPosition() * onemotorclick), power);

            leftCalculatedPow = Range.clip((calculatedPow * direction) - ((wallDistance - targetDistance) / steerConsant), -1, 1);

            rightCalculatedPow = Range.clip((calculatedPow * direction) + ((wallDistance - targetDistance) / steerConsant), -1, 1);


            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", GetCurrentHeading());
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);
            opMode.telemetry.addData("ultrasonic ", sensorUltrasonic.getUltrasonicLevel());
            opMode.waitForNextHardwareCycle();


            //Log.d("@@@@@@@@@@@@@@@Heading:", "" + GetCurrentHeading());
            // Log.d("@@@@@@@@@@@@@@@Delta:", "" + GetDeltaHeading());
        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        return (distance);
    }

    public long DriveForwardWall(double power, long distance, double targetDistance) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (DriveWall(power, distance, targetDistance, 1));
    }

    public long DriveBackwardWall(double power, long distance, double targetDistance) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (DriveWall(power, distance, targetDistance, -1));
    }



    double calculateTurnPow(double totalTurn, double currentHeading, double maxPow)
    {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minSpinRampUpPow = .5;
        double minSpinRampDownPow = .5;
        double rampDownCalcPow = 0;
        //number of degrees for speeding up
        double rampUpDegrees = 30;
        //number of degrees for slowing down
        double rampDownDegrees = 30;

        rampUpCalcPow = minSpinRampUpPow + (((maxPow - minSpinRampUpPow) / rampUpDegrees) * currentHeading);
        rampDownCalcPow = minSpinRampDownPow + (((minSpinRampDownPow - maxPow) / rampDownDegrees) * (currentHeading - totalTurn));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        if (calculatedPow < minSpinRampDownPow)
        {
            calculatedPow = minSpinRampDownPow;

        }
        return Range.clip(calculatedPow, -1, 1);
    }

    double calculateDrivePow(double totalDistance, double currentDistance, double maxPow)
    {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minPow = .25;
        double rampDownCalcPow = 0;
        //distance in cm for speeding up
        double rampUpDistance = 20;
        //distance in cm for slowing down
        double rampDownDistance = 40;

        rampUpCalcPow = minPow + (((maxPow - minPow) / rampUpDistance) * currentDistance);
        rampDownCalcPow = minPow + (((minPow - maxPow) / rampDownDistance) * (currentDistance - totalDistance));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        if (calculatedPow < minPow)
        {
            calculatedPow = minPow;

        }
        if (calculatedPow > maxPow)
        {
            calculatedPow = maxPow;
        }
        return Range.clip(calculatedPow, -1, 1);
    }


    private long LineDrive(double power, long distance) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        double currentLight = 0;


        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
        }
        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorBackLeft.getCurrentPosition()) < encoderTarget &&
                Math.abs(motorBackRight.getCurrentPosition()) < encoderTarget)
        {

            currentLight = GetODSValue();
            //calculatedPow = calculateDrivePow(distance, motorFrontLeft.getCurrentPosition()*onemotorclick, power)*direction;
            leftCalculatedPow = Range.clip((-power) - ((0.5 - currentLight) / 2), -1, 0);

            rightCalculatedPow = Range.clip((-power) + ((0.5 - currentLight) / 2), -1, 0);


            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Current Light ", currentLight);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);
            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        return (distance);
    }

    public long LineDriveBackward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (LineDrive(power, distance));
    }


    public void parkrobot()throws InterruptedException
    {
        stallDetectionOn = true;
        DriveBackward(.4, 67);
    }

    public void attackBeacon()throws InterruptedException
    {
        stallDetectionOn = false;
        SpinLeft(.7, 45);
        DriveForward(.9, 130);
    }

    public void  delayRobot(long delay)throws  InterruptedException
    {
        long timer = Calendar.getInstance().getTimeInMillis();

        while (Calendar.getInstance().getTimeInMillis() < timer + delay*1000)
        {
        }
    }

    public void auto_1_Climber(Alliance alliance, int endPos, long delay) throws InterruptedException
    {
        stallDetectionOn = false;

        delayRobot(delay);

        // we should use the ultrasonic to drive to the right distance
        DriveBackward(.3, 217);

        //stallDetectionOn = false;

        SpinRight(.7, 45);
        DriveToUSDistance(16);
        SpinRight(.7, 90);
        DriveForwardToTape(.25, 100, RSRobot.Alliance.WHITE);
        DriveBackward(.5, 12);
        DeliverClimber();
        switch (endPos)
        {
            case park:
                parkrobot();
                break;
            case defenseBeacon:
                attackBeacon();
                break;
            default:
                break;

        }
    }

    public void auto_3_Climber(Alliance alliance,int endPos, long delay) throws InterruptedException
    {

        stallDetectionOn = true;

        delayRobot(delay);

        DriveBackward(.5, 10);

        SpinRight(1, 47);

        DriveBackward(1, 210);

        SpinRight(1, 43);

        //we should use the ultrasonic to drive to the right distance
        DriveToUSDistance(16);

        SpinRight(1, 90);

        DriveForwardToTape(.25, 100, RSRobot.Alliance.WHITE);
        DriveBackward(.5, 12);
        DeliverClimber();
        switch(endPos)
        {
            case park:
                parkrobot();
                break;
            case defenseBeacon:
                attackBeacon();
                break;
            default:
        }
    }

    public void initializeHarvester()
    {
        bucketThread = BucketThreadSingleton.getBucketThread();

        bucketThread.InitializeBucket(motorHarvester);
    }

    public void dropHarvester() throws InterruptedException
    {
        bucketThread.targetPos = -1450;
    }

    public void raiseHarvester() throws InterruptedException
    {
        bucketThread.targetPos = -50;
    }

    public void climberScoreHarvester() throws InterruptedException
    {

        bucketThread.targetPos = -300;
        opMode.sleep(1000);
        /*
        for (double servoPos = .16; servoPos <= .54; servoPos = servoPos + .01)
        {
            servoClimber.setPosition(servoPos);
            opMode.sleep(40);
        }
        */
    }

    /*public void servoClimberScore() throws InterruptedException
    {
        for (double servoPos = .14; servoPos <= .54; servoPos = servoPos + .01)
        {
            servoClimber.setPosition(servoPos);
            opMode.sleep(40);
        }
        opMode.sleep(500);
    }*/

    public void recalibrateHarvester() throws InterruptedException
    {
        //resets the target position to where the bucket is currently (used at ground)
        bucketThread.targetPos = motorHarvester.getCurrentPosition();
    }

    public void reverseSpinner() throws InterruptedException
    {
        motorSpinner.setPower(-1);
    }

    public void intakeSpinner() throws InterruptedException
    {
        motorSpinner.setPower(1);
    }

    public void stopSpinner() throws InterruptedException
    {
        motorSpinner.setPower(0);
    }

    public void startHarvester() throws InterruptedException
    {
        motorHarvester.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        bucketThread.startBucketThread();
        bucketThread.startTimer(32);
    }

    public void stopHarvester() throws InterruptedException
    {
        bucketThread.stopBucketThread();
    }

    public void extendSlide() throws InterruptedException
    {
        motorSlide.setPower(.5);
        opMode.sleep(1000);
        motorSlide.setPower(0);
        opMode.sleep(500);
    }

    public void reverseSlide() throws InterruptedException
    {
        motorSlide.setPower(-.5);
        opMode.sleep(500);
        motorSlide.setPower(0);
    }

    public void servoHopperOut() throws InterruptedException
    {
        if (myAlliance == Alliance.BLUE)
        {
            servoHopper.setPosition(1);
            opMode.sleep(1000);
            servoHopper.setPosition(.5);
        }
        else
        {
            servoHopper.setPosition(0);
            opMode.sleep(1000);
            servoHopper.setPosition(.5);
        }
    }

    public void servoHopperIn() throws InterruptedException
    {
        if (myAlliance == Alliance.BLUE)
        {
            servoHopper.setPosition(0);
            opMode.sleep(1000);
            servoHopper.setPosition(.5);
        }
        else
        {
            servoHopper.setPosition(1);
            opMode.sleep(1000);
            servoHopper.setPosition(.5);
        }
    }

    public void servoDoorOpen() throws InterruptedException
    {
        servoDoor.setPosition(180);
        opMode.sleep(1000);
    }

    public void servoDoorClose() throws InterruptedException
    {
        servoDoor.setPosition(0);
        opMode.sleep(1000);
    }

    //to be run before matches to make sure all motors and servos are working
    public void pre_Match_Test() throws InterruptedException
    {
        opMode.sleep(1000);

        //Test straight driving
        DriveForward(1, 50);
        opMode.sleep(1000);

        DriveBackward(1, 50);
        opMode.sleep(1000);

        //test harvester bucket
        dropHarvester();
        opMode.sleep(3000);

        //test spinner
        intakeSpinner();
        opMode.sleep(500);

        reverseSpinner();
        opMode.sleep(500);

        stopSpinner();

        //move scoring slide
        extendSlide();
        opMode.sleep(500);

        //move scoring bucket servo
        servoHopperOut();

        //test door servo
        servoDoorOpen();
        opMode.sleep(500);

        servoDoorClose();
        opMode.sleep(500);

        servoHopperIn();

        //move climber dumper

        //move servoZip

        //move hanger thingy

        //move button presser
    }


    // RSColor
    // red
    // blue
    // other
    private Alliance GetColorRGB()
    {
        int redTapeThreshold = 4;
        int blueTapeThreshold = 4;
        Alliance allianceColor;
        Log.d("RGB:", "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Blue " + sensorRGB.blue());
        Log.d("RGB:", "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Red  " + sensorRGB.red());

        if (sensorRGB.red() > redTapeThreshold && sensorRGB.blue() < blueTapeThreshold)
        {
            allianceColor = Alliance.RED;
        }
        else if (sensorRGB.blue() > blueTapeThreshold && sensorRGB.red() < redTapeThreshold)
        {
            allianceColor = Alliance.BLUE;
        }
        else if (sensorRGB.blue() > blueTapeThreshold && sensorRGB.red() > redTapeThreshold)
        {
            allianceColor = Alliance.WHITE;
        }
        else
        {
            allianceColor = Alliance.UNKNOWN;
        }
        return allianceColor;
    }

    // intensity value from ODS 0.0-1.0
    public double GetODSValue()
    {
        Log.d("ODS:", "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ " + sensorODS.getLightDetected());
        return sensorODS.getLightDetected();
    }

    private long Spin(double power, double degrees, double direction) throws InterruptedException
    {
        double calculatedPow = 0;
        //set current heading to zero
        ResetCurrentHeading();

        // fudge heading to account for gyro drift
        degrees = degrees - (degrees / 45);

        Log.d("@@@@@@@@@@@degrees:", "" + degrees);
        Log.d("@@@@@@@@@@@turn error:", "" + turnError);

        degrees = degrees - (turnError * direction);

        Log.d("@@@@@@@@@@@adj degr:", "" + degrees);

        //use a while loop to keep motors going until desired heading reached
        while (Math.abs(GetCurrentHeading()) < (degrees - 5))
        {
            // calculatedPow = (calculateTurnPow(degrees, GetCurrentHeading(), power))*direction*myAlliance.alliance;
            calculatedPow = power * direction * myAlliance.alliance;
            motorFrontRight.setPower(-calculatedPow);
            motorBackRight.setPower(-calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);

            opMode.telemetry.addData("curr heading ", GetCurrentHeading());
            opMode.telemetry.addData("pow ", calculatedPow);

            Log.d("@@@@@@@@@@@@@@@Heading:", "" + GetCurrentHeading());

            opMode.waitForNextHardwareCycle();

        }
        Log.d("@@@@@@@@@@@tgt head:", "" + GetCurrentHeading());

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        opMode.waitForNextHardwareCycle();
        opMode.sleep(300);

        Log.d("@@@@@@@@@@@final head:", "" + GetCurrentHeading());
        turnError = GetCurrentHeading() - (degrees * direction * myAlliance.alliance);

        Log.d("@@@@@@@@@@@final error:", "" + turnError);


        return (long) Math.abs(GetCurrentHeading());
    }

    public long SpinRight(double power, long degrees) throws InterruptedException
    {
        //Calling spin function and direction 1 is right
        return (Spin(power, degrees, 1));
    }

    public long SpinLeft(double power, long degrees) throws InterruptedException
    {
        //Calling spin function and direction -1 is left
        return (Spin(power, degrees, -1));

    }


    public void DeliverClimber() throws InterruptedException
    {

        servoClimberRight.setPosition(opMode.servoClimberRightOpen);
        servoClimberLeft.setPosition(opMode.servoClimberLeftOpen);
        opMode.sleep(800);
        servoClimberRight.setPosition(opMode.servoClimberRightClose);
        servoClimberLeft.setPosition(opMode.servoClimberLeftClose);


    }

}

/*

    private long DrivePID(double power, long distance, double direction) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;

        Log.d("@@@@@@@@@@@@@@@Drive: ", "" + distance * direction);
        Log.d("@@@Drive: Target head ", "" + targetHeading);

        //set current heading to zero
        ResetCurrentHeading();
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        int counter = 0;
        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
            opMode.telemetry.addData("Waiting left ", motorBackLeft.getCurrentPosition());
            opMode.telemetry.addData("Waiting right ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Power left ", motorBackLeft.getPower());
            opMode.telemetry.addData("Power right ", motorBackRight.getPower());
            counter ++;
            if (counter%100 == 0)
            {
                motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                counter = 0;
            }
        }


        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        //motorBackLeft.setTargetPosition((int) encoderTarget);
        //motorBackRight.setTargetPosition((int) encoderTarget);

        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        //  motorBackLeft.setPower(power);
        //  motorBackRight.setPower(power);

        while (motorBackLeft.getCurrentPosition() < encoderTarget && motorBackRight.getCurrentPosition() < encoderTarget)
        {

            currentHeading = GetCurrentHeading();
            Log.d("@@@Heading ", "" + currentHeading);

            calculatedPow = calculateDrivePow(distance, Math.abs(motorBackLeft.getCurrentPosition()) * onemotorclick, power);

            Log.d("@@@Calc Pww ", "" + calculatedPow);

//            leftCalculatedPow = Range.clip((calculatedPow * direction) - (currentHeading / 10), -1, 1);

//            rightCalculatedPow = Range.clip((calculatedPow * direction) + (currentHeading / 10), -1, 1);

            //leftCalculatedPow = LeftPowerCalc(calculatedPow, direction, 0);

            motorFrontLeft.setPower(calculatedPow);
            motorFrontRight.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);
            motorBackRight.setPower(calculatedPow);


            // Log.d("@@@Left Calc Pow ", "" + calculatedPow);
            //  Log.d("@@@Right Calc Pow ", "" + calculatedPow);


            opMode.telemetry.addData("Current Encoder Position ", motorBackLeft.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", motorBackLeft.getPower());
            opMode.telemetry.addData("right power ", motorBackRight.getPower());
            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();
        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        return (distance);
    }

    public long DrivePIDForward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (DrivePID(power, distance, 1));
    }

    public long DrivePIDBackward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (DrivePID(power, distance, -1));
    }



    private long DriveStallDetection(double power, long distance, double direction) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos = 0;
        int rightMotorPos = 0;
        int leftStallCutoff;
        int rightStallCutoff;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftMagicNumberofDeath = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightMagicNumberofDeath = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;


        //set current heading to zero
        ResetCurrentHeading();
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
        }
        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorBackLeft.getCurrentPosition()) < encoderTarget &&
                Math.abs(motorBackRight.getCurrentPosition()) < encoderTarget &&
                !isLeftMotorStalled &&
                !isRightMotorStalled)
        {

            currentHeading = GetCurrentHeading();

            calculatedPow = calculateDrivePow(distance, motorBackLeft.getCurrentPosition() * onemotorclick, power);

            leftCalculatedPow = Range.clip((calculatedPow * direction) - (currentHeading / 10), -1, 1);

            rightCalculatedPow = Range.clip((calculatedPow * direction) + (currentHeading / 10), -1, 1);


            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);

            leftMotorPos = motorBackLeft.getCurrentPosition();
            rightMotorPos = motorBackRight.getCurrentPosition();
            leftStallCutoff = leftMagicNumberofDeath;
            rightStallCutoff = rightMagicNumberofDeath;

//			nxtDisplayBigTextLine( 5, "L %d", leftMotorPos );
            //		nxtDisplayTextLine( 7, "R %d", rightMotorPos );

            //if trying to move and not successfully moving
            if ((leftCalculatedPow != 0) && (Math.abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
            {
                //if stalling for 50 rounds through the loop (.5 second)
                if (++leftStallCount == 20)
                {
                    //left motor has stalled.
                    isLeftMotorStalled = true;

                }
            }
            else
            {
                // not stalled, reset stall counter
                leftStallCount = 0;

                isLeftMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            leftStallPos = leftMotorPos;

            //if trying to move and not successfully moving
            if ((rightCalculatedPow != 0) && (Math.abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
            {
                //if stalling for 50 rounds through the loop
                if (++rightStallCount == 20)
                {
                    //right motor has stalled.
                    isRightMotorStalled = true;


                }
            }
            else
            {
                // not stalled, reset stall counter
                rightStallCount = 0;

                isRightMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            rightStallPos = rightMotorPos;

            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        if (isLeftMotorStalled || isRightMotorStalled)
        {
            if (leftMotorPos > rightMotorPos)

                distance = (long) (leftMotorPos * onemotorclick);
            else
                distance = (long) (rightMotorPos * onemotorclick);
        }

        return (distance);
    }

    public long DriveForwardStallDetection(double power, long distance) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (DriveStallDetection(power, distance, 1));
    }

    public long DriveBackwardStallDetection(double power, long distance) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (DriveStallDetection(power, distance, -1));
    }


private long DriveStallDetectionToTape(double power, long distance, Alliance tapeColor, double direction) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos = 0;
        int rightMotorPos = 0;
        int leftStallCutoff;
        int rightStallCutoff;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftMagicNumberofDeath = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightMagicNumberofDeath = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        double leftEncoderTarget;
        double rightEncoderTarget;
        double leftEncoderStart;
        double rightEncoderStart;
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;
        boolean tapeFound = false;

        Log.d("XXXXXXXXXXXXX", "Drive to Tape");


        //set current heading to zero
        ResetCurrentHeading();
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        Log.d("XXXXXXXXXXXXX", "TRYING TO RESET ENCODERS");
//        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        while (motorBackRight.getCurrentPosition() != 0 || motorBackLeft.getCurrentPosition() != 0)
//        {
//            opMode.waitForNextHardwareCycle();
//        }

        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        opMode.waitForNextHardwareCycle();

        leftEncoderTarget = direction * distance / onemotorclick;
        rightEncoderTarget = distance / onemotorclick;

        leftEncoderStart = motorBackLeft.getCurrentPosition() * direction;
        rightEncoderStart = motorBackRight.getCurrentPosition();
        Log.d("XXXXXXXXXXXXX", "leftEncoderTarget = " + leftEncoderTarget);
        Log.d("XXXXXXXXXXXXX", "leftEncoderStart = " + leftEncoderStart);
        Log.d("XXXXXXXXXXXXX", "rightEncoderTarget = " + rightEncoderTarget);
        Log.d("XXXXXXXXXXXXX", "rightEncoderStart = " + rightEncoderStart);

        while (Math.abs(motorBackLeft.getCurrentPosition()) < Math.abs(leftEncoderStart + leftEncoderTarget) &&
                Math.abs(motorBackRight.getCurrentPosition()) < Math.abs(rightEncoderStart + rightEncoderTarget) &&
                !isLeftMotorStalled &&
                !isRightMotorStalled &&
                !tapeFound)

        {

            currentHeading = GetCurrentHeading();

            calculatedPow = calculateDrivePow(distance, (motorBackLeft.getCurrentPosition() - leftEncoderStart) * onemotorclick, power);

            leftCalculatedPow = Range.clip((calculatedPow * direction) - (currentHeading / 10), -1, 1);

            rightCalculatedPow = Range.clip((calculatedPow * direction) + (currentHeading / 10), -1, 1);


            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);

            leftMotorPos = motorBackLeft.getCurrentPosition();
            rightMotorPos = motorBackRight.getCurrentPosition();
            leftStallCutoff = leftMagicNumberofDeath;
            rightStallCutoff = rightMagicNumberofDeath;

//			nxtDisplayBigTextLine( 5, "L %d", leftMotorPos );
            //		nxtDisplayTextLine( 7, "R %d", rightMotorPos );

            //if trying to move and not successfully moving
            if ((leftCalculatedPow != 0) && (Math.abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
            {
                //if stalling for 50 rounds through the loop (.5 second)
                if (++leftStallCount == 20)
                {
                    //left motor has stalled.
                    isLeftMotorStalled = true;

                }
            }
            else
            {
                // not stalled, reset stall counter
                leftStallCount = 0;

                isLeftMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            leftStallPos = leftMotorPos;

            //if trying to move and not successfully moving
            if ((rightCalculatedPow != 0) && (Math.abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
            {
                //if stalling for 50 rounds through the loop
                if (++rightStallCount == 20)
                {
                    //right motor has stalled.
                    isRightMotorStalled = true;


                }
            }
            else
            {
                // not stalled, reset stall counter
                rightStallCount = 0;

                isRightMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            rightStallPos = rightMotorPos;

            //Check color sensor for alliance color tape

            if (GetColorRGB() == tapeColor)
            {
                Log.d("XXXXXXXXXXXXXXXXXXXX", "TapeFound");
                tapeFound = true;
            }

            opMode.waitForNextHardwareCycle();
            Log.d("XXXXXXXXXXXXX", "leftMotorPos = " + motorBackLeft.getCurrentPosition());
            Log.d("XXXXXXXXXXXXX", "rightMotorPos = " + motorBackRight.getCurrentPosition());
        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        //  if (isLeftMotorStalled || isRightMotorStalled)
        // {
        if (leftMotorPos > rightMotorPos)

            distance = (long) ((leftMotorPos - leftEncoderStart) * onemotorclick);
        else
            distance = (long) ((rightMotorPos - rightEncoderStart) * onemotorclick);
        // }

        Log.d("XXXXXXXXXXXXX", "leftMotorPos = " + motorBackLeft.getCurrentPosition());
        Log.d("XXXXXXXXXXXXX", "rightMotorPos = " + motorBackRight.getCurrentPosition());

        Log.d("XXXXXXXXXXXXXXXXXXXXX", "Drive to Tape Distance = " + distance);

        return (distance);
    }

    public long DriveForwardStallDetectionToTape(double power, long distance, Alliance tapeColor) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (DriveStallDetectionToTape(power, distance, tapeColor, 1));
    }

    public long DriveBackwardStallDetectionToTape(double power, long distance, Alliance tapeColor) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (DriveStallDetectionToTape(power, distance, tapeColor, -1));
    }

*/
/*    public void auto_3_PZ(Alliance alliance, long delay) throws InterruptedException
    {
        long distanceDriven = 0;
        setMyAlliance(alliance);

        opMode.sleep(delay * 1000);

        //start moving
        dropHarvester();

        opMode.sleep(3000);

        recalibrateHarvester();

        reverseSpinner();

        // distanceDriven = DriveBackwardStallDetection(.5, 330);
        DriveBackward(.5, 210);
        //if(distanceDriven < 330)
        //{
        //    stopSpinner();
        //    raiseHarvester();
        //    DriveForward(.5, 30);
        //    dropHarvester();
        //    opMode.sleep(3000);
        //    reverseSpinner();
        //    DriveBackward(.5, 330-distanceDriven + 30);
        //}


        stopSpinner();

        raiseHarvester();

        opMode.sleep(3000);

        DriveBackward(.5, 75);
    }
*/
 /*   public void auto_3_OMC(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);

        dropHarvester();

        opMode.sleep(3000);

        recalibrateHarvester();

        reverseSpinner();


        //robot.SpinRight(1, 360);
        DriveBackward(1, 45);

        opMode.sleep(1500);

        SpinRight(1, 90);

        opMode.sleep(1000);

        DriveBackward(1, 120);

        opMode.sleep(1000);

        stopSpinner();

        raiseHarvester();

        opMode.sleep(3000);

        SpinLeft(1, 135);

        opMode.sleep(1000);

        DriveForward(1, 150);


    }
*/
   /* public void auto_1_OMF(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);

        DriveForward(1.0, 85);

        opMode.sleep(1000);

        SpinRight(1, 45);

        opMode.sleep(1000);

        DriveForward(1.0, 60);

        opMode.sleep(1000);

        SpinRight(1, 90);

        opMode.sleep(1000);

        DriveForward(1, 300);
    }

    public void auto_1_PZ(Alliance alliance, long delay) throws InterruptedException
    {
        long distanceDriven = 0;
        setMyAlliance(alliance);

        opMode.sleep(delay * 1000);

        DriveBackward(.5, 30);
        opMode.sleep(1000);

        DriveForward(.5, 25);

        //start moving
        dropHarvester();

        opMode.sleep(3000);

        recalibrateHarvester();

        reverseSpinner();

        distanceDriven = DriveBackward(.5, 165);
        // DriveBackward(.5, 170);
        if (distanceDriven < 165)
        {
            stopSpinner();
            raiseHarvester();
            DriveForward(.5, 30);
            dropHarvester();
            opMode.sleep(3000);
            reverseSpinner();
            DriveBackward(.5, 165 - distanceDriven + 30);
        }


        stopSpinner();

        raiseHarvester();

        opMode.sleep(3000);

        DriveBackward(.5, 10);
    }

    public void auto_3_Defense(Alliance alliance, long delay) throws InterruptedException
    {
        long timer;

        timer = Calendar.getInstance().getTimeInMillis();

        setMyAlliance(alliance);

        opMode.sleep(delay * 1000);

        DriveBackward(1, 310);

        SpinLeft(.6, 90);

        while (Calendar.getInstance().getTimeInMillis() < timer + 10000)
        {
        }

        DriveBackward(1, 150);
    }


    public void auto_3_Defense2(Alliance alliance, long delay) throws InterruptedException
    {
        long timer = Calendar.getInstance().getTimeInMillis();

        setMyAlliance(alliance);

        opMode.sleep(delay * 1000);

        while (Calendar.getInstance().getTimeInMillis() < timer + 10000)
        {
        }

        DriveBackward(.7, 320);
    }

*/

   /* public void auto_1_Climber(Alliance alliance, long delay) throws InterruptedException
    {

        long distanceDriven;
        long distanceToWhite;

        setMyAlliance(alliance);

        DriveBackward(.3, 90);
        opMode.sleep(500);

        SpinRight(.7, 45);
        opMode.sleep(500);

        DriveBackward(.3, 75);
        opMode.sleep(500);

        SpinRight(.7, 45);
        opMode.sleep(500);

        DriveBackwardToTape(.2, 40, alliance);
        opMode.sleep(500);

        DriveBackward(.5, 25);
        opMode.sleep(500);

        SpinLeft(1, 90);
        opMode.sleep(500);

        DriveBackwardToTape(.2, 100, Alliance.WHITE);
        opMode.sleep(500);

        DriveBackward(.5, 5);
        opMode.sleep(500);

        climberScoreHarvester();
        // opMode.sleep(1000);

        SpinRight(.7, 90);
        opMode.sleep(500);

        DriveBackward(.3, 20);
        opMode.sleep(500);

        //servoClimberScore();
        opMode.sleep(500);
        // SpinRight(.7, 30);
        // opMode.sleep(500);

        bucketThread.targetPos = -620;
        opMode.sleep(1000);

        DriveForward(.3, 10);
        opMode.sleep(500);

        DriveForward(.3, 25);
        opMode.sleep(500);

        //servoClimber.setPosition(0.16);

        raiseHarvester();

        DriveBackward(.5, 20);
        opMode.sleep(2000);



        /*

       // opMode.sleep(delay * 1000);



        DriveBackward(.4, 80);
       opMode.sleep(1000);

        SpinRight(.5, 45);
        opMode.sleep(500);

        //fast drive towards tape
        DriveBackward(1, 91);
        opMode.sleep(500);

        //drive to alliance tape
        distanceDriven = DriveBackwardStallDetectionToTape(.2, 60, alliance);
      //  opMode.sleep(1000);

        if (distanceDriven < 60)
        {
            //alliance tape was found
            distanceToWhite = 61;
        }
        else // alliance tape was not found
            distanceToWhite = 30;
        opMode.sleep(500);
        DriveBackwardStallDetectionToTape(.2, distanceToWhite, Alliance.WHITE);
        opMode.sleep(500);

        DriveBackward(.5, 10);
        opMode.sleep(500);

        climberScoreHarvester();

        //spin right to face the beacon
        SpinRight(.45, 45);

        opMode.sleep(500);
       // opMode.sleep(3000);

    }
    */

    /*public void auto_3_Climber(Alliance alliance, long delay) throws InterruptedException
    {
        setMyAlliance(alliance);

        //drive away from wall
        DriveBackward(.5, 10);
        opMode.sleep(500);

        //spin to face back towards parking zone
        SpinRight(.6, 52);
        opMode.sleep(500);

        //drive quickly towards parking zone
        DriveBackward(1, 200);
        opMode.sleep(500);

        //drive to alliance tape
        DriveBackwardToTape(.3, 40, alliance);
        opMode.sleep(500);

        //spin to face back towards wall
        SpinRight(.6, 38);
        opMode.sleep(500);

        //drive back towards wall
        DriveBackward(.5, 20);
        opMode.sleep(500);

        //spin to face back towards white tape
        SpinLeft(.8, 90);
        opMode.sleep(500);

        //drive towards white tape
        DriveBackwardToTape(.3, 100, Alliance.WHITE);
        opMode.sleep(500);

        //drive a little past white tape
        DriveBackward(.5, 5);
        opMode.sleep(500);

        climberScoreHarvester();
        // opMode.sleep(1000);

        //spin to face back towards beacon
        SpinRight(.6, 90);
        opMode.sleep(500);

        //drive back towards beacon
        DriveBackward(.3, 25);
        opMode.sleep(500);

        //servoClimberScore();
        opMode.sleep(250);
        // SpinRight(.7, 30);
        // opMode.sleep(500);

        bucketThread.targetPos = -620;
        opMode.sleep(1000);

        //drive away from beacon to release climbers
        DriveForward(.3, 35);
        opMode.sleep(500);

        //put servo climber into away position
        //servoClimber.setPosition(0.16);

        //put harvester into raised position
        raiseHarvester();

        //drive toward wall to make sure we are in parking zone at the end of autonomous
        DriveBackward(.5, 20);
        opMode.sleep(2000);

    }
*/

/*    public void auto_1_OMC(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);


        //robot.SpinRight(1, 360);
        DriveForward(1.0, 85);

        opMode.sleep(1000);

        SpinRight(1, 135);

        opMode.sleep(1000);

        DriveForward(1.0, 300);
    }


    public void auto_3_OMF(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);


        //robot.SpinRight(1, 360);
        DriveForward(1.0, 20);

        opMode.sleep(1500);

        SpinRight(1, 45);

        opMode.sleep(1000);

        DriveForward(1.0, 175);

        opMode.sleep(1000);

        SpinRight(1, 87);

        opMode.sleep(1000);

        DriveForward(1, 300);
    }
*/
    /*    private long Spin(double power, long degrees, double direction) throws InterruptedException
        {

            Log.d("@@@@@@@@@@@@@@@Spin:", "" + degrees * direction);
            ChangeTargetHeading(degrees * direction);
            Log.d("@@@@@@@@@@@Spin: Target", "" + targetHeading);
            double calculatedPow = 0;
            //use a while loop to keep motors going until desired heading reached
            while (Math.abs(GetDeltaHeading()) > 8)
            {
                // calculatedPow = (calculateTurnPow(degrees, GetCurrentHeading(), power))*direction*myAlliance.alliance;
                calculatedPow = power * direction;
                motorFrontRight.setPower(-calculatedPow);
                motorBackRight.setPower(-calculatedPow);
                motorFrontLeft.setPower(calculatedPow);
                motorBackLeft.setPower(calculatedPow);

                opMode.telemetry.addData("curr heading ", GetCurrentHeading());
                opMode.telemetry.addData("target heading ", targetHeading);
                opMode.telemetry.addData("delta heading ", GetDeltaHeading());
                opMode.telemetry.addData("pow ", calculatedPow);


                Log.d("@@@@@@@@@@@@@@@Heading:", "" + GetCurrentHeading());
                Log.d("@@@@@@@@@@@@@@@Delta:", "" + GetDeltaHeading());

                opMode.waitForNextHardwareCycle();

            }
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);


            opMode.waitForNextHardwareCycle();

            opMode.sleep(200);

            Log.d("@@@@@@@@@Final Heading:", "" + GetCurrentHeading());
            Log.d("@@@@@@@@@Final Delta:", "" + GetDeltaHeading());

            return (long) Math.abs(GetCurrentHeading());
        }

        public long SpinRight(double power, long degrees) throws InterruptedException
        {
            //Calling spin function and direction 1 is right
            if (myAlliance == Alliance.BLUE)
                return (Spin(power, degrees, 1));
            else
                return (Spin(power, degrees, -1));
        }

        public long SpinLeft(double power, long degrees) throws InterruptedException
        {
            //Calling spin function and direction -1 is left
            if (myAlliance == Alliance.BLUE)
                return (Spin(power, degrees, -1));
            else
                return (Spin(power, degrees, 1));
        }

    */
   /* private long ArcTurn(double power, long degrees, int turnDirection, int driveDirection) throws InterruptedException
    {
        double calculatedPow = 0;
        //set current heading to zero
        ResetCurrentHeading();
        //sets turn direction based on blue alliance so red alliance will swap turn direction
        turnDirection = turnDirection * (int) myAlliance.alliance;
        //use a while loop to keep motors going until desired heading reached
        while (Math.abs(GetCurrentHeading()) < (degrees - 3))
        {
            if (driveDirection == forward)
            {
                calculatedPow = power;
                if (turnDirection == right)
                {
                    motorFrontRight.setPower(-calculatedPow);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(calculatedPow);
                    motorBackLeft.setPower(calculatedPow);
                }
                else
                {
                    motorFrontRight.setPower(calculatedPow);
                    motorBackRight.setPower(calculatedPow);
                    motorFrontLeft.setPower(-calculatedPow);
                    motorBackLeft.setPower(0);
                }
            }
            else
            {
                calculatedPow = -power;
                if (turnDirection == right)
                {
                    motorFrontRight.setPower(calculatedPow);
                    motorBackRight.setPower(calculatedPow);
                    motorFrontLeft.setPower(-calculatedPow);
                    motorBackLeft.setPower(0);
                }
                else
                {
                    motorFrontRight.setPower(-calculatedPow);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(calculatedPow);
                    motorBackLeft.setPower(calculatedPow);
                }
            }


            opMode.telemetry.addData("curr heading ", GetCurrentHeading());

            opMode.waitForNextHardwareCycle();

        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        opMode.waitForNextHardwareCycle();
        return (long) Math.abs(GetCurrentHeading());
    }

    public long ArcTurnBackRight(double power, long degrees) throws InterruptedException
    {
        //
        //Calling spin function and direction 1 is right
        return (ArcTurn(power, degrees, right, backward));
    }

    public long ArcTurnBackLeft(double power, long degrees) throws InterruptedException
    {
        //Calling spin function and direction -1 is left
        return (ArcTurn(power, degrees, left, backward));

    }

    public long ArcTurnFrontRight(double power, long degrees) throws InterruptedException
    {
        //
        //Calling spin function and direction 1 is right
        return (ArcTurn(power, degrees, right, forward));
    }

    public long ArcTurnFrontLeft(double power, long degrees) throws InterruptedException
    {
        //Calling spin function and direction -1 is left
        return (ArcTurn(power, degrees, left, forward));

    }
*/

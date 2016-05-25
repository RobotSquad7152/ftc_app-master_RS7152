package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import RobotSquad.RSRobot;

/**
 * Created by Tony on 2/16/2016.
 */
public abstract class RSLinearOpMode extends LinearOpMode
{
    public RSRobot robot;

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorSlide;
    public DcMotor motorSpinner;
    public DcMotor motorTape;
    public DcMotor motorBucket;
    public Servo servoChurro;
    public Servo servoHopper;
    public Servo servoClimberRight;
    public Servo servoClimberLeft;
    public Servo servoDoor;
    public Servo servoLeftZip;
    public Servo servoRightZip;
    public Servo servoClutch;
    public Servo servoTape;


    public DcMotorController motorControllerFrontDrive;
    public DcMotorController motorControllerRearDrive;
    public ModernRoboticsI2cGyro gyro;
    public ColorSensor sensorRGB;
    public UltrasonicSensor ultrasonic;
    public OpticalDistanceSensor ods;
    public double servoChurroUp = 0.15;
    public double servoHopperPos = .5;
    public double servoDoorClose = 0;
    public double servoClimberRightClose = .03;
    public double servoClimberRightOpen = 0.85;

    public double servoClimberLeftClose = 1;
    public double servoClimberLeftOpen = 0.175;
    //double servoClimberPos = 0;
    double servoTapeMeasurePos = 0.5;
    double servoPosRightZipIn = 0.01;
    double servoPosRightZipOut = 0.77;
    double servoPosLeftZipIn = 0.99;
    double servoPosLeftZipOut = 0.1;
    double servoClutchDisengaged = 0.8;

    @Override
    public abstract void runOpMode() throws InterruptedException;

    public RSRobot InitHardware()
    {
        //initialize motors
        motorFrontLeft = hardwareMap.dcMotor.get("motor_fl");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight = hardwareMap.dcMotor.get("motor_fr");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft = hardwareMap.dcMotor.get("motor_bl");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight = hardwareMap.dcMotor.get("motor_br");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorSlide = hardwareMap.dcMotor.get("motor_slide");
        motorSlide.setDirection(DcMotor.Direction.FORWARD);

        motorTape = hardwareMap.dcMotor.get("motor_tape");
        motorTape.setDirection(DcMotor.Direction.FORWARD);

        motorBucket = hardwareMap.dcMotor.get("motor_harv_arm");
        motorBucket.setDirection(DcMotor.Direction.FORWARD);

        motorSpinner = hardwareMap.dcMotor.get("motor_spinner");
        motorSpinner.setDirection(DcMotor.Direction.FORWARD);

        servoDoor = hardwareMap.servo.get("servo_door");
        servoDoor.setPosition(servoDoorClose);

        servoHopper = hardwareMap.servo.get("servo_hopper");
        servoHopper.setPosition(servoHopperPos);

        servoChurro = hardwareMap.servo.get("servo_churro");
        servoChurro.setPosition(servoChurroUp);

        servoClimberRight = hardwareMap.servo.get("servo_climber_r");
        servoClimberRight.setPosition(servoClimberRightClose);

        servoClimberLeft = hardwareMap.servo.get("servo_climber_l");
        servoClimberLeft.setPosition(servoClimberLeftClose);

        servoClutch = hardwareMap.servo.get("servo_clutch");
        servoClutch.setPosition(servoClutchDisengaged);

        servoTape = hardwareMap.servo.get("servo_tape");
        servoTape.setPosition(servoTapeMeasurePos);

        servoRightZip = hardwareMap.servo.get("servo_rightzip");
        servoRightZip.setPosition(servoPosRightZipIn);

        servoLeftZip = hardwareMap.servo.get("servo_leftzip");
        servoLeftZip.setPosition(servoPosLeftZipIn);

        motorControllerFrontDrive = hardwareMap.dcMotorController.get("frontdrive");
        motorControllerRearDrive = hardwareMap.dcMotorController.get("reardrive");


        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        sensorRGB = hardwareMap.colorSensor.get("color");

        ods = hardwareMap.opticalDistanceSensor.get("ods");

        ultrasonic = hardwareMap.ultrasonicSensor.get("ultrasonic");

        robot = new RSRobot(gyro);


        // pass motor objects to robot
        robot.SetFrontRightMotor(motorFrontRight);
        robot.SetFrontLeftMotor(motorFrontLeft);
        robot.SetBackRightMotor(motorBackRight);
        robot.SetBackLeftMotor(motorBackLeft);
        robot.setMotorControllerFrontDrive(motorControllerFrontDrive);
        robot.setMotorControllerRearDrive(motorControllerRearDrive);
        robot.SetHarvesterMotor(motorBucket);
        robot.SetSpinnerMotor(motorSpinner);
        robot.SetColorSensor(sensorRGB);
        robot.SetClimberRightServo(servoClimberRight);
        robot.SetClimberLeftServo(servoClimberLeft);
        robot.SetODS(ods);
        robot.SetUltrasonic(ultrasonic);

        robot.setOpMode(this);

        // calibrate gyro etc.
        robot.Initialize();

        robot.initializeHarvester();

        return robot;
    }
}



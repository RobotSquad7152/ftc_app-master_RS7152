package RobotSquad;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.Calendar;

/**
 * Created by Tony on 1/30/2016.
 */
public class BucketThreadSingleton extends Thread
{

    DcMotor motorHarvester;
    static int currentPos = 0;
    volatile public static int targetPos = 0;
    static volatile boolean stopped = false;
    static long runTime;
    static boolean autonomousTimer = false;

    private BucketThreadSingleton()
    {

    }


    private static BucketThreadSingleton bucketThread;


    public static synchronized BucketThreadSingleton getBucketThread()
    {
        {
            if (bucketThread == null)
            {
                bucketThread = new BucketThreadSingleton();
            }
            return bucketThread;
        }
    }

    public static synchronized void startBucketThread()
    {
        {
            if (bucketThread == null)
            {
                bucketThread = new BucketThreadSingleton();
            }
            try
            {
                bucketThread.start();
            } catch (IllegalThreadStateException e)
            {
                stopped = false;

            }

        }
    }


    public void stopBucketThread()
    {

        stopped = true;
        autonomousTimer = false;
        Log.d("DEBUG", "Stop Bucket Thread");
    }

    public void run()
    {

        while (true)
        {
            if (!stopped)
            {
                if (autonomousTimer && Calendar.getInstance().getTimeInMillis() > runTime)
                {
                    //failsafe to stop the bucket from moving when autonomous ends
                    stopped = true;
                    autonomousTimer = false;
                }
                currentPos = motorHarvester.getCurrentPosition();

                if ((currentPos >= (targetPos - 25)) && (currentPos <= (targetPos + 25)))
                    motorHarvester.setPower(0);
                else if ((currentPos >= (targetPos - 50)) && (currentPos <= targetPos))
                    motorHarvester.setPower(.02);
                else if ((currentPos <= (targetPos + 50)) && (currentPos >= targetPos))
                    motorHarvester.setPower(-0.02);
                else if (currentPos > targetPos)
                    motorHarvester.setPower(-0.2);//.2
                else if (currentPos < targetPos)
                    motorHarvester.setPower(0.2);//.2
            }
            else
                //never move the harvester arm when we tell the thread to stop
                motorHarvester.setPower(0);
            try
            {
                sleep(50);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }

        }


        //  Log.d("DEBUG", "BucketThreadSingleton.RunFinished");
    }

    public void InitializeBucket(DcMotor motor)
    {
        motorHarvester = motor;
        motorHarvester.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorHarvester.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        // wait half a second for encoder to reset
        try
        {
            sleep(500);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }


    }

    public void startTimer(int timeInSeconds)
    {
        runTime = (timeInSeconds * 1000) + Calendar.getInstance().getTimeInMillis();
        autonomousTimer = true;
    }
}

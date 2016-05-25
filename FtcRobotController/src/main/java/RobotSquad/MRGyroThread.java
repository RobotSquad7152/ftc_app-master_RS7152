package RobotSquad;

import com.qualcomm.robotcore.hardware.GyroSensor;

import java.util.Calendar;

/**
 * Created by Tony on 9/20/2015.
 */
public class MRGyroThread extends Thread
{
    double currentHeading;
    double calibratedGyroRotation;
    GyroSensor gyro;

    public MRGyroThread(GyroSensor g)
    {
        gyro = g;
        gyro.resetZAxisIntegrator();
        calibratedGyroRotation = 0;
    }

    public double getCurrentHeading()
    {
        currentHeading = gyro.getHeading();
        if(currentHeading > 180){
            //MR gyro returns 0 through 359.  We need it to return -179 through 180
            currentHeading = currentHeading - 360;

        }
        return currentHeading;}

    public void setCurrentHeading(double currentHeading)
    {
        gyro.resetZAxisIntegrator();
    }


    public void calibrategyro()
    {

        gyro.calibrate();

        //may be needed for finding z value
       /* double totalGyroReadings = 0;

        //set the millisecond timer
        //    long millisStart = Calendar.getInstance().getTimeInMillis();


        // Take 1000 readings and average them out
        for (int i = 0; i < 500; i++)
        {
            // Wait until 3ms has passed
            //   while (Calendar.getInstance().getTimeInMillis() < millisStart + 3)
            //       try {
            try
            {
                sleep(5);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            //     } catch (InterruptedException e) {
            //          e.printStackTrace();
            //      }

            // millisStart = Calendar.getInstance().getTimeInMillis();
            totalGyroReadings += gyro.getRotation();
        }

        calibratedGyroRotation = totalGyroReadings / 500;
        */
    }


    public void run()
    {
        long prevTime = Calendar.getInstance().getTimeInMillis();
        long currTime;
        do
        {

            try
            {
                sleep(5);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }

        currTime = Calendar.getInstance().getTimeInMillis();
            // calculate turn based on gyro reading divided by the time taken since the last reading (6ms)
         //   currentHeading = currentHeading + (gyro.getRotation() - calibratedGyroRotation) * ((double)(currTime-prevTime)/1000);
            prevTime = currTime;
        } while (true);

//        Stopwatch stopwatch = new Stopwatch();
//        do
//        {
//
//
//            while (stopwatch.elapsedTime() < 5)
//                try
//                {
//                    sleep(1);
//                } catch (InterruptedException e)
//                {
//                    e.printStackTrace();
//                }
//
//            // Reset the timer
//            stopwatch.Resetstopwatch();
//
//            // calculate turn based on gyro reading divided by the time taken since the last reading (6ms)
//            currentHeading = currentHeading + (gyro.getRotation() - calibratedGyroRotation) * .005;
//        } while (true);

    }
}

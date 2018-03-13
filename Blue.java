package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by P00110437 on 3/12/2018.
 */


@Autonomous
public class Blue extends WiredcatsLinearOpMode {

    public void runOpMode() throws InterruptedException{

        inititalize();

        while(!(isStarted()  || isStopRequested())) {

            idle();

        }

        waitForStart();
        telemetry();
        relicTrackables.activate();
        jewelDetect("Blue");
        sleep(100);
        picto = detectPicto();
        sleep(50);

        //Center
        if(picto == 1){

           Backwards(20, 2500);
           resetDrive();
           sleep(100);
           rotate(90);
           sleep(50);
           AlignLeft.setPosition(AlignLeftFinal);
           driveToLeftWall(0.2, 1000);
           resetDrive();
           sleep(50);
           Dump.setPosition(DumpFinal);
           sleep(400);
           Dump.setPosition(DumpInit);
           Forwards(3, 500);
           resetDrive();
           sleep(50);
           Backwards(5, 500);
           resetDrive();
           sleep(50);
           Forwards(30, 3000);
           resetDrive();
           sleep(50);
           intakeUntilOneGlyph(0.1, 600);
           resetDrive();
           sleep(50);
           Backwards(20, 1500);
           resetDrive();
           sleep(50);
           Left(5, 400);
           resetDrive();
           sleep(50);
           FBByGyro(0.3, 1500);
           resetDrive();
           sleep(50);
           Forwards(2, 200);
           resetDrive();
           sleep(50);
           Dump.setPosition(DumpFinal);
           sleep(400);
           Backwards(3, 150);
           resetDrive();
           sleep(50);
           Forwards(6, 500);
           Dump.setPosition(DumpInit);
           resetDrive();
           sleep(50);



        }

        //Left
        else if(picto == 2){



        }

        //Right
        else if(picto == 3){



        }
        //If nothing is detected, run left column
        else{



        }

        idle();

    }



}

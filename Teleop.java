package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by P00110437 on 3/12/2018.
 */

@TeleOp
public class Teleop extends WiredcatsLinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        inititalize();

        while(!(isStarted()  || isStopRequested())) {
            idle();

        }
        waitForStart();
        
        while(opModeIsActive()){
            //Use Telemetry

            telemetry();
            //Drive Train

            FrontLeft.setPower(Flp);
            FrontRight.setPower(Frp);
            BackLeft.setPower(Blp);
            BackRight.setPower(Brp);

            //Intake
            if(a && intake == 0){
                intake = 1;
            }
            
            else if (!a && intake == 1){
                intakeByPower(1.0);
                intake = 2;
            }
            
            else if(a && intake == 2){
                intake = 3;
            }
            
            else if(!a && intake == 3){
                stopIntake();
                intake = 0;
            }

            //Outtake
            if(b && outtake == 0){
                outtake = 1;
            }
            
            else if (!b && outtake == 1){
                outtakeByPower(-1.0);
                outtake = 2;
            }
            
            else if(b && outtake == 2){
                outtake = 3;
            }
            
            else if(!b && outtake == 3){
                stopIntake();
                outtake = 0;
            }

            //Dumping Toggle
            if(y && dump == 0){
                dump = 1;

            }
            else if(!y && dump == 1){
                Dump.setPosition(DumpFinal);
                dump = 2;
            }
            
            else if(y && dump == 2){
                dump = 3;
            }
            
            else if(!y && dump == 3){
                Dump.setPosition(DumpInit);
                dump = 0;
            }

            liftByPower(leftt - rightt);
        }
    }
}

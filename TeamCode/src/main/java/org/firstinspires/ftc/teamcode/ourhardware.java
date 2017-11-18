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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class ourhardware
{
    /* Public OpMode members. */
    public DcMotor  leftfront  = null;
    public DcMotor  rightfront  = null;
    public DcMotor  leftback  = null;
    public DcMotor  rightback   = null;
    public DcMotor  armmotor    = null;
    public Servo    rightClaw   = null;
    public Servo    lazysusan   = null;


    static final double MIN_ARM_POSITION  = -400;
    static final double MAX_ARM_POSITION  = 400;
    static final int MULTIPLIER  = 2;
    public static final double MID_SERVO       =  0.5 ;
    static final double ARM_UP_POWER    =  0 ;
    static final double ARM_DOWN_POWER  = 1 ;
    static final double jawsopen = 0.7;
    static final double jawsclosed = 0;
    static final double lazyleft = 0.4;
    static final double lazyright = 0.7;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ourhardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        leftfront  = hwMap.get(DcMotor.class, "leftfront");
        rightfront = hwMap.get(DcMotor.class, "rightfront");
        leftback = hwMap.get(DcMotor.class, "leftback");
        rightback = hwMap.get(DcMotor.class, "rightback");
        armmotor = hwMap.get(DcMotor.class, "armmotor");
        leftfront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightfront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftback.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightback.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        armmotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);
        armmotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        rightClaw = hwMap.get(Servo.class, "jaws");
        rightClaw.setPosition(MID_SERVO);
        lazysusan = hwMap.get(Servo.class, "lazysusan");
        lazysusan.setPosition(ARM_DOWN_POWER);

        //servo
        rightClaw= hwMap.servo.get("jaws");
        lazysusan= hwMap.servo.get("lazysusan");

        armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move_arm_function (float stick_y_value) {

        int change_amount =((int)stick_y_value*10*MULTIPLIER);
        int armPosition = ((int) armmotor.getCurrentPosition()+ change_amount);
        int safePosition = ((int)Range.clip(armPosition,MIN_ARM_POSITION,MAX_ARM_POSITION));

        armmotor.setTargetPosition(safePosition);
        armmotor.setPower(.3);

        while (armmotor.isBusy()){
            try {
                Thread.sleep(1);
            }
            catch (InterruptedException e){
                System.out.print("got interrrupted");
            }
            break;
        }
        armmotor.setPower(0);
    }

      /*  public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }*/

    /* We stole this code from https://github.com/ftc-9773/ftc_app_9773/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/TeleOp.java */
    public void mecanumWheelDrive(float strafeDirection, float strafeThrottle, float turnDirection, float turnThrottle) {
        strafeDirection = Range.clip(strafeDirection, -1, 1);
        strafeThrottle = Range.clip(strafeThrottle, -1, 1);
        turnThrottle = Range.clip(turnThrottle, -1, 1);
        turnDirection = Range.clip(turnDirection, -1, 1);

        float frontLeftPwr = (Range.clip(strafeThrottle + strafeDirection, -1, 1) - Range.clip(turnThrottle + turnDirection, -1, 1));
        float frontRightPwr = (Range.clip(strafeDirection - strafeThrottle, -1, 1) + Range.clip(turnThrottle - turnDirection, -1, 1));
        float rearLeftPwr = Range.clip(strafeDirection - strafeThrottle, -1, 1) + Range.clip(turnThrottle + turnDirection, -1, 1);
        float rearRightPwr = Range.clip(strafeThrottle + strafeDirection, -1, 1) - Range.clip(turnThrottle - turnDirection, -1, 1);

        leftfront.setPower(Range.clip(frontLeftPwr, -1, 1));
        rightback.setPower(Range.clip(rearRightPwr, -1, 1));
        rightfront.setPower(Range.clip(frontRightPwr, -1, 1));
        leftback.setPower(Range.clip(rearLeftPwr, -1, 1));
    }
 }

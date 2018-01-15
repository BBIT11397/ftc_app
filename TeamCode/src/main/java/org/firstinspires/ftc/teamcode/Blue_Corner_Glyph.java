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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.ourhardware.PADDLE_IN;
import static org.firstinspires.ftc.teamcode.ourhardware.PADDLE_OUT;
import static org.firstinspires.ftc.teamcode.ourhardware.jawsclosed;
import static org.firstinspires.ftc.teamcode.ourhardware.jawsopen;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue_Corner_Glyph", group="Pushbot")
public class Blue_Corner_Glyph extends LinearOpMode {

    /* Declare OpMode members. */
    ourhardware         robot   = new ourhardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final int        COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Nevrest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("runMode", robot.rightback.getMode());
        telemetry.update();

        telemetry.addData("runMode", robot.leftback.getMode());
        telemetry.update();

        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("newRunMode", robot.rightback.getMode());
        telemetry.update();
        telemetry.addData("newRunMode", robot.leftback.getMode());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.rightClaw.setPosition(jawsclosed);
        robot.leftClaw.setPosition(jawsopen);
        sleep(1);

        if (opModeIsActive()){
            robot.jewelDown();
            sleep(750);
        }

        int blueValue = robot.checkForBlue();
        int redValue = robot.checkForRed();

        telemetry.addData("blue", robot.checkForBlue());
        telemetry.addData("red", robot.checkForRed());
        telemetry.update();

        if (redValue > blueValue){
            robot.mecanumWheelDrive(0,0,.25f, 0);
            sleep(250);
            robot.mecanumWheelDrive(0,0,0,0);

            robot.mecanumWheelDrive(0,0,-.25f, 0);
            sleep(250);
            robot.mecanumWheelDrive(0,0,0,0);

            if (opModeIsActive()){
                robot.jewelUp();
                sleep(750);
            }
        }

        else  {
            robot.mecanumWheelDrive(0,0,-.25f,0);
            sleep(250);
            robot.mecanumWheelDrive(0,0,0,0);

            robot.mecanumWheelDrive(0,0,.25f, 0);
            sleep(250);
            robot.mecanumWheelDrive(0,0,0,0);

            if (opModeIsActive()){
                robot.jewelUp();
                sleep(750);
            }
        }

        if (!opModeIsActive()){
            return;
        }

        //Move Forward 30 Inches
        int MOVE_INCHES = (int)COUNTS_PER_INCH * 30;
        robot.leftback.setTargetPosition(-MOVE_INCHES);
        robot.rightback.setTargetPosition(MOVE_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.5);
        robot.rightback.setPower(0.5);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Driving Forward");
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        sleep(500);

        //Move Backwards Inches
        int BACKWARD_INCHES = (int)COUNTS_PER_INCH * 5;
        robot.leftback.setTargetPosition(BACKWARD_INCHES);
        robot.rightback.setTargetPosition(-BACKWARD_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.5);
        robot.rightback.setPower(0.5);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Driving Backward");
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Move Forward 5 Inches
        int FORWARD_7_INCHES = (int)COUNTS_PER_INCH * 7;
        robot.leftback.setTargetPosition(-FORWARD_7_INCHES);
        robot.rightback.setTargetPosition(FORWARD_7_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.5);
        robot.rightback.setPower(0.5);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Driving Forward");
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turn 90 degrees
        int PIVOT_RIGHT= (int)COUNTS_PER_INCH * 9;
        robot.leftback.setTargetPosition(-PIVOT_RIGHT);
        robot.rightback.setTargetPosition(-PIVOT_RIGHT);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.25);
        robot.rightback.setPower(0.25);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Turning 90 Degrees");
            telemetry.addData("left encoder", robot.leftback.getCurrentPosition());
            telemetry.addData("right encoder", robot.rightback.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Move Forward Inches
        int FORWARD4_INCHES = (int)COUNTS_PER_INCH * 10;
        robot.leftback.setTargetPosition(-FORWARD4_INCHES);
        robot.rightback.setTargetPosition(FORWARD4_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.5);
        robot.rightback.setPower(0.5);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Driving Forward");
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turn 90 degrees left
        int PIVOT2_INCHES = (int)COUNTS_PER_INCH * 10;
        robot.leftback.setTargetPosition(PIVOT2_INCHES);
        robot.rightback.setTargetPosition(PIVOT2_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.25);
        robot.rightback.setPower(0.25);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Turning 90 Degrees");
            telemetry.addData("left encoder", robot.leftback.getCurrentPosition());
            telemetry.addData("right encoder", robot.rightback.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        telemetry.addData("Arm Position", robot.armmotor.getCurrentPosition());
        telemetry.update();

        robot.rightClaw.setPosition(jawsopen);
        robot.leftClaw.setPosition(jawsclosed);
        sleep(1);

        robot.armmotor.setTargetPosition(-300);
        robot.armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armmotor.setPower(0.5);
        sleep(500);

        robot.armmotor.setPower(0);

        telemetry.addData("Arm Position", robot.armmotor.getCurrentPosition());
        telemetry.update();

        robot.paddleArm.setPosition(PADDLE_OUT);
        sleep(500);
        robot.paddleArm.setPosition(PADDLE_IN);

        if (!opModeIsActive()){
            return;
        }

        // going forward 8 inches toward the crypto box
        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int FORWARD3_INCHES = (int)COUNTS_PER_INCH * 8;
        robot.leftback.setTargetPosition(-FORWARD3_INCHES);
        robot.rightback.setTargetPosition(FORWARD3_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.25);
        robot.rightback.setPower(0.25);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Driving Forward");
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        robot.rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Move Backwards Inches
        int BACKWARD3_INCHES = (int)COUNTS_PER_INCH * 5;
        robot.leftback.setTargetPosition(BACKWARD3_INCHES);
        robot.rightback.setTargetPosition(-BACKWARD3_INCHES);

        robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftback.setPower(0.5);
        robot.rightback.setPower(0.5);

        if (!opModeIsActive()){
            return;
        }

        while (opModeIsActive() &&
                (robot.leftback.isBusy() && robot.rightback.isBusy())) {
            // Display it for the driver.
            telemetry.addLine("Driving Backward");
            telemetry.update();
        }

        telemetry.addData("LP", robot.leftback.getCurrentPosition());
        telemetry.addData("RP", robot.rightback.getCurrentPosition());
        telemetry.update();

        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}
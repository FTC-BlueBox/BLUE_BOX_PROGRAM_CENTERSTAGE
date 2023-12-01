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

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode V1", group="Linear OpMode")
public class OpModeV1 extends LinearOpMode {


    private DcMotor MOTOR1;
    private DcMotor MOTOR2;
    private DcMotor MOTOR3;
    private DcMotor MOTOR4;
    private DcMotor MOTOR_LINEAR_RACK;
    private DcMotor MOTOR_INTAKE;
    private Servo PLANE_SERVO;

    double  reduceSpeedFactor = 0.6;
    double intakeMotorPower = 0;
    double linearRackMotorPower = 0;

    @Override
    public void runOpMode() {
        double MotorPower = 0; //motor power

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        MOTOR1  = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3  = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        MOTOR_LINEAR_RACK  = hardwareMap.get(DcMotor.class, "LINEARRACK");
        MOTOR_INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        PLANE_SERVO = hardwareMap.get(Servo.class, "PLANE");

        MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //old comments
        boolean CurrentGamepad1_X = false;          // Gamepad1 Button X is not pressed (Toggle Slow Speed
        boolean CurrentGamepad1_Y = false;          // Gamepad1 Button Y is not pressed
        boolean CurrentGamepad1_B = false;          // Gamepad1 Button B is not pressed
        boolean CurrentGamepad1_A = false;          // Gamepad1 Button A is not pressed

        String ALLIANCE_COLOR = "";

        // Wait for the game to start (driver presses PLAY)
        while(!opModeIsActive())
        {
            // Alliance Color (X=Blue, B=Red)
            if      (gamepad1.x || gamepad2.x)  ALLIANCE_COLOR = "BLUE";
            else if (gamepad1.b || gamepad2.b)  ALLIANCE_COLOR = "RED";

            telemetry.addData("Alliance", ALLIANCE_COLOR);
            telemetry.update();

            waitForStart();                           // Wait for the Play Button press
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //old comments
            CurrentGamepad1_X = gamepad1.x;             //
            CurrentGamepad1_Y = gamepad1.y;             //
            CurrentGamepad1_B = gamepad1.b;             //
            CurrentGamepad1_A = gamepad1.a;             //

            //gampad 1
            if (gamepad1.right_stick_y != 0)        // Robot Movement: Forward and Backward
            {
                MotorPower = gamepad1.right_stick_y * reduceSpeedFactor;

                //check motor configs
                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            }
            else if (gamepad1.right_stick_x != 0)        // Robot Movement: Forward and Backward
            {
                MotorPower =  gamepad1.right_stick_x * reduceSpeedFactor;

                //check motor configs
                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);
            }
            else if (gamepad1.left_stick_x != 0)        // Robot Movement: strafing
            {
                MotorPower =  gamepad1.left_stick_x * reduceSpeedFactor;

                //check motor configs
                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(MotorPower);
            } else{
                MotorPower =  0;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            }

            if (gamepad1.left_bumper) {                 //use linear rack
                if (linearRackMotorPower == 0){
                    linearRackMotorPower = 0.5;         //adjust
                }
                else{
                    linearRackMotorPower = 0;
                }
                MOTOR_LINEAR_RACK.setPower(linearRackMotorPower);
            }
            if (gamepad1.right_bumper) {                //use intake
                if (intakeMotorPower == 0){
                    intakeMotorPower = 0.4;
                }
                else {
                    intakeMotorPower = 0;
                }
                MOTOR_INTAKE.setPower(intakeMotorPower);
            }
            if(gamepad1.b){
                PLANE_SERVO.setPosition(0.4); //untested
            }
            //gamepad 2


            telemetry.update();
        }
    }
}

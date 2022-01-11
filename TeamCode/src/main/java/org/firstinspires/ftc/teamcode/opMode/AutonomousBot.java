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

package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

import static java.lang.Math.abs;


@Autonomous(name="AutonomousBot", group="Autonomous")
//@Disabled
public abstract class AutonomousBot extends LinearOpMode {
    protected CompetitionBot robot;

    @Override
    public void runOpMode() {
        waitForStart();
        forward(.5, 100, telemetry);
    }

    public void forward(double speed, double revCount, Telemetry telemetry) {
        //int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int stepCount = (int) revCount;
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int referencePos = avgPos;
        int targetPos = avgPos + stepCount;

        double initTime = System.currentTimeMillis();
        double initCollisionTime = 0;
        double currentTime;
        double currentVel = 0;

        boolean collisionDetected = false;

        double rampedSpeed;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) avgMotorPos();

            currentTime = System.currentTimeMillis();
            telemetry.addData("time: ", currentTime - initTime);

            // Calculating encoder velocity and checking for collision
            if (currentTime - initTime > 100) {
                currentVel = abs(avgPos - referencePos) / .1;
                initTime = currentTime;
                referencePos = avgPos;
            }
            telemetry.addData("currentVel: ", currentVel);
            if (!collisionDetected && currentVel < 50) {
                initCollisionTime = currentTime;
                collisionDetected = true;
            }
            if (collisionDetected && currentVel > 100) {
                collisionDetected = false;
            }
            if (collisionDetected && currentTime - initCollisionTime > 1000) {
                break;
            }


            rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .05, true);

            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            robot.setMotors(clamp(rampedSpeed), clamp(rampedSpeed));
        }
        robot.setMotors(0,0);
    }

    public void turnBy(double maxSpeed, double deltaAngle, Telemetry telemetry) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double speed = maxSpeed;
        double dir = 1;
        double rampStart = .75;


        while (Math.abs(diff) > .75) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);

            if (diff < 0) dir = -1;
            else dir = 1;

            if (Math.abs(diff) < rampStart * deltaAngle) {
                if (Math.abs(diff) < 2) speed = .125;
                else speed = Math.min(0.5 * maxSpeed * (diff / (0.5*deltaAngle)),.125);
            }

            speed *= dir;

            robot.LFmotor.setPower(speed);
            robot.LBmotor.setPower(speed);
            robot.RFmotor.setPower(-speed);
            robot.RBmotor.setPower(-speed);

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("speed: ", speed);
            telemetry.update();

        }
        stopMotors(250);
    }

    private double rampSpeed(double currentVal, double initVal, double targetVal, double speed, double minSpeed, boolean linearRamp) {
        double range = abs(targetVal - initVal);
        double currentDist = abs(currentVal - initVal);
        double remainingDist = abs(targetVal - currentVal);
        double rampRange = .25*range;

        telemetry.addData("dist: ", Math.round(currentDist / range * 100.0) / 100.0);

        if (currentDist <= range) {
            if (remainingDist <= rampRange) {
                return (remainingDist / rampRange) * (speed - minSpeed) * (linearRamp ? 1 : (speed - minSpeed)) + minSpeed;
            }
        } else {
            stopMotors(250);
            return 0;
        }
        return speed;
    }

    private void stopMotors(int durationTime) {
        robot.LFmotor.setTargetPosition(robot.LFmotor.getCurrentPosition());
        robot.LBmotor.setTargetPosition(robot.LBmotor.getCurrentPosition());
        robot.RFmotor.setTargetPosition(robot.RFmotor.getCurrentPosition());
        robot.RBmotor.setTargetPosition(robot.RBmotor.getCurrentPosition());

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotors(.75, .75);

        sleep(durationTime);

        robot.setMotors(0, 0);

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double avgMotorPos() { return ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0); }

    private double angleDiff(double angle1, double angle2) {
        double d1 = angle2 - angle1;
        if (d1 > 180) {
            return d1 - 360;
        } else if (d1 < -180) {
            return d1 + 360;
        } else {
            return d1;
        }
    }

    private double clamp(double power) {
        // ensures power does not exceed abs(1)
        if (power > 1) {
            return 1;
        }
        if (power < -1) {
            return -1;
        }
        return power;
    }
}

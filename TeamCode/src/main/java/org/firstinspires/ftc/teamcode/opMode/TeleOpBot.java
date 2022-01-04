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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.GamepadButton;

@TeleOp(name="TeleOpBot", group="TeleOp")
//@Disabled
public class TeleOpBot extends LinearOpMode {

    @Override
    public void runOpMode() {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        GamepadButton intakeButton = new GamepadButton(2);
        GamepadButton depositButton = new GamepadButton(5);
        GamepadButton duckButton = new GamepadButton(2);

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad 1: Driving
            double motion = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            // Gamepad 2: Utility
            double slide = gamepad2.left_stick_y;
//            boolean slideUpBool = gamepad2.dpad_up;
//            boolean slideDownBool = gamepad2.dpad_down;

            // Button Updates
            intakeButton.update(gamepad2.a);
            depositButton.update(gamepad2.b);
            duckButton.update(gamepad2.y);

            // Motion
            robot.tankMove(motion, rotation, telemetry);

            // Intake
            if (intakeButton.toggle == 1) robot.Intake.setPower(CompetitionBot.INTAKE_POWER);
            else robot.Intake.setPower(0);

            // Deposit
            if (depositButton.toggle == 0) robot.BoxServo.setPosition(CompetitionBot.BOX_VERT);
            else if (depositButton.toggle == 1) robot.BoxServo.setPosition(CompetitionBot.BOX_SLANT);
            else if (depositButton.toggle == 2) robot.BoxServo.setPosition(CompetitionBot.BOX_HORIZ);
            else if (depositButton.toggle == 3) robot.BoxServo.setPosition(CompetitionBot.BOX_OUT);
            else if (depositButton.toggle == 4) robot.BoxServo.setPosition(CompetitionBot.BOX_OUT2);

            // Ducks
            if (duckButton.toggle == 1) robot.DuckWheel.setPower(CompetitionBot.DUCK_POWER);
            else robot.DuckWheel.setPower(0);

            // Linear Slide
            if (slide>.1 || slide <.1) robot.LinearSlide.setPower(slide);
            else robot.LinearSlide.setPower(0);
        }
    }
}

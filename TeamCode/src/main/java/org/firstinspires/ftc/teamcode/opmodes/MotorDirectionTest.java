package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Run this FIRST on a new drivetrain, with the robot on blocks.
 * Hold a button to spin one wheel at low power. Every wheel should roll FORWARD
 * and its encoder should count UP. If not, fix the direction in MecanumDrive
 * (or swap the encoder/motor cable if only the encoder is backwards).
 *
 * Directions below must match MecanumDrive's constructor.
 *   X = frontLeft   Y = frontRight   A = backLeft   B = backRight
 */
@TeleOp(name = "Test: Motor Directions", group = "Test")
public class MotorDirectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor fr = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor bl = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor br = hardwareMap.get(DcMotor.class, "backRight");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Robot on blocks! X=FL  Y=FR  A=BL  B=BR");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            fl.setPower(gamepad1.x ? 0.3 : 0);
            fr.setPower(gamepad1.y ? 0.3 : 0);
            bl.setPower(gamepad1.a ? 0.3 : 0);
            br.setPower(gamepad1.b ? 0.3 : 0);

            telemetry.addLine("Each wheel should roll FORWARD and count UP");
            telemetry.addData("X frontLeft",  fl.getCurrentPosition());
            telemetry.addData("Y frontRight", fr.getCurrentPosition());
            telemetry.addData("A backLeft",   bl.getCurrentPosition());
            telemetry.addData("B backRight",  br.getCurrentPosition());
            telemetry.update();
        }
    }
}

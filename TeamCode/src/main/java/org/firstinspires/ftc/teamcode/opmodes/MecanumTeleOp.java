package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Driver-controlled mecanum TeleOp.
 *
 * gamepad1:
 *   Left stick      - drive (forward/back) and strafe (left/right)
 *   Right stick X   - turn
 *   Right bumper    - hold for slow/precision mode
 *   A               - toggle field-centric / robot-centric
 *   Back / Share    - reset heading (point robot away from driver first)
 */
@TeleOp(name = "Mecanum TeleOp", group = "Competition")
public class MecanumTeleOp extends LinearOpMode {

    private static final double NORMAL_POWER = 1.0;
    private static final double SLOW_POWER   = 0.35;
    private static final double DEADBAND     = 0.05;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        boolean fieldCentric = true;
        boolean lastA = false;

        telemetry.addLine("Ready. Point robot away from the driver, then press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Sticks: pushing up gives negative Y, so invert it
            double forward = deadband(-gamepad1.left_stick_y);
            double strafe  = deadband(gamepad1.left_stick_x);
            double turn    = deadband(gamepad1.right_stick_x);

            // Square inputs (keeping sign) for finer control at low speed
            forward = Math.copySign(forward * forward, forward);
            strafe  = Math.copySign(strafe * strafe, strafe);
            turn    = Math.copySign(turn * turn, turn);

            // Toggle field-centric on the A button's rising edge
            if (gamepad1.a && !lastA) fieldCentric = !fieldCentric;
            lastA = gamepad1.a;

            if (gamepad1.back) drive.resetHeading();

            drive.setMaxPower(gamepad1.right_bumper ? SLOW_POWER : NORMAL_POWER);

            if (fieldCentric) {
                drive.driveFieldCentric(forward, strafe, turn);
            } else {
                drive.drive(forward, strafe, turn);
            }

            telemetry.addData("Mode", fieldCentric ? "FIELD-centric (A to switch)" : "ROBOT-centric (A to switch)");
            telemetry.addData("Speed", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("Encoders", drive.wheelPositions());
            telemetry.update();
        }
        drive.stop();
    }

    private static double deadband(double v) {
        return Math.abs(v) < DEADBAND ? 0.0 : v;
    }
}

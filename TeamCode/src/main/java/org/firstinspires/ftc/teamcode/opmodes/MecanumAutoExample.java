package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Example encoder + IMU autonomous. Drives a small square-ish path to prove the
 * drivetrain constants are right. Replace the steps with your real routine.
 *
 * Tip: run it on blocks first and measure. If 24 in comes out as 22 in, adjust
 * WHEEL_DIAMETER_IN or TICKS_PER_REV in MecanumDrive; if strafes are short, raise
 * STRAFE_MULTIPLIER.
 */
@Autonomous(name = "Mecanum Auto Example", group = "Competition", preselectTeleOp = "Mecanum TeleOp")
public class MecanumAutoExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        telemetry.addLine("Initialized. Heading resets at START.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        drive.resetHeading();

        drive.driveDistance(this, 24, 0, 0.5, 4);    // forward 24 in
        drive.driveDistance(this, 0, 12, 0.5, 4);    // strafe right 12 in
        drive.turnToHeading(this, 90, 0.5, 3);       // face left (CCW 90)
        drive.turnToHeading(this, 0, 0.5, 3);        // face original direction
        drive.driveDistance(this, -24, -12, 0.5, 5); // diagonal back to start

        telemetry.addLine("Done");
        telemetry.update();
    }
}

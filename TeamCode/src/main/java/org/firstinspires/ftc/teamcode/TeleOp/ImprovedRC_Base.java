package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Controls.IntakeControl;
import org.firstinspires.ftc.teamcode.Controls.OuttakeControl;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@TeleOp
@Config
public class ImprovedRC_Base extends LinearOpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IntakeControl intakeControl = new IntakeControl(robot.IntakeSlides, robot.IntakeDiffy, robot.IntakeGripper);
        OuttakeControl outtakeControl = new OuttakeControl(robot.wrist, robot.OuttakeGripper, robot.pivotSlides,robot.hang);
        Subsystem[] subsystems = new Subsystem[]{robot};
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();
        robot.wrist.setWristPos(robot.wrist.getWristPos());
        robot.OuttakeGripper.setPosition(.55);
        robot.pivotSlides.togglePID(false);
        waitForStart();

        while(opModeIsActive()) {
            for(Subsystem system : subsystems) system.update();
            intakeControl.update(gamepad1, gamepad2);
            outtakeControl.update(gamepad1, gamepad2);
            if(gamepad1.left_bumper) {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*.35, -gamepad1.left_stick_x*.35, -gamepad1.right_stick_x*.35);
            }
            else{
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*.5, -gamepad1.left_stick_x*.5, -gamepad1.right_stick_x*.5);
            }
            follower.update();
            telemetry.addData("slidePos",robot.pivotSlides.getSlidePos());
            telemetry.addData("wristPos",robot.wrist.getWristPos());
            telemetry.addData("hangPos",robot.hang.getArmPos());
            telemetry.addData("hangTarget",robot.hang.getArmTarget());
            telemetry.update();



        }
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing.localization;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@Autonomous (name = "runPedroLeftAuto", group = "Autonomous Pathing")
public class runPedroLeftAuto extends OpMode
{
    private Servo claw;
    private DcMotor wrist;
    private AnalogInput pot;
    private DcMotor arm;
    private CRServo intake;
    
    private Follower follower;
    private Pose startPose = new Pose(9, 84, 0);
    private GeneratedPathPedroLeftAuto gppla;
    private int pathNumber = -1;
    private int actionNumber = -1;
    private boolean waitingForActions = true;
    private String[] telemetryMessages;

    private double targetPot = 0;
    private double prevTargetPot = 0;
    private double prevPot = 0;
    private double currentPot = 0;
    private int prevEncoderPos = 0;
    private int currentEncoderPos = 0;
    private int targetEncoderPos = 0;
    private int targetEncoderOffset = 0;
    final double ratio = 1100;

    private int timer;


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init()
    {
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        pot = hardwareMap.get(AnalogInput.class, "pot");
        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.8);
        follower.setStartingPose(startPose);
        gppla = new GeneratedPathPedroLeftAuto();
        pathNumber = 0;
        actionNumber = 0;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        targetEncoderPos = (int) (targetPot * ratio);

        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setTargetPosition(0);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(0.4); // likely needs tweaking

        telemetryMessages = new String[] {
                "specimen placement",
                "cycle 1 grab",
                "cycle 1 place",
                "cycle 2 grab",
                "cycle 2 place",
                "cycle 3 grab",
                "cycle 3 place",
                "park"
        };

    }

    /* In this code, we have a target encoder position (which is proportional to the target pot)
    and another to account for slack. Whenever we notice that the pot value isn't changing, we add on the 
    change to the encoder position to the offset value. This makes it so that if the pot value isn't changing,
    the "goal" is moving "farther away" to account for the fact that we aren't actually moving the arm */
    public void armMovement() {
        // Put loop blocks here.
        prevPot = currentPot;
        currentPot = pot.getVoltage();
        prevEncoderPos = currentEncoderPos;
        currentEncoderPos = arm.getCurrentPosition();
        if (prevPot < 0.05) {
            targetEncoderOffset += currentEncoderPos - prevEncoderPos;
        }
        arm.setTargetPosition(targetEncoderPos + targetEncoderOffset);
    }

    /* This simply updates the base target encoder position. It is CRUCIAL that we do NOT change the encoder
    offset. */
    private void updateTargetEncoderPosition(double newPot) {
        targetEncoderPos = (int) (newPot * ratio);
    }

    /* This is the main loop. It is important to note that this will be looping through many times a second,
    and sometimes code will start running before other lines of code even if one is supposed to run after
    the others. This is why we add timers to have a global way to keep track of which loop we are currently
    on. This also serves as a clock we can use to add pauses to our code, since of course loop() isn't going
    to be running infinitely fast. Values for the timer may have to be adjusted depending on how quickly
    the robot is executing the loop, but in general it should be roughly the same between battery levels.
    It's still good to add a bit more to the timer value just in case the loop runs a bit faster than intended. */
    @Override
    public void loop() {
        armMovement();

        if (pathNumber < 8) {
            telemetry.addLine(telemetryMessages[pathNumber]);
        }
        telemetry.addData("raw pot value", currentPot);
        telemetry.addData("offset pot value", currentPot);
        telemetry.addData("arm position", currentEncoderPos);
        telemetry.addData("targetPos", targetEncoderPos);
        telemetry.update();

        follower.update();
        // if the follower has finished moving and there aren't any more actions in the current path
        if (!follower.isBusy() && !waitingForActions) {
            pathNumber++;
            actionNumber = 0;
            waitingForActions = true;
        }

        switch (pathNumber) {
            case 0: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.648);
                    timer = 500;
                } else if (actionNumber == 1) {
                    follower.followPath(gppla.specimenPlacement);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 1: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.5);
                    timer = 500;
                } else if (actionNumber == 1) {
                    claw.setPosition(0);
                    timer = 250;
                } else if (actionNumber == 2) {
                    updateTargetEncoderPosition(0.648);
                    timer = 500;
                } else if (actionNumber == 3) {
                    follower.followPath(gppla.firstSampleGrab);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 2: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.277);
                    wrist.setTargetPosition(396);
                    timer = 500;
                } else if (actionNumber == 1) {
                    intake.setPower(1); // whichever way makes it suck in
                    timer = 100;
                } else if (actionNumber == 2) {
                    intake.setPower(0);
                } else if (ationNumber == 3) {
                    updateTargetEncoderPosition(0.4); // slightly above ground
                    wrist.setTargetPosition(0);
                    timer = 500;
                } else if (actionNumber == 4) {
                    follower.followPath(gppla.firstSampleRelease);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 3: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.277);
                    wrist.setTargetPosition(396);
                    timer = 500;
                } else if (actionNumber == 1) {
                    intake.setPower(1); // whichever way makes it spit out
                    timer = 100;
                } else if (actionNumber == 2) {
                    intake.setPower(0);
                } else if (ationNumber == 3) {
                    updateTargetEncoderPosition(0.4); // slightly above ground
                    wrist.setTargetPosition(0);
                    timer = 500;
                } else if (actionNumber == 4) {
                    follower.followPath(gppla.secondSampleGrab);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 4: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.277);
                    wrist.setTargetPosition(396);
                    timer = 500;
                } else if (actionNumber == 1) {
                    intake.setPower(1); // whichever way makes it suck in
                    timer = 100;
                } else if (actionNumber == 2) {
                    intake.setPower(0);
                } else if (ationNumber == 3) {
                    updateTargetEncoderPosition(0.4); // slightly above ground
                    wrist.setTargetPosition(0);
                    timer = 500;
                } else if (actionNumber == 4) {
                    follower.followPath(gppla.secondSampleRelease);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 5: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.277);
                    wrist.setTargetPosition(396);
                    timer = 500;
                } else if (actionNumber == 1) {
                    intake.setPower(1); // whichever way makes it spit out
                    timer = 100;
                } else if (actionNumber == 2) {
                    intake.setPower(0);
                } else if (ationNumber == 3) {
                    updateTargetEncoderPosition(0.4); // slightly above ground
                    wrist.setTargetPosition(0);
                    timer = 500;
                } else if (actionNumber == 4) {
                    follower.followPath(gppla.thirdSampleGrab);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 6: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.277);
                    wrist.setTargetPosition(396);
                    timer = 500;
                } else if (actionNumber == 1) {
                    intake.setPower(1); // whichever way makes it suck in
                    timer = 100;
                } else if (actionNumber == 2) {
                    intake.setPower(0);
                } else if (ationNumber == 3) {
                    updateTargetEncoderPosition(0.4); // slightly above ground
                    wrist.setTargetPosition(0);
                    timer = 500;
                } else if (actionNumber == 4) {
                    follower.followPath(gppla.thirdSampleRelease);
                } else {
                    waitingForActions = false;
                }
                break;
            }
            case 7: {
                if (actionNumber == 0) {
                    follower.followPath(gppla.park);
                    timer = 1000;
                } else {
                    waitingForActions = false;
                }
                break;
            }

            default: {
                if (actionNumber == 0) {
                    wrist.setPosition(0);
                    updateTargetEncoderPosition(0.1);
                    timer = 200;
                } else {
                    requestOpModeStop();
                }
                break;
            }
        }

        /* if the code wants it to pause, it will continue to "waste" computation
        cycles until the timer reaches zero, and then the actionNumber will go up
        previously this actionNumber++; was in every single switch case, but I
        brought it out of that here since every single one did it anyway. */

        if (timer != 0) {
            timer -= 1;
        } else {
            /* java doesn't always execute things in order (sometimes putting them aside to execute later)
            so we want to make sure that it gets a few cycles to start executing instructions since we may
            accidentally skip some instructions. I arbitrarily set the timer to 5 here, but maybe more is
            necessary. */
            timer = 5; 
            actionNumber++;
        }
    }
}
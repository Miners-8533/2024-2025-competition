package org.firstinspires.ftc.teamcodetestbot.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public class Trajectories {
    Pose2d initialPose = new Pose2d(-24,-63, Math.toRadians(90));

    Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

    Pose2d scoreRedBasketLeft = new Pose2d(-41,-60, Math.toRadians(180));

    Pose2d firstSpikeMark = new Pose2d(-36,-22.25, Math.toRadians(180));

    Action sumbersibleToBasketLeft(TrajectoryActionBuilder tab) {
        return tab.splineToLinearHeading(scoreChamber, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(scoreRedBasketLeft, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(90))
                .waitSeconds(2)
                .strafeTo(scoreRedBasketLeft.position)
                .build();
    }
}

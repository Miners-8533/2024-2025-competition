package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepValidateDocumentation {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        DriveShim drive = myBot.getDrive();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(0.0, 48.0, 0.0))
                .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2)
                .splineTo(new Vector2d(0.0, -48.0), -Math.PI)
                .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
                .splineTo(new Vector2d(0.0, 48.0), 0.0);

        myBot.runAction(new SequentialAction(
                tab1.build()
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
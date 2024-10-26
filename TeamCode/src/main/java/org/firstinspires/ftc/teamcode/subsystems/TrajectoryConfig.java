package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.FieldStartPosition;

public class TrajectoryConfig {
        private MecanumDrive rrdrive;
        PoseConfig poseConfig = new PoseConfig(rrdrive);
        public TrajectoryConfig(MecanumDrive autonDrive){
            this.rrdrive = autonDrive;
        }

        static Pose2d mirrorPose(double x, double y, double heading) {
            // Flip the X and Y coordinates
            double mirroredX = -x;
            double mirroredY = -y;

            // Adjust the heading to be mirrored
            double mirroredHeading = heading + Math.PI;

            // Normalize the heading to be within [-PI, PI]
            mirroredHeading = (mirroredHeading + Math.PI) % (2 * Math.PI) - Math.PI;

            return new Pose2d(mirroredX, mirroredY, mirroredHeading);
        }

        static double mirrorAxis(double component){
            return -component;
        }

        static Pose2d[] DetermineStartAndEndPose(Pose2d startPose, Pose2d endPose, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
            Pose2d[] returnArray = new Pose2d[2];

            if (fieldStartPosition == FieldStartPosition.LEFT){
                startPose = new Pose2d(mirrorAxis(startPose.position.x), startPose.position.y, startPose.heading.toDouble());
                endPose = new Pose2d(mirrorAxis(endPose.position.x), endPose.position.y, endPose.heading.toDouble());
            }

            if (allianceColor == AllianceColor.BLUE){
                startPose = mirrorPose(startPose.position.x, startPose.position.y, startPose.heading.toDouble());
                endPose = mirrorPose(endPose.position.x, endPose.position.y, endPose.heading.toDouble());
            }

            returnArray[0] = startPose;
            returnArray[1] = endPose;

            return returnArray;
        }

        static double DetermineEndTangent(double endTangent, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){

        }

        public Action ScoreChamberTrajectory(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
            // assume start position is always lower right quadrant, aka red observation zone
            Pose2d[] startAndEndPose = DetermineStartAndEndPose(poseConfig.getInitialPose(), poseConfig.getChamberPose(),allianceColor, fieldStartPosition);
            double endTangent = 0;
            Pose2d initialPose = startAndEndPose[0];
            Pose2d chamberPose = startAndEndPose[1];

            if (fieldStartPosition == FieldStartPosition.LEFT){
                initialPose = new Pose2d(mirrorAxis(initialPose.position.x), initialPose.position.y, initialPose.heading.toDouble());
                chamberPose = new Pose2d(mirrorAxis(chamberPose.position.x), chamberPose.position.y, chamberPose.heading.toDouble());
            }

            if (allianceColor == AllianceColor.BLUE){
                endTangent = 180;
                initialPose = mirrorPose(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
                chamberPose = mirrorPose(chamberPose.position.x, chamberPose.position.y, chamberPose.heading.toDouble());
            }

            TrajectoryActionBuilder tab1 = rrdrive.actionBuilder(initialPose)
                    .splineToLinearHeading(chamberPose, Math.toRadians(90 + endTangent));

            return tab1.build();
        }
    public Action FirstSpikeMarkTrajectory(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        // assume start position is always lower right quadrant, aka red observation zone
        double endTangent = 0;
        Pose2d initialPose = poseConfig.getInitialPose();
        Pose2d chamberPose = poseConfig.getChamberPose();

        if (fieldStartPosition == FieldStartPosition.LEFT){
            initialPose = new Pose2d(mirrorAxis(initialPose.position.x), initialPose.position.y, initialPose.heading.toDouble());
            chamberPose = new Pose2d(mirrorAxis(chamberPose.position.x), chamberPose.position.y, chamberPose.heading.toDouble());
        }

        if (allianceColor == AllianceColor.BLUE){
            endTangent = 180;
            initialPose = mirrorPose(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
            chamberPose = mirrorPose(chamberPose.position.x, chamberPose.position.y, chamberPose.heading.toDouble());
        }

        TrajectoryActionBuilder tab1 = rrdrive.actionBuilder(initialPose)
                .splineToLinearHeading(chamberPose, Math.toRadians(90 + endTangent));

        return tab1.build();
    }
}

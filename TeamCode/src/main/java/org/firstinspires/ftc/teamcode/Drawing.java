package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
    public static void drawCamera(Canvas c, Vector2d t) {
        final double CAMERA_RADIUS = 3;

        c.setStrokeWidth(1);
        c.strokeCircle(t.x, t.y, CAMERA_RADIUS);

    }
    public static void drawGripper(Canvas c, Vector2d t) {
        final double GRIPPER_RADIUS = 3;

        c.setStrokeWidth(1);
        c.strokeCircle(t.x, t.y, GRIPPER_RADIUS);

    }
    public static void drawSample(Canvas c, Vector2d t) {
        final double SAMPLE_RADIUS = 2;

        c.setStrokeWidth(1);
        c.strokeCircle(t.x, t.y, SAMPLE_RADIUS);
    }
}

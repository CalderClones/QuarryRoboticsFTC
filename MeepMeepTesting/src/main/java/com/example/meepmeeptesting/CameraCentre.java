/*
 * Make sure to use MeepMeep 1.0.1
 *
 * This is a very rough POC and not polished at all. Velocity doesn't decelerate, no clear units, etc.
 * Also the shot in the marker should be moved to a reusable function
 */

/**** RingEntity.java *****/
package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.core.util.FieldUtil;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

public class CameraCentre implements Entity {
    private final Vector2d robotToCamera =  new Vector2d(-9.37 / 25.4, 488.84 / 25.4);
    private final String tag = "CAMERA_CENTRE";

    private int zIndex = 10;

    private final MeepMeep meepMeep;

    private double canvasWidth = FieldUtil.getCANVAS_WIDTH();
    private double canvasHeight = FieldUtil.getCANVAS_WIDTH();

    private final double radius = 1;
    private final double thickness = 1;

    private Vector2d position;
    private final Vector2d velocity;

    private final RoadRunnerBotEntity bot;

    public CameraCentre(MeepMeep meepMeep, RoadRunnerBotEntity bot, Vector2d intialPosition, Vector2d initialVelocity) {
        this.meepMeep = meepMeep;
        this.bot = bot;
        this.position = intialPosition;
        this.velocity = initialVelocity;
    }

    @Override
    public void update(long deltaTime) {
        Vector2d robotVel =
        position = bot.getPose().position.plus(Rotation2d.exp(bot.getPose().heading.toDouble() - toRadians(90)).times(robotToCamera));
        /*if (position.x > FieldUtil.getFIELD_WIDTH() / 2.0 || position.x < -FieldUtil.getFIELD_WIDTH() / 2.0 || position.y > FieldUtil.getFIELD_HEIGHT() / 2.0 || position.y < -FieldUtil.getFIELD_HEIGHT() / 2.0) {
            meepMeep.requestToRemoveEntity(this);
        }*/
    }

    @Override
    public void render(Graphics2D gfx, int i, int i1) {
        Vector2d screenCoords = FieldUtil.fieldCoordsToScreenCoords(position);
        double radPixels = FieldUtil.scaleInchesToPixel(radius, canvasWidth, canvasHeight);

        gfx.setStroke(new BasicStroke((int) FieldUtil.scaleInchesToPixel(thickness, canvasWidth, canvasHeight)));
        gfx.setColor(new Color(255, 255, 0));
        gfx.drawOval(
                (int) (screenCoords.x - radPixels),
                (int) (screenCoords.y - radPixels),
                (int) (radPixels * 2),
                (int) (radPixels * 2)
        );
    }

    @NotNull
    @Override
    public MeepMeep getMeepMeep() {
        return null;
    }

    @NotNull
    @Override
    public String getTag() {
        return tag;
    }

    @Override
    public int getZIndex() {
        return this.zIndex;
    }

    @Override
    public void setZIndex(int i) {
        this.zIndex = i;
    }

    @Override
    public void setCanvasDimensions(double width, double height) {
        this.canvasWidth = width;
        this.canvasHeight = height;
    }
}

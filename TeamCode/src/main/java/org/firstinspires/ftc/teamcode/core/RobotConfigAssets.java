package org.firstinspires.ftc.teamcode.core;



import android.content.res.AssetManager;



import java.io.ByteArrayOutputStream;

import java.io.IOException;

import java.io.InputStream;

import java.nio.charset.StandardCharsets;



public final class RobotConfigAssets {

    public static final String ROBOT_CONFIG_PATH = "robot-config.json";

    public static final String FIELD_BLUE_PATH = "field-blue.json";



    private RobotConfigAssets() {}



    public static LiftingSequenceConfig load(AssetManager assets) throws IOException {

        return load(assets, Alliance.BLUE);

    }



    public static LiftingSequenceConfig load(AssetManager assets, Alliance alliance) throws IOException {

        if (assets == null) throw new IllegalArgumentException("missing assets");

        LiftingSequenceConfig robot = LiftingSequenceConfigLoader.load(readAsset(assets, ROBOT_CONFIG_PATH));

        FieldBlueConfig field = FieldBlueConfigLoader.load(readAsset(assets, FIELD_BLUE_PATH));

        return robot.withField(field, alliance);

    }



    public static String readAsset(AssetManager assets, String path) throws IOException {

        try (InputStream input = assets.open(path)) {

            ByteArrayOutputStream bytes = new ByteArrayOutputStream();

            byte[] buffer = new byte[1024];

            int count;

            while ((count = input.read(buffer)) != -1) {

                bytes.write(buffer, 0, count);

            }

            return new String(bytes.toByteArray(), StandardCharsets.UTF_8);

        }

    }

}


package org.firstinspires.ftc.teamcode.core;

import android.content.res.AssetManager;

import org.firstinspires.ftc.teamcode.core.pen.DrawPathConfig;
import org.firstinspires.ftc.teamcode.core.pen.DrawPathLoader;
import org.firstinspires.ftc.teamcode.core.pen.RobotConfig;
import org.firstinspires.ftc.teamcode.core.pen.RobotConfigLoader;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

public final class RobotConfigAssets {

    public static final String ROBOT_CONFIG_PATH = "robot-config.json";
    public static final String DRAW_PATH_PATH = "draw-path.json";

    private RobotConfigAssets() {}

    public static RobotConfig loadRobotConfig(AssetManager assets) throws IOException {
        if (assets == null) throw new IllegalArgumentException("missing assets");
        return RobotConfigLoader.load(readAsset(assets, ROBOT_CONFIG_PATH));
    }

    public static DrawPathConfig loadDrawPath(AssetManager assets) throws IOException {
        if (assets == null) throw new IllegalArgumentException("missing assets");
        return DrawPathLoader.load(readAsset(assets, DRAW_PATH_PATH));
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

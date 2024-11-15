package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.Arrays;

public final class DashboardConfig {
    public static final String GROUP = "tuning";
    public static final boolean TESTING_ENABLED = true;

    private DashboardConfig() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!TESTING_ENABLED) return;

        // Register tuning/test OpModes
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(StraightTest.class), StraightTest.class);

        // Register with Dashboard using ReflectionConfig
        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    SplineTest.class,
                    StraightTest.class
            )) {
                configRoot.putVariable(c.getSimpleName(),
                        ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
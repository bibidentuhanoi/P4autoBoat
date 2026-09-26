#include "runtime_schedule.h"

#include <stddef.h>

static const runtime_task_spec_t s_schedule[RUNTIME_TASK_COUNT] = {
    [RUNTIME_TASK_CONTROL] = {"Control", 4096, 10, 0, 10000, 10000, true},
    [RUNTIME_TASK_ARM_SEQUENCE] = {"ArmSeq", 4096, 9, 0, 0, 0, true},
    [RUNTIME_TASK_SENSOR_BUS] = {"SensorBus", 8192, 8, 0, 20000, 20000, true},
    [RUNTIME_TASK_FUSION] = {"Fusion", 4096, 7, 0, 0, 40000, true},
    [RUNTIME_TASK_GPS] = {"GPS", 4096, 6, 0, 0, 0, false},
    /* Split off SensorBus 2026-08-11: a VL53L5CX read at the old 400kHz bus
     * setting (~33ms) exceeded
     * SensorBus's own 20ms IMU period, so interleaving it there guaranteed a
     * deadline miss on every cycle ToF was selected (hw-confirmed: sustained
     * ~38% SensorBus miss rate). Lower priority than SensorBus/Fusion so IMU
     * always wins any scheduling contention -- I2C bus access itself is still
     * serialized by ESP-IDF's own per-transaction bus lock, not an
     * application mutex around the whole read (that would just relocate the
     * same long stall onto whichever task loses the lock). period/deadline
     * here describe the poll tick (fine enough to hit each sensor's own
     * ~100ms due-time precisely), not the read itself -- ticks with an
     * actual read due may exceed the 25ms deadline under contention; the task
     * is optional and lower priority than IMU acquisition. */
    [RUNTIME_TASK_TOF_READ] = {"ToFRead", 4096, 5, 0, 20000, 25000, false},
    [RUNTIME_TASK_DETECT] = {"Detect", 32768, 7, 1, 0, 0, false},
    [RUNTIME_TASK_CAMERA_DRAIN] = {"CamDrain", 2048, 6, 1, 0, 0, false},
    [RUNTIME_TASK_SNAPSHOT] = {"Snapshot", 16384, 5, 1, 50000, 50000, false},
    [RUNTIME_TASK_TOF_PROCESS] = {"ToFProc", 6144, 4, 1, 200000, 0, false},
    [RUNTIME_TASK_WS_TX] = {"WS_TX", 8192, 3, 1, 0, 0, false},
    /* 6 KB: it writes the bench and mission records to SD -- printf of
     * doubles, FATFS and the SDMMC driver all on this stack.  Measured from
     * the linked firmware (2026-09-25): the worst path, a heap error logged
     * from inside that printf, needs 5,520 B (5,104 B on main c8a4c5e), over
     * the old 4 KB; the normal path ~3 KB. */
    [RUNTIME_TASK_DIAGNOSTICS] = {"Diagnostics", 6144, 2, 1, 1000000, 0, false},
    /* Stack in PSRAM (hw, 2026-09-26: in WiFi mode the internal RAM left after
     * boot is ~17 KB in pieces under 1.4 KB, too small for this 8 KB stack).
     * It captures a JPEG and writes it to the SD card, never flash: its NVS
     * session number is read in training_log_init(), on the boot task. */
    [RUNTIME_TASK_TRAINING_LOG] = {"TrainingLog", 8192, 2, 1, 0, 0, false, true},
    [RUNTIME_TASK_STATUS_LED] = {"StatusLED", 2048, 2, 1, 0, 0, false},
    /* Compass/IMU calibration at boot, then a 1 Hz health report. Low
     * priority: it only reads the shared sensor snapshot, never the bus.
     * 8 KB: it also writes the spin record to SD (FATFS runs on this stack). */
    [RUNTIME_TASK_COMPASS_CAL] = {"CompassCal", 8192, 2, 1, 0, 0, false},
    /* The out-and-back mission (2026-09-25): 20 Hz on core 1, above every
     * other core-1 task so the control task's setpoint never goes stale
     * (0.5 s ends the run).  Pure arithmetic: no bus, no SD (the record is
     * written by Diagnostics), no radio (MissionStatus too), no mutex -- only
     * spinlock copies (the drive snapshot included: never the ESC driver's
     * mutex, which is the control task's) and the console lock of its few
     * log lines.  Optional: if it cannot start the boat simply has no mission.
     * Stack in PSRAM: the internal RAM had no 4 KB piece left for it on the
     * first WiFi-mode boot (hw, 2026-09-26, "no missions this boot"), and it
     * never touches flash or NVS. */
    [RUNTIME_TASK_AUTONOMY] = {"Autonomy", 4096, 8, 1, 50000, 50000, false, true},
};

const runtime_task_spec_t *runtime_schedule_get(runtime_task_id_t id)
{
    if (id < 0 || id >= RUNTIME_TASK_COUNT) {
        return NULL;
    }
    return &s_schedule[id];
}

bool runtime_schedule_validate(void)
{
    for (runtime_task_id_t id = RUNTIME_TASK_CONTROL; id < RUNTIME_TASK_COUNT; ++id) {
        const runtime_task_spec_t *spec = &s_schedule[id];
        if (!spec->name || !spec->stack_size || !spec->priority ||
            (spec->core != 0 && spec->core != 1)) {
            return false;
        }
        if (spec->critical && spec->stack_in_psram) {
            return false;       /* critical tasks keep internal stacks: some save to NVS */
        }
    }

    return s_schedule[RUNTIME_TASK_CONTROL].core == 0 &&
           s_schedule[RUNTIME_TASK_CONTROL].critical &&
           s_schedule[RUNTIME_TASK_SENSOR_BUS].core == 0 &&
           s_schedule[RUNTIME_TASK_SENSOR_BUS].critical;
}

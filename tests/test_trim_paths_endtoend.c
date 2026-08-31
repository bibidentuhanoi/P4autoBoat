/* Three ways a throttle reaches the motors. All three must apply the SAME
 * trim, or a BASE run measures something the pilot never drives.
 *
 *   1. BASE bench run   bench_step() + esc_trim_apply_pair()
 *   2. linked slider    esc_trim_mix(throttle, rudder)
 *   3. unlinked slider  motor_command_handler() folds L/R back into
 *                       throttle+rudder, then the same esc_trim_mix
 *
 * Run with the real proportional table, over the whole 10-50% BASE sweep.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "esc_trim.h"
#include "bench_run.h"
#include "trim_learn.h"

#define C 0.20f
static int near(float a, float b) { return fabsf(a - b) < 1e-5f; }

/* motor_control.c:654 motor_command_handler -- per-motor commands are folded
 * into a common throttle plus a differential, exactly so they rejoin the one
 * drive path. Mirrored here to prove the unlinked slider is trimmed too. */
static void unlinked_slider(float l_cmd, float r_cmd, float *thr, float *rud)
{
    float m = fmaxf(fabsf(l_cmd), fabsf(r_cmd));
    if (m > 1.0f) { l_cmd /= m; r_cmd /= m; }
    *thr = 0.5f * (l_cmd + r_cmd);
    *rud = 0.5f * (l_cmd - r_cmd);
}

int main(void)
{
    EscTrimPoint pts[2];
    uint8_t n = esc_trim_build_proportional(C, pts);

    const bench_cfg_t cfg = { .baseline_us = 500000, .run_us = 3000000,
                              .coast_us = 1000000, .max_yaw_dps = 200.0f };

    for (int pct = 10; pct <= 50; pct += 5) {
        const float T = (float)pct / 100.0f;
        const float want_l = T * (1.0f - C), want_r = T * (1.0f + C);

        /* ---- 1. the BASE bench run, tick by tick at 100 Hz ---- */
        bench_t b;
        bench_init(&b);
        assert(bench_start(&b, BENCH_KIND_BASE, T, 0.04f, 0));
        int base_n = 0, run_n = 0, coast_n = 0;
        for (int64_t us = 0; us <= 4600000; us += 10000) {
            /* the trim the boat looks up, at the run's base throttle */
            float trim = esc_trim_lookup(pts, n, b.base);
            bench_out_t o = bench_step(&b, &cfg, us, 0.0f, true, trim, NULL);
            if (!o.active) continue;
            if (b.state == BENCH_BASELINE) {
                /* the gyro-drift window every other number is corrected
                 * against: one jet waking here silently ruins the run */
                assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);
                base_n++;
            } else if (b.state == BENCH_RUN) {
                assert(near(o.left_cmd, want_l) && near(o.right_cmd, want_r));
                run_n++;
            } else if (b.state == BENCH_COAST) {
                assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);
                coast_n++;
            }
        }
        assert(base_n == 50 && run_n == 300 && coast_n == 100);
        assert(b.count == 450 && !b.overflow);

        /* What the boat WROTE is what it SENT -- and the c the analyzer reads
         * back out of that file is the c that was flashed. */
        for (uint16_t i = 0; i < b.count; ++i) {
            const bench_sample_t *s = &b.samples[i];
            if (s->left + s->right <= 0.0f) continue;
            assert(near((s->right - s->left) / (s->right + s->left), C));
        }

        /* ---- 2. the linked slider ---- */
        float ml = 0.0f, mr = 0.0f;
        esc_trim_mix(T, 0.0f, pts, n, &ml, &mr);
        assert(near(ml, want_l) && near(mr, want_r));

        /* ---- 3. the unlinked slider, both motors set to the same value ---- */
        float thr, rud;
        unlinked_slider(T, T, &thr, &rud);
        assert(near(thr, T) && near(rud, 0.0f));
        float ul = 0.0f, ur = 0.0f;
        esc_trim_mix(thr, rud, pts, n, &ul, &ur);
        assert(near(ul, want_l) && near(ur, want_r));

        /* ...and unlinked with a real per-motor difference: the pilot's own
         * differential survives, and the trim still lands on the common. */
        unlinked_slider(T + 0.05f, T - 0.05f, &thr, &rud);
        assert(near(thr, T) && near(rud, 0.05f));
        float dl = 0.0f, dr = 0.0f;
        esc_trim_mix(thr, rud, pts, n, &dl, &dr);
        assert(near(0.5f * (dl + dr), T));            /* common kept */
        assert(near(dl - dr, 0.10f - 2.0f * C * T));  /* pilot diff + trim */
    }

    /* ---- the learner is frozen while the ESCs are not armed ----
     * motor_control's trim_learn_tick reports throttle 0 when the boat is not
     * ARMED. A steady yaw must then move nothing, however long it lasts. */
    trim_learn_cfg_t lc = { .c_init = C, .c_min = 0.10f, .c_max = 0.35f,
                            .deadband_dps = 0.5f, .step_per_s = 0.005f,
                            .yaw_tau_s = 2.0f, .min_throttle = 0.15f };
    trim_learn_t st;
    trim_learn_init(&st, &lc);
    for (uint32_t seq = 1; seq <= 500; ++seq) {       /* 10 s at 50 Hz */
        trim_learn_update(&st, &lc, seq, 0.02f, -6.0f, 0.0f, false, true);
    }
    assert(near(st.c, C));                            /* not one step */

    /* ...and with the same yaw while ARMED and driving, it does move, the
     * right way: a negative drift asks for MORE trim. */
    trim_learn_init(&st, &lc);
    for (uint32_t seq = 1; seq <= 500; ++seq) {
        trim_learn_update(&st, &lc, seq, 0.02f, -6.0f, 0.30f, false, true);
    }
    assert(st.c > C);
    assert(st.c <= C + 0.005f * 10.0f + 1e-5f);       /* rate-limited */

    printf("test_trim_paths_endtoend: OK\n");
    return 0;
}

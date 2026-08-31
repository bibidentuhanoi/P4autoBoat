/* Monte-Carlo replay of the trim learner against REAL recorded gyro noise.
 *
 * Answers: does the adaptive trim converge to the same place from a low and a
 * high start, does it stay there, and does it beat simply flashing a fixed c?
 *
 * Everything here is measured, not modelled:
 *   plant slope   +20.4 deg/s per unit c   (regression, autotrim2 T30, r=0.83)
 *   true c        drawn from 0.145..0.212  (12 sign-flip brackets across two
 *                                           sessions, three throttles)
 *   noise         bootstrap of 3 s windows from the residual of those same
 *                 runs -- real gyro, real hull, real pool, so the within-run
 *                 correlation and the wall bounces are preserved
 *
 * The truth is RE-DRAWN each trial because it genuinely moves between sessions
 * (the same c=0.190 read -0.26 one day and +0.18 the next). That is the whole
 * case for adapting rather than flashing a number, so a fair comparison has to
 * include it.
 *
 *   cc -std=c11 -I ../../main ../../main/trim_learn.c trim_replay.c -lm -o replay
 *   ./replay <residuals.txt> [trials]
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "trim_learn.h"

#define SLOPE       20.3911f     /* deg/s per unit c */
static float TRUTH_LO = 0.145f;
static float TRUTH_HI = 0.212f;
#define RUN_SAMPLES 150          /* 3 s at the 50 Hz fusion rate */
#define RUNS        20
#define SETTLED     15           /* runs after which we call it settled */

static float res[20000];
static int nres;
static unsigned long long rs = 88172645463325252ULL;

static float urand(void) {                      /* xorshift, reproducible */
    rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
    return (float)((rs >> 11) & 0xFFFFFF) / 16777216.0f;
}

typedef struct { float final_c, err_c, yaw, drift; } result_t;

/* One session: RUNS x 3 s, c carried over, filter resynced each run (which is
 * what the firmware does -- bench_learning is false outside BENCH_RUN). */
static result_t trial(float c0, float truth, int adaptive)
{
    trim_learn_cfg_t cfg = { .c_init=c0, .c_min=0.05f, .c_max=0.40f,
        .deadband_dps=0.5f, .step_per_s=0.005f, .yaw_tau_s=2.0f,
        .min_throttle=0.15f, .reject_dps=10.0f };
    trim_learn_t s; trim_learn_init(&s, &cfg);
    uint32_t seq = 0;
    double yawsum = 0, drift = 0; int yawn = 0, dn = 0;

    for (int run = 0; run < RUNS; ++run) {
        s.initialized = false;
        /* a whole 3 s window of real noise, chosen at random: keeps the
         * within-run correlation and the bounces intact */
        int off = (int)(urand() * (float)(nres - RUN_SAMPLES - 1));
        float before = s.c;
        for (int k = 0; k < RUN_SAMPLES; ++k) {
            float yaw = SLOPE * (s.c - truth) + res[off + k];
            if (adaptive)
                trim_learn_update(&s, &cfg, ++seq, 0.02f, yaw, 0.30f, false, true);
            if (run >= SETTLED) { yawsum += fabs(SLOPE*(s.c-truth)); yawn++; }
        }
        if (run >= SETTLED) { drift += fabs(s.c - before); dn++; }
    }
    result_t r = { s.c, fabsf(s.c - truth),
                   (float)(yawsum/yawn), (float)(drift/dn) };
    return r;
}

static int cmpf(const void *a, const void *b)
{ float x=*(const float*)a, y=*(const float*)b; return x<y?-1:(x>y); }

static void report(const char *tag, result_t *v, int n)
{
    float *fc=malloc(n*sizeof(float)), *ec=malloc(n*sizeof(float)),
          *yw=malloc(n*sizeof(float)); double dr=0;
    for (int i=0;i<n;i++){ fc[i]=v[i].final_c; ec[i]=v[i].err_c;
                           yw[i]=v[i].yaw; dr+=v[i].drift; }
    qsort(fc,n,sizeof(float),cmpf); qsort(ec,n,sizeof(float),cmpf);
    qsort(yw,n,sizeof(float),cmpf);
    printf("   %-22s %.3f [%.3f..%.3f]  %.4f/%.4f  %5.2f/%5.2f  %.4f\n",
           tag, fc[n/2], fc[n/20], fc[n-1-n/20],
           ec[n/2], ec[(int)(0.9*n)], yw[n/2], yw[(int)(0.9*n)], dr/n);
    free(fc); free(ec); free(yw);
}

int main(int argc, char **argv)
{
    FILE *f = fopen(argc>1?argv[1]:"residuals.txt","r");
    if (!f) { fprintf(stderr,"cannot open residual file\n"); return 1; }
    while (nres < 20000 && fscanf(f,"%f",&res[nres])==1) nres++;
    fclose(f);
    int N = argc>2 ? atoi(argv[2]) : 200;
    /* How far the TRUE c actually moves between sessions is the whole
     * question: if it never moves, a flashed number wins by definition. */
    if (argc>4) { TRUTH_LO=atof(argv[3]); TRUTH_HI=atof(argv[4]); }
    if (nres < RUN_SAMPLES*2) { fprintf(stderr,"too few residuals\n"); return 1; }

    printf("=== %d trials each. truth redrawn per trial from %.3f..%.3f ===\n",
           N, TRUTH_LO, TRUTH_HI);
    printf("   %d real noise samples, bootstrapped in 3 s windows\n\n", nres);
    printf("   %-22s %-25s %-15s %-13s %s\n",
           "", "final c  med[5-95%]", "|err| med/p90", "yaw med/p90", "drift/run");

    result_t *a = malloc(N*sizeof(result_t));
    for (int i=0;i<N;i++){ float t=TRUTH_LO+urand()*(TRUTH_HI-TRUTH_LO);
                           a[i]=trial(0.12f,t,1); }
    report("ADAPTIVE from 0.12", a, N);
    for (int i=0;i<N;i++){ float t=TRUTH_LO+urand()*(TRUTH_HI-TRUTH_LO);
                           a[i]=trial(0.24f,t,1); }
    report("ADAPTIVE from 0.24", a, N);
    printf("\n");
    float fixed[]={0.17f,0.18f,0.19f};
    for (int j=0;j<3;j++){
        char tag[40]; snprintf(tag,sizeof tag,"FIXED c=%.2f",fixed[j]);
        for (int i=0;i<N;i++){ float t=TRUTH_LO+urand()*(TRUTH_HI-TRUTH_LO);
                               a[i]=trial(fixed[j],t,0); }
        report(tag, a, N);
    }
    free(a);
    return 0;
}

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

#include "imu_sample.h"
#include "sample_snapshot.h"

static imu_sample_t make_sample(uint32_t sequence)
{
    return (imu_sample_t){
        .ax = (int16_t)sequence,
        .ay = (int16_t)(sequence + 1),
        .az = (int16_t)(sequence + 2),
        .gx = (int16_t)(sequence + 3),
        .gy = (int16_t)(sequence + 4),
        .gz = (int16_t)(sequence + 5),
        .mx = (int16_t)(sequence + 6),
        .my = (int16_t)(sequence + 7),
        .mz = (int16_t)(sequence + 8),
        .accel_gyro_valid = true,
        .mag_valid = true,
        .captured_us = (uint64_t)sequence * 1000U,
        .sequence = sequence,
    };
}

static void test_read_returns_one_complete_latest_generation(void)
{
    sample_snapshot_t box;
    sample_snapshot_init(&box);

    imu_sample_t one = make_sample(1);
    imu_sample_t two = make_sample(2);
    sample_snapshot_publish(&box, &one);
    sample_snapshot_publish(&box, &two);

    imu_sample_t out;
    assert(sample_snapshot_read(&box, &out));
    assert(out.sequence == 2);
    assert(out.ax == 2);
    assert(out.mx == 8);
    assert(out.captured_us == 2000);
}

typedef struct {
    sample_snapshot_t *box;
    atomic_bool done;
    atomic_bool torn_read;
} stress_state_t;

static void *publish_many_samples(void *arg)
{
    stress_state_t *state = arg;
    for (uint32_t sequence = 1; sequence <= 100000; ++sequence) {
        imu_sample_t sample = make_sample(sequence);
        sample_snapshot_publish(state->box, &sample);
    }
    atomic_store(&state->done, true);
    return NULL;
}

static void *read_many_samples(void *arg)
{
    stress_state_t *state = arg;
    imu_sample_t out;
    do {
        if (!sample_snapshot_read(state->box, &out)) {
            continue;
        }
        uint32_t sequence = out.sequence;
        if (out.ax != (int16_t)sequence ||
            out.ay != (int16_t)(sequence + 1) ||
            out.gz != (int16_t)(sequence + 5) ||
            out.mx != (int16_t)(sequence + 6) ||
            out.mz != (int16_t)(sequence + 8) ||
            out.captured_us != (uint64_t)sequence * 1000U) {
            atomic_store(&state->torn_read, true);
            break;
        }
    } while (!atomic_load(&state->done));
    return NULL;
}

static void test_reader_never_observes_mixed_generations_under_stress(void)
{
    sample_snapshot_t box;
    sample_snapshot_init(&box);
    stress_state_t state = {
        .box = &box,
        .done = ATOMIC_VAR_INIT(false),
        .torn_read = ATOMIC_VAR_INIT(false),
    };
    pthread_t writer;
    pthread_t reader;

    assert(pthread_create(&writer, NULL, publish_many_samples, &state) == 0);
    assert(pthread_create(&reader, NULL, read_many_samples, &state) == 0);
    assert(pthread_join(writer, NULL) == 0);
    assert(pthread_join(reader, NULL) == 0);
    assert(!atomic_load(&state.torn_read));
}

int main(void)
{
    test_read_returns_one_complete_latest_generation();
    test_reader_never_observes_mixed_generations_under_stress();
    return 0;
}

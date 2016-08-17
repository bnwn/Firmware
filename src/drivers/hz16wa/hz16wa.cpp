/****************************************************************************
 *
 *   Copyright (c) 2016 Enigma Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file hz16wa.cpp
 *
 * Driver for the flowmeter hz16wa connected via PWM.
 */

#include "hz16wa.h"


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

HZ16WA::HZ16WA(const char *path) :
    CDev("HZ16WA", path),
    _work{},
    _reports(nullptr),
    _class_instance(-1),
    _orb_class_instance(-1),
    _pwmSub(-1),
    _pwm{},
    _flowmeter_sensor_topic(nullptr),
    _flowmeter{},
    _sample_perf(perf_alloc(PC_ELAPSED, "hz16wa_read")),
    _read_errors(perf_alloc(PC_COUNT, "hz16wa_read_errors")),
    _buffer_overflows(perf_alloc(PC_COUNT, "hz16wa_buffer_overflows")),
    _sensor_zero_resets(perf_alloc(PC_COUNT, "hz16wa_zero_reset"))
{
}

HZ16WA::~HZ16WA()
{
    /* make sure we are truly inactive */
    stop();

    /* free any existing reports */
    if(_reports != nullptr) {
        delete _reports;
    }

    if(_class_instance != -1) {
        unregister_class_devname(FLOWMETER_BASE_DEVICE_PATH, _class_instance);
    }

    /* free perf counters */
    perf_free(_sample_perf);
    perf_free(_read_errors);
    perf_free(_buffer_overflows);
    perf_free(_sensor_zero_resets);
}

int HZ16WA::init()
{
    /* do regular cdev init */
    int ret = CDev::init();

    if(ret != OK) {
        return ERROR;
    }

    /* allocate basic report buffer */
    _reports = new ringbuffer::RingBuffer(2, sizeof(struct flowmeter_sensor_s));

    if(_reports == nullptr) {
        return ERROR;
    }

    _class_instance = register_class_devname(FLOWMETER_BASE_DEVICE_PATH);

    /* get a publish handle on the flowmeter_sensor topic */
    struct flowmeter_sensor_s fs_report = {};
    measure();
    _reports->get(fs_report);
    _flowmeter_sensor_topic = orb_advertise_multi(ORB_ID(flowmeter_sensor), &fs_report,
                                                  &_orb_class_instance, ORB_PRIO_DEFAULT);

    if(_flowmeter_sensor_topic == nullptr) {
        DEVICE_DEBUG("failed to create flowmeter_sensor. Did you start uORB?");
    }

    return OK;
}

void HZ16WA::print_info()
{
    perf_print_counter(_sample_perf);
    perf_print_counter(_read_errors);
    perf_print_counter(_buffer_overflows);
    perf_print_counter(_sensor_zero_resets);

    warnx("poll interval:   %u ticks.", _measure_ticks);
    warnx("flowrate:    %.3fL/min", (double)_flowmeter.Q);
}

void HZ16WA::print_registers()
{
    printf("Not supported in PWM mode!\n");
}

void HZ16WA::start()
{
    /* schedule a cycle tos start things */
    work_queue(HPWORK, &_work, (worker_t)&HZ16WA::cycle_trampoline, this, 1);
}

void HZ16WA::stop()
{
    work_cancel(HPWORK, &_work);
}

void HZ16WA::cycle_trampoline(void *arg)
{
    HZ16WA *dev = (HZ16WA *)arg;

    dev->cycle();
}

void HZ16WA::cycle()
{
    measure();

    /* schedule a fresh cycle call when the measurement is done */
    work_queue(HPWORK, &_work, (worker_t)&HZ16WA::cycle_trampoline, this, _measure_ticks);

    return;
}

int HZ16WA::measure()
{
    perf_begin(_sample_perf);

    if(OK != collect()) {
        DEVICE_DEBUG("collection error.");
        perf_count(_read_errors);
        perf_end(_sample_perf);
        return ERROR;
    }

    _flowmeter.timestamp = hrt_absolute_time();
    _flowmeter.max_Q = get_maximum_flowrate();
    _flowmeter.min_Q = get_minimum_flowrate();
    _flowmeter.Q = float(_pwm.period)*
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hz16wa_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace hmc5883
{
void	start(enum HMC5883_BUS busid, enum Rotation rotation);
bool	start_bus(struct hmc5883_bus_option &bus, enum Rotation rotation);
struct hmc5883_bus_option &find_bus(enum HMC5883_BUS busid);
void	test(enum HMC5883_BUS busid);
void	reset(enum HMC5883_BUS busid);
int	info(enum HMC5883_BUS busid);
int	calibrate(enum HMC5883_BUS busid);
int	temp_enable(HMC5883_BUS busid, bool enable);
void	usage();
} // namespace

int
hmc5883_main(int argc, char *argv[])
{
}

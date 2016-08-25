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

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hz16wa_main(int argc, char *argv[]);

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
	_input_rc_sub(-1),
	_rc{},
	_flowrate_arr{},
    _sample_perf(perf_alloc(PC_ELAPSED, "hz16wa_read")),
    _read_errors(perf_alloc(PC_COUNT, "hz16wa_read_errors")),
    _buffer_overflows(perf_alloc(PC_COUNT, "hz16wa_buffer_overflows")),
    _sensor_zero_resets(perf_alloc(PC_COUNT, "hz16wa_zero_reset")),
    _measure_ticks(0)
{
}

HZ16WA::~HZ16WA()
{
    /* make sure we are truly inactive */
    stop();

    /* free any existing reports */
    if (_reports != nullptr) {
        delete _reports;
    }

    if (_class_instance != -1) {
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

    if (ret != OK) {
        return ERROR;
    }

    /* allocate basic report buffer */
    _reports = new ringbuffer::RingBuffer(2, sizeof(struct flowmeter_sensor_s));

    if (_reports == nullptr) {
        return ERROR;
    }

    _class_instance = register_class_devname(FLOWMETER_BASE_DEVICE_PATH);

    /* get a publish handle on the flowmeter_sensor topic */
    struct flowmeter_sensor_s fs_report = {};
    measure();
    _reports->get(&fs_report);
    _flowmeter_sensor_topic = orb_advertise_multi(ORB_ID(flowmeter_sensor), &fs_report,
                                                  &_orb_class_instance, ORB_PRIO_DEFAULT);

    /* Subscribe input_rc topic */
    _input_rc_sub = orb_subscribe(ORB_ID(input_rc));

    if (_flowmeter_sensor_topic == nullptr) {
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
    warnx("flowrate:    %.3fL/min, PWM:     %.3fHz", (double)_flowmeter.flowrate, (double)_flowmeter.pluse_rate);
    warnx("flowmeter pesticide_remaining: %d .\n", _flowmeter.pesticide_remaining);
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

int HZ16WA::probe()
{
    /* Not use */
    return OK;
}

void HZ16WA::set_maximum_flowrate(const float max)
{
    _max_flowrate = max;
}

void HZ16WA::set_minimum_flowrate(const float min)
{
    _min_flowrate = min;
}

float HZ16WA::get_maximum_flowrate() const
{
    return _max_flowrate;
}

float HZ16WA::get_minimum_flowrate() const
{
    return _min_flowrate;
}

void HZ16WA::cycle()
{
	bool updated;
	orb_check(_input_rc_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(input_rc), _input_rc_sub, &_rc);

		if (_rc.channel_count > 8 && _rc.values[8] > _rc.PUMP_WORKING_PWM) {
			measure();
		}
	}

    /* schedule a fresh cycle call when the measurement is done */
    work_queue(HPWORK, &_work, (worker_t)&HZ16WA::cycle_trampoline, this, _measure_ticks);

    return;
}

int HZ16WA::measure()
{
    static uint64_t begin_times = hrt_absolute_time();
    _flowmeter.pesticide_remaining = true;
    perf_begin(_sample_perf);

    if (OK != collect()) {
        DEVICE_DEBUG("collection error or empty.");
        perf_count(_read_errors);
        perf_end(_sample_perf);

        _flowmeter.pluse_rate = 0;
        _flowmeter.flowrate = 0;

    } else {

        _flowmeter.pluse_rate = 1.0f / (float(_pwm.period) * 1e-6f);
        /* F(Hz) = [FLOWMETER_CONVERSION_COEFFICIENT * Q]. err = 10%. F = pulse_rate, Q = flowrate */
        _flowmeter.flowrate = _flowmeter.pluse_rate / FLOWMETER_CONVERSION_COEFFICIENT;
    }

    /* Reset sensor when flowrate < 0 */
    if (_flowmeter.flowrate < 0.0f) {
        perf_count(_sensor_zero_resets);
        perf_end(_sample_perf);
        return reset();
    }

    float max_flowrate = _flowmeter.flowrate;
    float min_flowrate = _flowmeter.flowrate;
    float sum_flowrate = _flowmeter.flowrate;
    /* Sliding filter and Extreme value filter */
    for (int i=0; i<9; i++) {
    	_flowrate_arr[i] = _flowrate_arr[i+1];
    	max_flowrate = _flowrate_arr[i] > max_flowrate ? _flowrate_arr[i] : max_flowrate;
    	min_flowrate = _flowrate_arr[i] < min_flowrate ? _flowrate_arr[i] : min_flowrate;
    	sum_flowrate += _flowrate_arr[i];
    }
    _flowrate_arr[9] = _flowmeter.flowrate;
    _flowmeter.flowrate = (sum_flowrate - max_flowrate - min_flowrate) / 8;

    _flowmeter.timestamp = hrt_absolute_time();
    _flowmeter.max_flowrate = get_maximum_flowrate();
    _flowmeter.min_flowrate = get_minimum_flowrate();

    if (_flowmeter.flowrate < 0.2f && (_flowmeter.timestamp - begin_times) > HZ16WA_EMPTY_TIMEOUT) {
        _flowmeter.pesticide_remaining = false;

    } else if (_flowmeter.flowrate >= 0.2f) {
        begin_times = hrt_absolute_time();
    }

    if (_flowmeter_sensor_topic != nullptr) {
        orb_publish(ORB_ID(flowmeter_sensor), _flowmeter_sensor_topic, &_flowmeter);
    }

    if (_reports->force(&_flowmeter)) {
        perf_count(_buffer_overflows);
    }

    poll_notify(POLLIN);
    perf_end(_sample_perf);
    return OK;
}

ssize_t HZ16WA::read(file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(struct flowmeter_sensor_s);
    struct flowmeter_sensor_s *rbuf = reinterpret_cast<struct flowmeter_sensor_s *>(buffer);
    int ret = 0;

    /* buffer must be enough large */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is enabled */
    if (_measure_ticks > 0) {
       /*
        * While there is space in the caller's buffer, and reports, copy them.
        * Note that we may be pre-empted by the workq thread while we are doing this;
        * we are careful to avoid racing with them.
        */
        while (count--) {
            if (_reports->get(rbuf)) {
                ret += sizeof(*rbuf);
                rbuf++;
            }
        }

        /* if there was no data, warn the caller */
        return ret ? ret : -EAGAIN;

    } else {

        _reports->flush();
        measure();

        if (_reports->get(rbuf)) {
            ret += sizeof(*rbuf);
        }
    }

    return ret;
}

int HZ16WA::ioctl(file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL: {
                    this->stop();
                    _measure_ticks = 0;
                    return OK;
                }

            /* external singalling not supported and zero would be bad */
            case SENSOR_POLLRATE_EXTERNAL:
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_DEFAULT:
            case SENSOR_POLLRATE_MAX: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* set interval for next measurement to minumum legal value */
                    _measure_ticks = USEC2TICK(HZ16WA_CONVERSION_INTERVAL);

                    /* if we need to start the poll state machine. do it */
                    if (want_start) {
                        this->start();
                    }

                    return OK;
                }

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start interval polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* convert Hz to tick interval via microsecond */
                    unsigned ticks = USEC2TICK(1000000 / arg);

                    /* check against maximum rate */
                    if (ticks < USEC2TICK(HZ16WA_CONVERSION_INTERVAL)) {
                        return -EINVAL;
                    }

                    /* update interval for next measurement */
                    _measure_ticks = ticks;

                    /* if we need to start the polling state machine. do it */
                    if (want_start) {
                        this->start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_measure_ticks == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return (1000 / _measure_ticks);

    case SENSORIOCRESET:
        this->reset();
        return OK;

    case FLOWMETERIOCSETMAXIMUMFLOWRATE:
        set_maximum_flowrate(*(float *)arg);
        return OK;

    case FLOWMETERIOCSETMINIMUMFLOWRATE:
        set_minimum_flowrate(*(float *)arg);
        return OK;

    default:
        return -EINVAL;
    }
}

int HZ16WA::collect()
{
    int fd = ::open(PWMIN0_DEVICE_PATH, O_RDONLY);

    if (-1 == fd) {
        return ERROR;
    }

    if (::read(fd, &_pwm, sizeof(_pwm)) == sizeof(_pwm)) {
        ::close(fd);
        return OK;
    }

    ::close(fd);
    return -EAGAIN;
}

int HZ16WA::reset()
{
    int fd = ::open(PWMIN0_DEVICE_PATH, O_RDONLY);

    if (-1 == fd) {
        return ERROR;
    }

    int ret = ::ioctl(fd, SENSORIOCRESET, 0);
    ::close(fd);
    return ret;
}

/**
 * Local functions in support of the shell command.
 */
namespace hz16wa
{
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

HZ16WA *g_dev;

void start();
void stop();
void test();
void reset();
void info();
void usage();

/**
 * Start the driver
 */
void start()
{
    int fd;

    if (g_dev != nullptr) {
       errx(1, "already started!");
    }

    /* create driver */
    g_dev = new HZ16WA(HZ16WA_DEVICE_PATH);

    if (g_dev == nullptr) {
        goto fail;
    }

    if (OK != g_dev->init()) {
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(HZ16WA_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        goto fail;
    }

    exit(0);

fail:

    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;
    }

    errx(1, "driver start failed!");
}

/**
 * Stop the driver
 */
void stop()
{
    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;
    } else {
        errx(1, "driver not running!");
    }

    exit(0);
}

/**
 * Perform some basic functional tests on the driver.
 * make sure we can collect data from the sensor in polled
 * and automatic mode.
 */
void test()
{
    struct flowmeter_sensor_s report;
    ssize_t sz;
    int ret;

    int fd = open(HZ16WA_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        errx(1, "%s open failed (try 'hz16wa start' if the driver is not running)", HZ16WA_DEVICE_PATH);
    }

    /* do a simple demand read */
    sz = read(fd, &report, sizeof(report));

    if (sz != sizeof(report)) {
        errx(1, "immediate read failed");
    }

    warnx("single read");
    warnx("measurement: %0.3f L/min of flowmeter, PWM: %0.3f Hz", (double) report.flowrate, (double) report.pluse_rate);
    warnx("time: %lld", report.timestamp);

    /* start the sensor polling at setting interval(Hz) */
    if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 20)) {
        errx(1, "falied to set poll rate!");
    }

    int i = 0;
    /* read the sensor x times and report each value */
    while(1) {
        struct pollfd fds;

        /* wait for data to be ready */
        fds.fd = fd;
        fds.events = POLLIN;
        ret = poll(&fds, 1, 2000);

        if (ret != 1) {
            warnx("timed out waiting for sensor data");
            continue;
        }

        /* go to get it */
        //sz = read(fd, &report, sizeof(report));
        orb_copy(ORB_ID(flowmeter_sensor), fd, &report);

//        if (sz != sizeof(report)) {
//            err(1, "periodic read failed");
//        }

        warnx("periodic read %d", ++i);

        /* Print the flowmeter report flowrate and pwm */
        warnx("measurement: %0.3f L/min of flowmeter, PWM: %0.3f Hz", (double) report.flowrate, (double) report.pluse_rate);
        warnx("timeL %lld", report.timestamp);
        warnx("flowmeter_pesticide_remaining:%d .\n", report.pesticide_remaining);
    }

    /* reset the sensor polling to default rate */
    if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
        errx(1, "failed to set default poll rate.");
    }

    errx(1, "test successful.");
}

/**
 * Reset the driver
 */
void reset()
{
    int fd = open(HZ16WA_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        err(1, "open failed.");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
       err(1, "driver reset failed.") ;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_EXTERNAL) < 0) {
        err(1, "driver poll restart failed.");
    }

    exit(0);
}

/**
 * Print info about the dirver
 */
void info()
{
    if(g_dev == nullptr) {
        errx(1, "driver not running.");
    }

    printf("state @ %p\n", g_dev);
    g_dev->print_info();

    exit(0);
}

/**
 * @brief usage
 */
void usage()
{
    warnx("missing command: try 'start', 'stop', 'test', 'info', 'reset'");
//    warnx("options:");
//    warnx("     -f polling interval(Hz)");
}

} // namespace


int hz16wa_main(int argc, char *argv[])
{
    const char *verb = argv[1];

    if (!strcmp(verb, "start")) {
        hz16wa::start();
    }

    if (!strcmp(verb, "stop")) {
        hz16wa::stop();
    }

    if (!strcmp(verb, "test")) {
        hz16wa::test();
    }

    if (!strcmp(verb, "info")) {
        hz16wa::info();
    }

    if (!strcmp(verb, "reset")) {
        hz16wa::reset();
    }

    hz16wa::usage();
    exit(1);
}

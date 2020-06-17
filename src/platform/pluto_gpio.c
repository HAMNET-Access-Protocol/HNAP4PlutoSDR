/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library;
 * if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA
 */

#define _GNU_SOURCE

#include "pluto_gpio.h"
#include "log.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#define EVENT_QUEUE_LEN 8

#define PLUTO_GPIO_WORKER_TH_CPUID 1
#define PLUTO_GPIO_WORKER_TH_PRIO 3

// event thread declaration
void* pin_ctrl_thread(void*);

// timespec to monitor thread wakeup delay
TIMECHECK_CREATE(thread_wakeup_mon);
TIMECHECK_CREATE(pin_write_mon);

// Structure for pin event
struct gpio_event {
    int fd;         // file descriptor to the pin
    char value;     // value to be set
    struct timespec sched_time;    // abs time when event is scheduled
};


// GPIO pin abstraction
struct gpio_pin_s {
    int id;                 // ID of the pin (without base)
    char* sysfs_gpio_node;  // string to the gpio pin sysfs device node
    int value_fd;            // file descriptor to write the pin value to
    int direction;          // in/out
    pthread_t pin_thread;   // reference to the pinctrl thread
    pthread_cond_t cond;    //
    pthread_mutex_t mutex;  // mutex and condition to signal pinctrl thread
    int thread_stop_signal; // kill signal for pinctrl thread
    struct gpio_event event_q[EVENT_QUEUE_LEN];
    int num_events_queued;
};


int gpio_event_cmp(const void* elem1, const void* elem2)
{
    struct gpio_event* event1,*event2;
    event1 = (struct gpio_event*)elem1;
    event2 = (struct gpio_event*)elem2;

    if (event1->sched_time.tv_sec == event2->sched_time.tv_sec)
        return event2->sched_time.tv_nsec - event1->sched_time.tv_nsec;
    else
        return event2->sched_time.tv_sec - event1->sched_time.tv_sec;
}

gpio_pin pluto_gpio_init(int pin_id, int direction)
{
    char tmpstr[80] = {0};
    gpio_pin pin = malloc(sizeof(struct gpio_pin_s));
    pin->id = pin_id;

    // create gpio device entry
    int fd = open("/sys/class/gpio/export",O_WRONLY);
    sprintf(tmpstr,"%d",PLUTO_GPIO_BASE+pin_id);
    write(fd,tmpstr,3);
    close(fd);

    char* basestr = "/sys/class/gpio/gpio";

    // set I/O direction of the pin
    sprintf(tmpstr,"%s%d/direction",basestr,PLUTO_GPIO_BASE+pin_id);
    fd = open(tmpstr,O_WRONLY);
    if (direction==IN) {
        write(fd, "in", 2);
        pin->direction = IN;
    } else if (direction==OUT) {
        write(fd, "out", 3);
        pin->direction = OUT;
    } else
        printf("Error initializing GPIO pin! Unknown direction");
    close(fd);

    // store path of the device entry value file
    pin->sysfs_gpio_node = calloc(strlen(basestr)+32,1);
    sprintf(pin->sysfs_gpio_node,"%s%d/value",basestr,PLUTO_GPIO_BASE+pin_id);
    pin->value_fd = open(pin->sysfs_gpio_node, O_WRONLY);
    char d = '0';
    write(pin->value_fd, &d, 1);

    pin->num_events_queued = 0;

    pin->thread_stop_signal = 0;
    pthread_cond_init(&pin->cond,NULL);
    pthread_mutex_init(&pin->mutex,NULL);
    pthread_create(&pin->pin_thread, NULL, pin_ctrl_thread, pin);

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(PLUTO_GPIO_WORKER_TH_CPUID,&cpu_set);
    struct sched_param prio_rt_high;
    prio_rt_high.sched_priority = PLUTO_GPIO_WORKER_TH_PRIO;    // highest prio for gpio control thread
    pthread_setaffinity_np(pin->pin_thread,sizeof(cpu_set_t),&cpu_set);
    pthread_setschedparam(pin->pin_thread, SCHED_FIFO, &prio_rt_high);


    TIMECHECK_INIT(pin_write_mon,"gpio.write",1000);
    TIMECHECK_INIT(thread_wakeup_mon,"gpio.thread_wake",1000);
    return pin;

}

void pluto_gpio_destroy(gpio_pin pin)
{
    pin->thread_stop_signal = 1;
    void** ret = NULL;
    pthread_join(pin->pin_thread,ret);

    close(pin->value_fd);

    // delete gpio device entry
    char tmpstr[64];
    int fd = open("/sys/class/gpio/unexport",O_WRONLY);
    sprintf(tmpstr,"%d",PLUTO_GPIO_BASE+pin->id);
    write(fd,tmpstr,3);
    close(fd);

    free(pin->sysfs_gpio_node);
    free(pin);
}

// pin write functions
void pluto_gpio_pin_write(gpio_pin pin, int level)
{
    if (pin->direction!=OUT)
        return;

    return pluto_gpio_pin_write_delayed(pin,level,0);
}

void pluto_gpio_pin_write_delayed(gpio_pin pin, int level, int delay_us) {
    if (pin->direction != OUT) {
        LOG(WARN,"[Platform] pin %d is not configured for output!\n",pin->id);
        return;
    }

    if (pin->num_events_queued==EVENT_QUEUE_LEN) {
        LOG(WARN,"[Platform] cannot enqueue gpio pin %d event! queue full\n",pin->id);
        return;
    }
    pthread_mutex_lock(&pin->mutex);

    struct gpio_event* new_event = &pin->event_q[pin->num_events_queued];
    if (level==LOW)
        new_event->value = '0';
    else
        new_event->value = '1';
    new_event->fd = pin->value_fd;
    struct timespec time;
    clock_gettime(CLOCK_REALTIME,&time);
    time.tv_nsec += delay_us*1000;
    if (time.tv_nsec>1000000000) {
        time.tv_sec++;
        time.tv_nsec-=1000000000;
    }
    new_event->sched_time = time;
    pin->num_events_queued++;
    TIMECHECK_START(thread_wakeup_mon);
    pthread_cond_signal(&pin->cond);
    pthread_mutex_unlock(&pin->mutex);
}

// Main Event thread for timed GPIO pin control.
// Adds incoming events to a queue and works off events at their scheduled time
// This thread mainly operates on a pthread_cond_timedwait().
// The condition is set, if a new event has to be scheduled.
// The timedwait is set to the expiration time of the next event, if it fires the event is handled.
void* pin_ctrl_thread(void* arg)
{
    gpio_pin pin = arg;
    struct timespec next_event_time;
    clock_gettime(CLOCK_REALTIME,&next_event_time);
    next_event_time.tv_sec += 3600; //  no events scheduled initially. expire long time in future

    while (!pin->thread_stop_signal) {
        pthread_mutex_lock(&pin->mutex);
        int ret = pthread_cond_timedwait(&pin->cond,&pin->mutex,&next_event_time);

        if (ret==ETIMEDOUT) {
            // thread woke up due to upcoming event. Handle it
            struct gpio_event* event =  &pin->event_q[pin->num_events_queued-1];
            pin->num_events_queued--;
            TIMECHECK_START(pin_write_mon);
            write(pin->value_fd,&event->value, 1);
            TIMECHECK_STOP(pin_write_mon);

            TIMECHECK_INFO(pin_write_mon);
        } else {
            // Thread woke up due to event scheduling. Monitor delay
            TIMECHECK_STOP(thread_wakeup_mon);
            TIMECHECK_INFO(thread_wakeup_mon);
        }
        // Schedule next event.
        if (pin->num_events_queued>0) {
            // sort events
            qsort(pin->event_q,pin->num_events_queued, sizeof(struct gpio_event),gpio_event_cmp);
            // next event is last element in the list
            next_event_time = pin->event_q[pin->num_events_queued - 1].sched_time;
        } else {
            clock_gettime(CLOCK_REALTIME,&next_event_time);
            next_event_time.tv_sec += 3600; //  no events scheduled. set expire to long time in future
        }
        pthread_mutex_unlock(&pin->mutex);
    }
    return NULL;
}
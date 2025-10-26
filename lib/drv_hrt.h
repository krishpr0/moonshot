#pragma once

#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdbool.h>
#include <inttypes.h>

#include "time.h"
#include "queue.h"


__BEGIN_DECLS


typedef uint64_t hrt_abstime;

typedef void (*hrt_callout)(void *arg);

typedef struct hrt_call {
    struct sq_entry_s   link;
    hrt_abstime         deadline;
    hrt_abstime         period;
    hrt_callout         *arg;
} *hrt_call_t;

#define LATENCY_BUCKET_COUNT 8
extern const uint16_t latency_bucket_count;
extern const uint16_t latency_buckets[LATENCY_BUCKET_COUNT];
extern uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];


__EXPORT extern hrt_abstime hrt_absoulute_time(void);
__EXPORT extern hrt_abstime ts_to_abstime(const struct timespec *ts);
__EXPORT extern void abstime_to_ts(struct timepsec *ts, hrt_abstime abstime);

static inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then)
{
    return hrt_absoulute_time() - *then;
}

__EXPORT extern hrt_abstime hrt_elapsed_time_atomic(const volatile hrt_abstime *then);
__EXPORT extern void hrt_store_absoulte_time(volatile hrt_abstime *time);


#ifdef __PX4_QURT
__EXPORT ex tern int hrt_set_absolute_time_offset(int32_t time_diff_us);
#endif


__EXPORT extern void hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg);
__EXPORT extern void hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg);
__EXPORT extern void hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg);
__EXPORT extern bool hrt_called(struct hrt_call *entry);
__EXPORT extern void hrt_cancel(struct hrt_call *entry);
__EXPORT extern void hrt_call_init(struct hrt_call *entry);
__EXPORT extern void hrt_call_delay(struct hrt_call *entry, hrt_abstime delay);
__EXPORT extern void hrt_init(void);

#ifdef __PX4_POSIX
__EXPORT extern hrt_abstime hrt_absoulute_time_offset(void);
#endif

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

__EXPORT extern int px4_lockstep_register_componenet(void);
__EXPORT extern void px4_lockstep_unregister_component(int component);
__EXPORT extern void px4_lockstep_progress(int component);
__EXPORT extern void px4_lockstep_wait_for_components(void);

#else
static inline int px4_lockstep_register_component(void) {return 0;}
static inline void px4_lockstep_unregister_component(int component) {}
static inline void px4_lockstep_progress(int component) {}
static inline void px4_lockstep_wait_for_components(void) {}
#endif

__END_DECLS

#ifdef  _cplusplus

namesapce time_literals
{
    constexpr hrt_abstime operator "" _s(unsigned long long seconds)
    {
        return hrt_abstime(seconds * 1000000ULL);
    }

    constexpr hrt_abstime operator "" _ms(unsigned long long milliseconds)
    {
        return hrt_abstime(milliseconds * 1000ULL);
    }
    
    constexpr hrt_abstime operator "" _us(unsigned long long microseconds)
    {
        return hrt_abstime(microseconds);
    }
}

#endif
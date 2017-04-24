#ifndef MN_TIMER_H_INC
#define MN_TIMER_H_INC

typedef uint32_t TIMER_TICK_T;

typedef struct timer_info_s {
   TIMER_TICK_T timer_start;
   TIMER_TICK_T timer_end;
   uint8_t timer_wrap;
} TIMER_INFO_T;
void Init_WDT(uint16_t toutInSecs);
typedef TIMER_INFO_T * PTIMER_INFO;

void Set_Timer(PTIMER_INFO, TIMER_TICK_T);
uint8_t mn_timer_expired(PTIMER_INFO);
TIMER_TICK_T mn_get_timer_tick(void);
void mn_wait_ticks(TIMER_TICK_T num_ticks);

#define MN_TICK_UPDATE              ++timer_tick
#define MN_GET_TICK                 (mn_get_timer_tick())

#define TICKRATE_HZ1                (100)
#define WATCHDOG_TIMEOUT_IN_SECS    (5 * 60)

#define SECOND                  (100)
#define MINUTE                  (60 * SECOND)
#define HOUR                    (60 * MINUTE)

#endif   /* #ifndef MN_TIMER_H_INC */

/* ===========================================================================
 * atlics_v13_scheduler.c
 *
 * FreeRTOS (POSIX simulator) port of:
 *   "Autonomous_intersection"
 *
 * This is a 1:1 translation of the UPPAAL model into FreeRTOS tasks.
 * Every UPPAAL template becomes a task; every UPPAAL broadcast channel
 * becomes a binary semaphore; every shared UPPAAL global becomes a field
 * in the mutex-protected SystemState_t struct below.
 *
 *   UPPAAL template          -> FreeRTOS task
 *   ------------------------    --------------------------
 *   VehicleGenerator          -> vVehicleGeneratorTask
 *   Vehicle(vid) x4           -> vVehicleTask(vid)            (x4 instances)
 *   Scheduler                 -> vSchedulerTask                 (4D-argmax)
 *   IntersectionController    -> vIntersectionControllerTask
 *   EmergencyHandler           -> vEmergencyHandlerTask
 *
 *   UPPAAL channel            -> FreeRTOS object
 *   ------------------------    --------------------------
 *   schedule_propose/exc      -> xSchedExecDone   (binary sem)
 *   reserve_slot / execute    -> modelled inline inside the controller
 *   vehicle_arrive/request_*  -> xSchedWake        (binary sem, "wake up
 *                                 and re-check the Idle->Compute guard")
 *   emergency_detect /
 *   priority_override          -> g.emergency_flag (mutex-protected, set
 *                                  directly by vEmergencyHandlerTask)
 *
 * NOTE ON ONE FIDELITY FIX vs. the raw XML:
 *   In the UPPAAL model, transition id30 (Release->Free) sets
 *   "selectedVehicle = -1" as part of the SAME broadcast step that
 *   transition id16 (Wait->Idle, on schedule_exc?) uses to reset
 *   waitTime[selectedVehicle]/cost[selectedVehicle]/score[selectedVehicle].
 *   Depending on UPPAAL's internal update-evaluation order this can index
 *   waitTime[-1]. Here the serviced vehicle id is captured explicitly
 *   (g.lastServicedVehicle) BEFORE selectedVehicle is cleared, so the
 *   Scheduler always resets the correct vehicle's bookkeeping. Functionally
 *   equivalent to the model's intent, just safe against that ordering trap.
 *
 * Build:
 *   Drop this file into your existing FreeRTOS-Kernel POSIX-simulator
 *   project (the same one used for rtos_task_architecture_v2.c) in place
 *   of / alongside main.c, and build with your normal Makefile.
 *
 *   FreeRTOSConfig.h must have at least:
 *     configUSE_PREEMPTION              1
 *     configUSE_MUTEXES                 1
 *     configSUPPORT_DYNAMIC_ALLOCATION  1
 *     configMAX_PRIORITIES              >= 6
 * ===========================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ===================== UPPAAL CONSTANTS (verbatim) ===================== */
#define MAX_WAIT        4   /* queue_len upper bound                       */
#define SLOT_TIME       5   /* exec_clk >= SLOT_TIME to release            */
#define SCHED_DEADLINE  2   /* sched_clk <= SCHED_DEADLINE invariant       */
#define SENSOR_PERIOD   1   /* VehicleGenerator sampling period            */
#define MAX_VEHICLES    4

/* Real-time scale: 1 UPPAAL time unit = SIM_TICK_MS milliseconds.
 * Tune this to make the console demo watchable (200ms -> SLOT_TIME = 1s). */
#define SIM_TICK_MS     200

/* ===================== MOVE -> QUADRANT REQUIREMENTS ====================
 * M1: Q2+Q3   M2: Q4+Q1   M3: Q4+Q3   M4: Q2+Q1
 * M5: Q2+Q1   M6: Q4+Q3   M7: Q4+Q3   M8: Q2+Q1
 * M9: Q2+Q3   M10:Q2+Q3   M11:Q4+Q1   M12:Q4+Q1
 * ========================================================================= */
#define M1   1   /* N->S */
#define M2   2   /* S->N */
#define M3   3   /* E->W */
#define M4   4   /* W->E */
#define M5   5   /* N->E */
#define M6   6   /* E->S */
#define M7   7   /* S->W */
#define M8   8   /* W->N */
#define M9   9   /* N->W */
#define M10 10   /* W->S */
#define M11 11   /* S->E */
#define M12 12   /* E->N */

static const char *MOVE_NAMES[13] = {
    "?",
    "N->S", "S->N", "E->W", "W->E",
    "N->E", "E->S", "S->W", "W->N",
    "N->W", "W->S", "S->E", "E->N"
};
static const char *moveName(int m) { return (m >= 1 && m <= 12) ? MOVE_NAMES[m] : "?"; }

/* ===================== SHARED STATE (== UPPAAL globals) ================= */
typedef struct {
    int  queue_len;
    int  emergency_flag;
    bool emergency_active;
    bool system_busy;
    bool has_proposal;
    int  selectedVehicle;
    int  generatedMove;
    bool Q1, Q2, Q3, Q4;

    int  vehicleMove[MAX_VEHICLES];
    bool vehicleActive[MAX_VEHICLES];
    int  vehicleRequest[MAX_VEHICLES];
    bool requestPending[MAX_VEHICLES];

    int  waitTime[MAX_VEHICLES];
    int  cost[MAX_VEHICLES];
    int  score[MAX_VEHICLES];

    int  lastServicedVehicle;
} SystemState_t;

static SystemState_t g = {
    .selectedVehicle    = -1,
    .generatedMove      = 1,
    .lastServicedVehicle = -1
};

static SemaphoreHandle_t xStateMutex;       /* protects struct g            */
static SemaphoreHandle_t xSchedWake;        /* vehicle_arrive/request_next  */
static SemaphoreHandle_t xIntersectionWake; /* schedule_propose             */
static SemaphoreHandle_t xSchedExecDone;    /* schedule_exc                 */

/* ===================== HELPERS (== UPPAAL guards/updates) =============== */

/* Quadrants required by a move, exactly matching IntersectionController
 * transitions id24 (3,6,7->Q4+Q3), id25 (1,9,10->Q2+Q3),
 * id26 (4,5,8->Q2+Q1), id28 (2,11,12->Q4+Q1). */
static bool quadrants_required(int move, bool *q1, bool *q2, bool *q3, bool *q4)
{
    *q1 = *q2 = *q3 = *q4 = false;
    switch (move) {
        case M1: case M9:  case M10: *q2 = true; *q3 = true; return true;
        case M2: case M11: case M12: *q4 = true; *q1 = true; return true;
        case M3: case M6:  case M7:  *q4 = true; *q3 = true; return true;
        case M4: case M5:  case M8:  *q2 = true; *q1 = true; return true;
        default: return false;
    }
}

/* Aging update, transition id19 (Compute->Propose), part 1:
 * waitTime[i] = waitTime[i] + 1 for ALL i, unconditionally. */
static void age_all(void)
{
    for (int i = 0; i < MAX_VEHICLES; i++) {
        g.waitTime[i] += 1;
        g.score[i] = g.waitTime[i] - g.cost[i];
    }
}

/* 4D-argmax selection, transition id19 (Compute->Propose), part 2.
 * Movement-conflict-cost-aware score = waitTime - cost.
 * Tie-break favours the lowest vehicle index (matches the ">=" chain
 * in the XML, evaluated vehicle 0 first). */
static int select_vehicle_argmax(void)
{
    bool a0 = g.requestPending[0] && g.vehicleActive[0];
    bool a1 = g.requestPending[1] && g.vehicleActive[1];
    bool a2 = g.requestPending[2] && g.vehicleActive[2];
    bool a3 = g.requestPending[3] && g.vehicleActive[3];

    if (a0 && (g.score[0] >= g.score[1] || !a1)
           && (g.score[0] >= g.score[2] || !a2)
           && (g.score[0] >= g.score[3] || !a3)) return 0;

    if (a1 && (g.score[1] >= g.score[0] || !a0)
           && (g.score[1] >= g.score[2] || !a2)
           && (g.score[1] >= g.score[3] || !a3)) return 1;

    if (a2 && (g.score[2] >= g.score[0] || !a0)
           && (g.score[2] >= g.score[1] || !a1)
           && (g.score[2] >= g.score[3] || !a3)) return 2;

    if (a3) return 3;

    return -1;
}

/* ===================== TASK: VehicleGenerator ============================
 * UPPAAL: Idle -[queue_len<MAX_WAIT && exists inactive i]-> Generate -> Idle
 * Cyclically emits M1..M12, round-robins.
 * ========================================================================= */
static void vVehicleGeneratorTask(void *pv)
{
    (void)pv;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD * SIM_TICK_MS));

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        int slot = -1;
        if (g.queue_len < MAX_WAIT) {
            for (int i = 0; i < MAX_VEHICLES; i++) {
                if (!g.vehicleActive[i]) { slot = i; break; }
            }
        }
        if (slot != -1) {
            int move = g.generatedMove;
            g.vehicleMove[slot]    = move;
            g.vehicleActive[slot]  = true;
            g.vehicleRequest[slot] = move;
            g.requestPending[slot] = true;
            g.waitTime[slot]       = 0;
            g.cost[slot]           = (move >= M1 && move <= M4) ? 1 : 2; /* straight=1, turn=2 */
            g.generatedMove        = (move == 12) ? 1 : move + 1;
            g.queue_len += 1;
            printf("[GEN ] vehicle %d requests M%-2d (%-5s) cost=%d queue_len=%d\n",
                   slot, move, moveName(move), g.cost[slot], g.queue_len);
        }
        xSemaphoreGive(xStateMutex);

        if (slot != -1) xSemaphoreGive(xSchedWake);
    }
}

/* ===================== TASK: Vehicle(vid) x4 ==============================
 * UPPAAL: idle -[active==true]-> Request -> waiting -[active==false]-> Done -> idle
 * Pure lifecycle watcher/logger; the actual flag flips happen in the
 * Generator and the IntersectionController, matching the model where this
 * template only re-confirms vehicleRequest/requestPending.
 * ========================================================================= */
static void vVehicleTask(void *pv)
{
    int vid = (int)(intptr_t)pv;
    bool wasActive = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SIM_TICK_MS));

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        bool isActive = g.vehicleActive[vid];
        int  move     = g.vehicleRequest[vid];
        xSemaphoreGive(xStateMutex);

        if (isActive && !wasActive) {
            printf("        vehicle %d: idle -> Request -> waiting (M%d)\n", vid, move);
        } else if (!isActive && wasActive) {
            printf("        vehicle %d: waiting -> Done -> idle\n", vid);
        }
        wasActive = isActive;
    }
}

/* ===================== TASK: Scheduler ====================================
 * UPPAAL: Idle -> Compute -> Propose -> Wait -> Idle (on schedule_exc?)
 * ========================================================================= */
static void vSchedulerTask(void *pv)
{
    (void)pv;
    for (;;) {
        /* ---- Idle: block until something might have changed ---- */
        xSemaphoreTake(xSchedWake, portMAX_DELAY);

        for (;;) {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);

            bool pendingExists = false;
            for (int i = 0; i < MAX_VEHICLES; i++)
                if (g.requestPending[i] && g.vehicleActive[i]) { pendingExists = true; break; }

            bool canEnterCompute = (g.emergency_flag == 0 && g.queue_len > 0 && pendingExists);
            if (!canEnterCompute) { xSemaphoreGive(xStateMutex); break; /* back to Idle */ }

            if (g.system_busy) {
                /* sched_clk <= SCHED_DEADLINE invariant: still allowed to
                 * wait briefly for system_busy==false guard. */
                xSemaphoreGive(xStateMutex);
                vTaskDelay(pdMS_TO_TICKS(SIM_TICK_MS / 4));
                continue;
            }

            /* ---- Compute -> Propose ---- */
            age_all();
            int sel = select_vehicle_argmax();
            g.selectedVehicle = sel;
            g.has_proposal     = (sel != -1);

            if (sel != -1) {
                printf("[SCHED] selected vehicle %d (M%d) score=%d  "
                       "[V0 wt=%d c=%d s=%d | V1 wt=%d c=%d s=%d | "
                       "V2 wt=%d c=%d s=%d | V3 wt=%d c=%d s=%d]\n",
                       sel, g.vehicleRequest[sel], g.score[sel],
                       g.waitTime[0], g.cost[0], g.score[0],
                       g.waitTime[1], g.cost[1], g.score[1],
                       g.waitTime[2], g.cost[2], g.score[2],
                       g.waitTime[3], g.cost[3], g.score[3]);
            }
            xSemaphoreGive(xStateMutex);

            if (sel != -1) xSemaphoreGive(xIntersectionWake); /* schedule_propose */

            /* ---- Wait: block until IntersectionController finishes ---- */
            xSemaphoreTake(xSchedExecDone, portMAX_DELAY);

            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            int serviced = g.lastServicedVehicle;
            if (serviced != -1) {
                g.waitTime[serviced] = 0;
                g.cost[serviced]     = 0;
                g.score[serviced]    = 0;
            }
            xSemaphoreGive(xStateMutex);
            break; /* back to Idle */
        }
    }
}

/* ===================== TASK: IntersectionController =======================
 * UPPAAL: Free -> Reserve -> Execute -> Release -> Free (schedule_exc!)
 * ========================================================================= */
static void vIntersectionControllerTask(void *pv)
{
    (void)pv;
    for (;;) {
        xSemaphoreTake(xIntersectionWake, portMAX_DELAY);

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        int v = g.selectedVehicle;
        bool ok = (g.queue_len > 0 && !g.system_busy && g.emergency_flag == 0 &&
                   g.has_proposal && v >= 0 && v < MAX_VEHICLES);

        bool q1n = false, q2n = false, q3n = false, q4n = false;
        if (ok) ok = quadrants_required(g.vehicleRequest[v], &q1n, &q2n, &q3n, &q4n);
        if (ok) ok = !((q1n && g.Q1) || (q2n && g.Q2) || (q3n && g.Q3) || (q4n && g.Q4));

        if (!ok) { xSemaphoreGive(xStateMutex); continue; }

        /* ---- Free -> Reserve ---- */
        g.queue_len    -= 1;
        g.system_busy   = true;
        g.has_proposal  = false;
        int move = g.vehicleRequest[v];

        /* ---- Reserve -> Execute (reserve_slot!) ---- */
        if (q1n) g.Q1 = true;
        if (q2n) g.Q2 = true;
        if (q3n) g.Q3 = true;
        if (q4n) g.Q4 = true;
        printf("[INTX] RESERVE vehicle %d M%-2d (%-5s)  Q1=%d Q2=%d Q3=%d Q4=%d\n",
               v, move, moveName(move), g.Q1, g.Q2, g.Q3, g.Q4);
        xSemaphoreGive(xStateMutex);

        /* ---- Execute: exec_clk >= SLOT_TIME ---- */
        vTaskDelay(pdMS_TO_TICKS(SLOT_TIME * SIM_TICK_MS));

        /* ---- Execute -> Release ---- */
        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        g.system_busy = false;
        g.Q1 = g.Q2 = g.Q3 = g.Q4 = false;
        g.requestPending[v]  = false;
        g.vehicleRequest[v]  = -1;
        g.vehicleActive[v]   = false;
        g.lastServicedVehicle = v;   /* captured BEFORE clearing selectedVehicle */
        g.selectedVehicle    = -1;
        printf("[INTX] RELEASE vehicle %d, intersection free, queue_len=%d\n", v, g.queue_len);
        xSemaphoreGive(xStateMutex);

        /* ---- Release -> Free (schedule_exc!) ---- */
        xSemaphoreGive(xSchedExecDone);
        xSemaphoreGive(xSchedWake); /* let the scheduler re-check immediately */
    }
}

/* ===================== TASK: EmergencyHandler ==============================
 * UPPAAL: Normal -[emergency_detect?]-> Emergency -[priority_override!]-> Normal
 * Randomly injects emergencies for the console demo; replace the random
 * trigger with a GPIO/IRQ hook on real hardware.
 * ========================================================================= */
static void vEmergencyHandlerTask(void *pv)
{
    (void)pv;
    srand((unsigned)time(NULL));
    for (;;) {
        int idleMs = 4000 + (rand() % 6000);
        vTaskDelay(pdMS_TO_TICKS(idleMs));

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        g.emergency_flag    = 1;
        g.emergency_active  = true;
        xSemaphoreGive(xStateMutex);
        printf("[EMRG] *** emergency_detect: new proposals/reservations paused ***\n");

        int durMs = 1000 + (rand() % 1500);
        vTaskDelay(pdMS_TO_TICKS(durMs));

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        g.emergency_flag   = 0;
        g.emergency_active = false;
        xSemaphoreGive(xStateMutex);
        printf("[EMRG] priority_override: normal operation resumed\n");

        xSemaphoreGive(xSchedWake);
    }
}

/* ===================== TASK: Stats (not in the XML; demo dashboard) ====== */
static void vStatsTask(void *pv)
{
    (void)pv;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        printf("---- STATUS queue_len=%d busy=%d emergency=%d Q[%d %d %d %d] selected=%d ----\n",
               g.queue_len, g.system_busy, g.emergency_flag,
               g.Q1, g.Q2, g.Q3, g.Q4, g.selectedVehicle);
        xSemaphoreGive(xStateMutex);
    }
}

/* ===================== main =============================================== */
int main(void)
{
    xStateMutex       = xSemaphoreCreateMutex();
    xSchedWake        = xSemaphoreCreateBinary();
    xIntersectionWake = xSemaphoreCreateBinary();
    xSchedExecDone    = xSemaphoreCreateBinary();

    configASSERT(xStateMutex != NULL);
    configASSERT(xSchedWake != NULL);
    configASSERT(xIntersectionWake != NULL);
    configASSERT(xSchedExecDone != NULL);

    xTaskCreate(vVehicleGeneratorTask, "Gen",  configMINIMAL_STACK_SIZE * 4, NULL, 2, NULL);
    for (int i = 0; i < MAX_VEHICLES; i++) {
        xTaskCreate(vVehicleTask, "Veh", configMINIMAL_STACK_SIZE * 4, (void *)(intptr_t)i, 1, NULL);
    }
    xTaskCreate(vSchedulerTask,             "Sched", configMINIMAL_STACK_SIZE * 4, NULL, 4, NULL);
    xTaskCreate(vIntersectionControllerTask, "Intx",  configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
    xTaskCreate(vEmergencyHandlerTask,       "Emrg",  configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    xTaskCreate(vStatsTask,                  "Stats", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);

    printf("=== ATLICS V13 (movement conflict-cost-aware 4D-argmax) starting ===\n");

    vTaskStartScheduler();

    /* Only reached if vTaskStartScheduler() fails (out of heap). */
    for (;;);
    return 0;
}

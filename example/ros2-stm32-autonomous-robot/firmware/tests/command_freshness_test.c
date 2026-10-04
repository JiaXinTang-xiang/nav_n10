/* Bounded host check of production command/safety code; no hardware actuation.
 * cc -std=c11 -Wall -Wextra -I firmware/App/Inc
 *    firmware/tests/command_freshness_test.c -o /tmp/command_freshness_test
 * /tmp/command_freshness_test
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Substitute only the platform and actuator dependencies. */
#define APP_PLATFORM_H
static uint32_t tick;
static bool locked;
static bool motor_authorized;
static uint32_t faults;
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t AppPlatform_IrqLock(void)
{
  assert(!locked);
  locked = true;
  return 0;
}
static void AppPlatform_IrqUnlock(uint32_t key)
{
  (void)key;
  assert(locked);
  locked = false;
}
static uint32_t AppPlatform_ElapsedMs(uint32_t now, uint32_t then)
{
  return now - then;
}

#include "../App/Src/app_command.c"
#include "../App/Src/app_safety.c"

static AppSystemState state;
void AppState_SetFault(uint32_t flags) { faults |= flags; }
void AppState_ClearFault(uint32_t flags) { faults &= ~flags; }
bool AppState_HasCriticalFault(void) { return (faults & APP_FAULT_CRITICAL_MASK) != 0; }
void AppState_SetSystemState(AppSystemState value, uint32_t now)
{
  (void)now;
  state = value;
}
AppSystemState AppState_GetSystemState(void) { return state; }
void AppMotor_ForceSafe(void) { motor_authorized = false; }
void AppMotor_SetAuthorized(bool value) { motor_authorized = value; }
void AppControl_Reset(void) {}
bool AppControl_AllConfigured(void) { return true; }
bool AppDrivetrain_WheelControlReady(void) { return true; }
bool AppDrivetrain_BodyCommandReady(void) { return true; }
bool AppEncoder_AllValid(uint32_t now) { (void)now; return true; }

static void check_freshness(uint32_t received)
{
  AppCommandSnapshot command;
  faults = 0;
  AppCommand_Init();
  AppSafety_Init();
  AppSafety_SetInitializationComplete(true);
  tick = received;
  assert(AppCommand_SubmitBodyVelocity(APP_COMMAND_SOURCE_FDCAN, 0.0f, 0.0f,
                                       APP_COMMAND_DEFAULT_TIMEOUT_MS));
  AppCommand_RequestArm();
  /* Communication cycle captured its time before the command was submitted. */
  AppSafety_Update(received - 1U);
  assert(faults == 0 && motor_authorized);
  assert(AppSafety_ValidateActiveCommand(received - 1U, &command));
  tick = received + 250U;
  AppSafety_Update(tick);
  assert(faults == 0 && motor_authorized);
  tick = received + 251U;
  AppSafety_Update(tick);
  assert(faults == APP_FAULT_COMMAND_TIMEOUT && !motor_authorized);
  assert(state == APP_SYSTEM_FAULT);
  /* Returning traffic cannot clear the latch or restore motor permission. */
  assert(AppCommand_SubmitBodyVelocity(APP_COMMAND_SOURCE_FDCAN, 0.0f, 0.0f,
                                       APP_COMMAND_DEFAULT_TIMEOUT_MS));
  AppSafety_Update(tick);
  assert(faults == APP_FAULT_COMMAND_TIMEOUT && !motor_authorized);
  AppCommand_GetSnapshot(tick, &command);
  assert(!command.valid && !command.timed_out && !command.arm_requested);
}

int main(void)
{
  check_freshness(1001U);
  check_freshness(0U); /* acceptance across UINT32_MAX -> 0 */
  check_freshness(UINT32_MAX - 100U); /* genuine expiry across wrap */
  puts("PASS: stale caller time, 250/251 ms boundary, wrap, safe fault latch");
  return 0;
}

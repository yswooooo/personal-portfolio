# RC Channel Polarity Normalization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Normalize both joystick horizontal channels as left-positive at the SBUS receive boundary while preserving the current physical chassis steering direction.

**Architecture:** Treat channel polarity as an RC hardware mapping concern in `bsp_rc.h` and apply it exactly once while publishing `g_rc`. Keep the filter and differential-drive equations unchanged, and compensate the prior APP-layer inversion by setting `APP_CHASSIS_CMD_POLARITY` to positive.

**Tech Stack:** C11, STM32H723 HAL, SBUS over UART5 DMA+IDLE, GNU Make/GCC, Keil5 ARMCLANG.

## Global Constraints

- `ch_rx` and `ch_lx` are positive when pushed up.
- `ch_ry` and `ch_ly` are positive when pushed left.
- Right-stick left remains positive chassis yaw and a physical left turn.
- Do not change low-pass filtering, dead-zone logic, differential kinematics, motor installation polarities, RS485, or Pr3.04 handling.
- Preserve all unrelated existing working-tree changes.

---

### Task 1: Normalize RC channel polarity at the receive boundary

**Files:**
- Modify: `BSP/Inc/bsp_rc.h:27-32,75-83`
- Modify: `BSP/Src/bsp_rc.c:103-108`
- Modify: `App/Inc/app_config.h:126-131`
- Test: one-off PowerShell source-contract check plus root `make`

**Interfaces:**
- Consumes: decoded SBUS values `raw_channels[0..3]` and `BSP_SBUS_MID_VALUE`.
- Produces: `g_rc.ch_ry/ch_rx/ch_lx/ch_ly` with the documented signs; downstream `g_rc_filter` consumes these values unchanged.

- [ ] **Step 1: Run the source-contract test and verify RED**

```powershell
$header = Get-Content -Raw BSP/Inc/bsp_rc.h
$source = Get-Content -Raw BSP/Src/bsp_rc.c
$config = Get-Content -Raw App/Inc/app_config.h

$checks = @(
  @{ Name='RY polarity macro'; Text=$header; Pattern='#define\s+BSP_RC_CH_RY_POLARITY\s+\(-1\)' },
  @{ Name='RX polarity macro'; Text=$header; Pattern='#define\s+BSP_RC_CH_RX_POLARITY\s+\(\s*1\)' },
  @{ Name='LX polarity macro'; Text=$header; Pattern='#define\s+BSP_RC_CH_LX_POLARITY\s+\(\s*1\)' },
  @{ Name='LY polarity macro'; Text=$header; Pattern='#define\s+BSP_RC_CH_LY_POLARITY\s+\(-1\)' },
  @{ Name='RY receive mapping'; Text=$source; Pattern='raw_channels\[0\][\s\S]*BSP_RC_CH_RY_POLARITY' },
  @{ Name='RX receive mapping'; Text=$source; Pattern='raw_channels\[1\][\s\S]*BSP_RC_CH_RX_POLARITY' },
  @{ Name='LX receive mapping'; Text=$source; Pattern='raw_channels\[2\][\s\S]*BSP_RC_CH_LX_POLARITY' },
  @{ Name='LY receive mapping'; Text=$source; Pattern='raw_channels\[3\][\s\S]*BSP_RC_CH_LY_POLARITY' },
  @{ Name='positive chassis mapping'; Text=$config; Pattern='#define\s+APP_CHASSIS_CMD_POLARITY\s+\(1\)' }
)

foreach ($check in $checks) {
  if ($check.Text -notmatch $check.Pattern) {
    throw "Missing contract: $($check.Name)"
  }
}
```

Expected before implementation: FAIL with `Missing contract: RY polarity macro`.

- [ ] **Step 2: Add explicit channel polarity macros and update comments**

Add to `BSP/Inc/bsp_rc.h`:

```c
/** @brief 摇杆通道极性：统一为上正、左正 */
#define BSP_RC_CH_RY_POLARITY  (-1)
#define BSP_RC_CH_RX_POLARITY  ( 1)
#define BSP_RC_CH_LX_POLARITY  ( 1)
#define BSP_RC_CH_LY_POLARITY  (-1)
```

Update `RC_Channels_t` and `RC_Filter_t` direction comments to state that `ch_ry/ch_ly` are left-positive and `ch_rx/ch_lx` are up-positive.

- [ ] **Step 3: Apply each polarity once while publishing `g_rc`**

Replace the four assignments in `bsp_rc_on_frame_received()` with:

```c
g_rc.ch_ry = (int16_t)(((int16_t)raw_channels[0] - (int16_t)BSP_SBUS_MID_VALUE)
                       * BSP_RC_CH_RY_POLARITY);
g_rc.ch_rx = (int16_t)(((int16_t)raw_channels[1] - (int16_t)BSP_SBUS_MID_VALUE)
                       * BSP_RC_CH_RX_POLARITY);
g_rc.ch_lx = (int16_t)(((int16_t)raw_channels[2] - (int16_t)BSP_SBUS_MID_VALUE)
                       * BSP_RC_CH_LX_POLARITY);
g_rc.ch_ly = (int16_t)(((int16_t)raw_channels[3] - (int16_t)BSP_SBUS_MID_VALUE)
                       * BSP_RC_CH_LY_POLARITY);
```

- [ ] **Step 4: Preserve physical steering direction at the APP boundary**

Change `App/Inc/app_config.h` to:

```c
/** @brief 遥控角速度方向极性；RC 层已统一为左正，因此保持正号 */
#define APP_CHASSIS_CMD_POLARITY (1)
```

- [ ] **Step 5: Re-run the source-contract test and verify GREEN**

Run the PowerShell block from Step 1.

Expected: exit code 0 with no missing-contract exception.

- [ ] **Step 6: Build the complete firmware**

Run: `make -B -j4`

Expected: exit code 0 and generation of `build/CtrBoard-H7-AIBOT-DPC.elf`.

- [ ] **Step 7: Review and commit only scoped files**

```powershell
git diff --check -- BSP/Inc/bsp_rc.h BSP/Src/bsp_rc.c App/Inc/app_config.h docs/superpowers/plans/2026-08-07-rc-channel-polarity.md
git add BSP/Inc/bsp_rc.h BSP/Src/bsp_rc.c App/Inc/app_config.h docs/superpowers/plans/2026-08-07-rc-channel-polarity.md
git commit -m "feat: normalize RC joystick channel polarity"
```

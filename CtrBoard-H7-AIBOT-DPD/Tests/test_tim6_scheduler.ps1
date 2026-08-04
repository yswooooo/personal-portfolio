$ErrorActionPreference = 'Stop'

$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'
$mainHeaderPath = Join-Path $PSScriptRoot '..\Core\Inc\main.h'
$timPath = Join-Path $PSScriptRoot '..\Core\Src\tim.c'
$makefilePath = Join-Path $PSScriptRoot '..\Makefile'
$main = Get-Content -Raw $mainPath
$mainHeader = Get-Content -Raw $mainHeaderPath
$tim = Get-Content -Raw $timPath
$makefile = Get-Content -Raw $makefilePath

$checks = [ordered]@{
    'scheduler tick macro' = $mainHeader -match '#define\s+APP_SCHEDULER_TICK_MS\s+1U'
    'steer period macro' = $mainHeader -match '#define\s+APP_STEER_TASK_PERIOD_MS\s+20U'
    'TIM6 interrupt start' = $main -match 'HAL_TIM_Base_Start_IT\s*\(\s*&htim6\s*\)'
    'HAL timer callback is in tim.c' = ($tim -match 'void\s+HAL_TIM_PeriodElapsedCallback\s*\(\s*TIM_HandleTypeDef\s*\*\s*htim\s*\)') -and ($main -notmatch 'void\s+HAL_TIM_PeriodElapsedCallback')
    'TIM6 callback filter' = $tim -match 'htim->Instance\s*==\s*TIM6'
    'TIM6 callback sets task flag directly' = $tim -match 's_steerwheel_chassis_task_20ms_flag\s*=\s*1U'
    'macro-derived period' = $tim -match 'APP_STEER_TASK_PERIOD_MS\s*/\s*APP_SCHEDULER_TICK_MS'
    'static TIM6 software counter' = $tim -match 'static\s+uint32_t\s+s_scheduler_tick_count\s*=\s*0U'
    'global volatile task flag definition' = $main -match '(?m)^volatile\s+uint8_t\s+s_steerwheel_chassis_task_20ms_flag\s*=\s*0U\s*;'
    'extern volatile task flag declaration' = $mainHeader -match 'extern\s+volatile\s+uint8_t\s+s_steerwheel_chassis_task_20ms_flag\s*;'
    'clear flag before three task calls' = $main -match '(?s)if\s*\(\s*s_steerwheel_chassis_task_20ms_flag\s*!=\s*0U\s*\).*?s_steerwheel_chassis_task_20ms_flag\s*=\s*0U\s*;.*?app_rc_channels_check_lost\s*\(\s*&g_rc\s*\)\s*;.*?app_rc_channels_set_emergency_flag\s*\(\s*&g_rc\s*\)\s*;.*?app_steer_chassis_rc_control\s*\(\s*&g_rc_filter\s*\)\s*;'
    'RC lost check only runs in main loop' = ([regex]::Matches($main, 'app_rc_channels_check_lost\s*\(\s*&g_rc\s*\)')).Count -eq 1
    'emergency check only runs in main loop' = ([regex]::Matches($main, 'app_rc_channels_set_emergency_flag\s*\(\s*&g_rc\s*\)')).Count -eq 1
    'steer control only runs in main loop' = ([regex]::Matches($main, 'app_steer_chassis_rc_control\s*\(\s*&g_rc_filter\s*\)')).Count -eq 1
    'Makefile compiles tim.c' = $makefile -match '(?m)^\s*Core/Src/tim\.c\s*\\'
}

$failed = @($checks.GetEnumerator() | Where-Object { -not $_.Value })
if ($failed.Count -ne 0) {
    $failed | ForEach-Object { Write-Error ("FAIL: " + $_.Key) -ErrorAction Continue }
    exit 1
}

$checks.Keys | ForEach-Object { Write-Output ("PASS: " + $_) }

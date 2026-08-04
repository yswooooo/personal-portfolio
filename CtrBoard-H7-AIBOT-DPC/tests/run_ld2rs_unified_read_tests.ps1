$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path ([System.IO.Path]::GetTempPath()) 'ld2rs-unified-read-tests'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$bspExe = Join-Path $buildDir 'test_bsp_rs485_cancel.exe'
$appExe = Join-Path $buildDir 'test_app_ld2rs_read_fsm.exe'

Push-Location $repo
try {
    & gcc `
        -std=c11 -Wall -Wextra -Werror `
        -I'BSP/Inc' -I'tests/fakes' `
        'BSP/Src/bsp_rs485.c' `
        'tests/test_bsp_rs485_cancel.c' `
        -o $bspExe
    if ($LASTEXITCODE -ne 0) {
        throw "BSP cancellation test compilation failed with exit code $LASTEXITCODE"
    }

    & $bspExe
    if ($LASTEXITCODE -ne 0) {
        throw "BSP cancellation test failed with exit code $LASTEXITCODE"
    }

    & gcc `
        -std=c11 -Wall -Wextra -Werror `
        -I'tests/fakes' -I'App/Inc' -I'BSP/Inc' `
        'tests/test_app_ld2rs_read_fsm.c' `
        'App/Src/app_encoder_speed.c' `
        -o $appExe
    if ($LASTEXITCODE -ne 0) {
        throw "APP state-machine test compilation failed with exit code $LASTEXITCODE"
    }

    & $appExe
    if ($LASTEXITCODE -ne 0) {
        throw "APP state-machine test failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

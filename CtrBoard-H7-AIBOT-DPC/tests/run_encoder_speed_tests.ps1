$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path ([System.IO.Path]::GetTempPath()) 'ld2rs-encoder-speed-tests'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$estimatorExe = Join-Path $buildDir 'test_app_encoder_speed.exe'
$dwtExe = Join-Path $buildDir 'test_bsp_dwt.exe'
$rs485Exe = Join-Path $buildDir 'test_bsp_rs485_cancel.exe'
$samplingExe = Join-Path $buildDir 'test_app_ld2rs_encoder_sampling.exe'

Push-Location $repo
try {
    & gcc `
        -std=c11 -Wall -Wextra -Werror `
        -I'App/Inc' `
        'tests/test_app_encoder_speed.c' `
        'App/Src/app_encoder_speed.c' `
        -o $estimatorExe
    if ($LASTEXITCODE -ne 0) {
        throw "Encoder estimator test compilation failed with exit code $LASTEXITCODE"
    }
    & $estimatorExe
    if ($LASTEXITCODE -ne 0) {
        throw "Encoder estimator test failed with exit code $LASTEXITCODE"
    }

    & gcc `
        -std=c11 -Wall -Wextra -Werror `
        -I'BSP/Inc' -I'tests/fakes' `
        'tests/test_bsp_dwt.c' `
        -o $dwtExe
    if ($LASTEXITCODE -ne 0) {
        throw "DWT timestamp test compilation failed with exit code $LASTEXITCODE"
    }
    & $dwtExe
    if ($LASTEXITCODE -ne 0) {
        throw "DWT timestamp test failed with exit code $LASTEXITCODE"
    }

    & gcc `
        -std=c11 -Wall -Wextra -Werror `
        -I'BSP/Inc' -I'tests/fakes' `
        'BSP/Src/bsp_rs485.c' `
        'tests/test_bsp_rs485_cancel.c' `
        -o $rs485Exe
    if ($LASTEXITCODE -ne 0) {
        throw "RS485 publication test compilation failed with exit code $LASTEXITCODE"
    }
    & $rs485Exe
    if ($LASTEXITCODE -ne 0) {
        throw "RS485 publication test failed with exit code $LASTEXITCODE"
    }

    & gcc `
        -std=c11 -Wall -Wextra -Werror `
        -I'tests/fakes' -I'App/Inc' -I'BSP/Inc' `
        'tests/test_app_ld2rs_encoder_sampling.c' `
        'App/Src/app_encoder_speed.c' `
        -o $samplingExe
    if ($LASTEXITCODE -ne 0) {
        throw "Encoder publication test compilation failed with exit code $LASTEXITCODE"
    }
    & $samplingExe
    if ($LASTEXITCODE -ne 0) {
        throw "Encoder publication test failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

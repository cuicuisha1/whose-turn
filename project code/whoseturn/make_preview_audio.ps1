# 只生成本机合成语音预览；正式研究前请试听并冻结同一套录音。
$ErrorActionPreference = 'Stop'
$taskAudioDir = Join-Path $PSScriptRoot 'audio_preview'
New-Item -ItemType Directory -Force -Path $taskAudioDir | Out-Null
$taskPrompts = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'prompts.json') -Raw | ConvertFrom-Json
$taskVoice = New-Object -ComObject SAPI.SpVoice
foreach ($taskProperty in $taskPrompts.PSObject.Properties) {
    $taskStream = New-Object -ComObject SAPI.SpFileStream
    try {
        $taskStream.Open((Join-Path $taskAudioDir ($taskProperty.Name + '.wav')), 3, $false)
        $taskVoice.AudioOutputStream = $taskStream
        [void]$taskVoice.Speak([string]$taskProperty.Value)
    } finally {
        $taskStream.Close()
        [void][Runtime.InteropServices.Marshal]::ReleaseComObject($taskStream)
    }
}
[void][Runtime.InteropServices.Marshal]::ReleaseComObject($taskVoice)
Write-Output ('Generated preview audio: ' + $taskAudioDir)

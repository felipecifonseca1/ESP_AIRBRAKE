# tools/safe_build.ps1 - Token-Optimized PlatformIO Build Wrapper for Windows
param (
    [string]$Command = "pio"
)

# Force the path to be absolute based on current execution directory
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$AbsoluteLogPath = Join-Path -Path $ScriptDir -ChildPath "build.log"

Write-Output "Executing Safe Build: $Command run"
Write-Output "Logging temporarily to: $AbsoluteLogPath"

# Verified absolute path to PlatformIO Core environment
$PioPath = "C:\Users\Fefon\.platformio\penv\Scripts\pio.exe"

if (-not (Test-Path $PioPath)) {
    Write-Output "CRITICAL ERROR: PlatformIO executable not found at $PioPath"
    exit 1
}

# Initialize process descriptor to bypass nested shell text-buffering issues
$ProcessInfo = New-Object System.Diagnostics.ProcessStartInfo
$ProcessInfo.FileName = $PioPath
$ProcessInfo.Arguments = "run"
$ProcessInfo.RedirectStandardOutput = $true
$ProcessInfo.RedirectStandardError = $true
$ProcessInfo.UseShellExecute = $false
$ProcessInfo.CreateNoWindow = $true

# Spawn process and capture kernel IO streams directly
$Process = [System.Diagnostics.Process]::Start($ProcessInfo)
$Stdout = $Process.StandardOutput.ReadToEnd()
$Stderr = $Process.StandardError.ReadToEnd()
$Process.WaitForExit()

$BuildExitCode = $Process.ExitCode

# Serialize and merge streams to file
$CombinedOutput = $Stdout + $Stderr
$CombinedOutput | Out-File -FilePath "$AbsoluteLogPath" -Encoding utf8

# Evaluate build process state
if ($BuildExitCode -ne 0) {
    Write-Output "BUILD FAILED. Extracting critical compilation errors:"
    Write-Output "--------------------------------------------------"
    
    # Enforce microscopic pause to ensure disk write operations conclude
    Start-Sleep -Milliseconds 300
    
    if ((Test-Path -Path "$AbsoluteLogPath") -and ((Get-Item "$AbsoluteLogPath").Length -gt 0)) {
        # Regular expressions capturing explicit GCC compilation faults, missing assets, or linker failures
        $Patterns = @("error:", "fatal error:", "undefined reference", "failed", ":\d+:\d+:", "compilation terminated")
        $ErrorLines = Select-String -Path "$AbsoluteLogPath" -Pattern $Patterns -Context 0,2
        
        if ($null -eq $ErrorLines -or $ErrorLines.Count -eq 0) {
            Write-Output "No explicit pattern matched. Displaying final 25 lines of raw build log:"
            Write-Output "--------------------------------------------------"
            Get-Content -Path "$AbsoluteLogPath" -Tail 25
        } else {
            $ErrorLines
        }
    } else {
        Write-Output "Error: No process data captured. Native execution payload was completely empty."
        Write-Output "Raw Stderr Fallback: $Stderr"
    }
    exit 1
} else {
    Write-Output "BUILD SUCCESS."
    # Flush temporary asset to optimize workspace state
    if (Test-Path -Path "$AbsoluteLogPath") { Remove-Item -Path "$AbsoluteLogPath" -Force }
    exit 0
}
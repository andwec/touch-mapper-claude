# Touch Mapper - Local Windows Setup Script
# Run this once after cloning the repo and downloading Blender 2.78.
#
# Prerequisites:
#   1. Java (any version) - for OSM2World
#   2. Node.js             - for web UI and clip-2d
#   3. Python 3.x          - for the converter pipeline
#   4. Blender 2.78 Windows (https://download.blender.org/release/Blender2.78/blender-2.78c-windows64.zip)
#      → extract and place as converter\blender\  (so converter\blender\blender.exe exists)
#
# Usage:  powershell -ExecutionPolicy Bypass -File setup-local-windows.ps1

Set-StrictMode -Off
$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Write-Host "Project root: $ProjectRoot"

# --- 1. Install npm dependencies (root server) ---
Write-Host ""
Write-Host "=== Installing root npm dependencies (Express server) ==="
Push-Location $ProjectRoot
npm install
Pop-Location

# --- 2. Install web UI npm dependencies ---
Write-Host ""
Write-Host "=== Installing web UI npm dependencies ==="
Push-Location "$ProjectRoot\web"
npm install
Pop-Location

# --- 3. Build web UI ---
Write-Host ""
Write-Host "=== Building web UI ==="
Push-Location "$ProjectRoot\web"
python pre2src.py
node build.js
Pop-Location

# --- 4. Build OSM2World JAR ---
Write-Host ""
Write-Host "=== Building OSM2World JAR ==="
$AntExe = $null
# Look for Ant in the project directory first (downloaded by this setup), then PATH
$localAnt = "$ProjectRoot\apache-ant-1.10.14\bin\ant.bat"
if (Test-Path $localAnt) {
    $AntExe = $localAnt
} else {
    $antCmd = Get-Command ant -ErrorAction SilentlyContinue
    if ($antCmd) { $AntExe = $antCmd.Source }
}

if (-not $AntExe) {
    Write-Host "WARNING: Ant not found. Downloading Apache Ant 1.10.14..."
    $antZip = "$ProjectRoot\apache-ant-bin.zip"
    Invoke-WebRequest -Uri "https://archive.apache.org/dist/ant/binaries/apache-ant-1.10.14-bin.zip" -OutFile $antZip
    Expand-Archive -Path $antZip -DestinationPath $ProjectRoot -Force
    Remove-Item $antZip
    $AntExe = "$ProjectRoot\apache-ant-1.10.14\bin\ant.bat"
}

Push-Location "$ProjectRoot\OSM2World"
& $AntExe clean jar
Pop-Location

if (Test-Path "$ProjectRoot\OSM2World\build\OSM2World.jar") {
    Write-Host "OSM2World.jar built successfully."
} else {
    Write-Error "OSM2World.jar not found after build. Check the Ant output above."
}

# --- 5. Install svgwrite into Blender 2.78 Python ---
Write-Host ""
Write-Host "=== Installing svgwrite for Blender 2.78 ==="
$blenderPython = "$ProjectRoot\converter\blender\2.78\python\bin\python.exe"
if (-not (Test-Path $blenderPython)) {
    # Try alternative naming
    $blenderPython = Get-ChildItem "$ProjectRoot\converter\blender\2.78\python\bin" -Filter "python*.exe" -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
}

if ($blenderPython) {
    $svgwriteTarget = "$ProjectRoot\converter\blender\2.78\python\lib\python3.5\svgwrite"
    New-Item -ItemType Directory -Force -Path $svgwriteTarget | Out-Null

    # Download and install pip if not present
    $pipExe = "$ProjectRoot\converter\blender\2.78\python\Scripts\pip.exe"
    if (-not (Test-Path $pipExe)) {
        $getPipPy = "$env:TEMP\get-pip.py"
        Invoke-WebRequest -Uri "https://bootstrap.pypa.io/pip/3.5/get-pip.py" -OutFile $getPipPy
        & $blenderPython $getPipPy
    }

    $pipExe = Get-ChildItem "$ProjectRoot\converter\blender\2.78\python" -Recurse -Filter "pip*.exe" -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
    if ($pipExe) {
        & $pipExe install --target=$svgwriteTarget svgwrite==1.1.9
        Write-Host "svgwrite installed."
    } else {
        Write-Host "WARNING: pip not found in Blender Python. svgwrite not installed."
        Write-Host "SVG export will be skipped during conversion."
    }
} else {
    Write-Host "WARNING: Blender 2.78 Python not found at: converter\blender\"
    Write-Host "Make sure you extracted Blender 2.78 to converter\blender\ first."
    Write-Host "(The converter will still run but without SVG output.)"
}

Write-Host ""
Write-Host "================================================================"
Write-Host "Setup complete!"
Write-Host ""
Write-Host "To start Touch Mapper locally:"
Write-Host "  node server.js"
Write-Host ""
Write-Host "Then open: http://localhost:3000"
Write-Host ""
Write-Host "NOTES:"
Write-Host "  - Blender 2.78 must be at: converter\blender\blender.exe"
Write-Host "    OR set env var TOUCH_MAPPER_BLENDER_PATH=C:\path\to\blender.exe"
Write-Host "  - Map generation needs internet access (fetches OSM data)"
Write-Host "================================================================"

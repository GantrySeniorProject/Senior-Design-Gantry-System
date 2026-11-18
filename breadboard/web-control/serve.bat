@echo off
rem Serves this folder at http://localhost:8000 and opens the gantry page in Edge.
rem Needed for YouTube E-stop sounds: YouTube won't play videos on a page opened
rem as a file (error 153). Web Serial works the same on localhost.
rem Close this window to stop the server.
cd /d "%~dp0"
set "PY=C:\Espressif\tools\python\v6.0\venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"
start "" /b cmd /c "timeout /t 1 /nobreak >nul & start "" msedge http://localhost:8000/index.html"
echo Gantry page: http://localhost:8000/index.html   (close this window to stop)
"%PY%" -m http.server 8000 --bind 127.0.0.1

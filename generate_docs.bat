@echo off
REM Add Doxygen to PATH if needed
set PATH=C:\Program Files\doxygen\bin;C:\Program Files (x86)\doxygen\bin;%PATH%

REM Check if Doxygen is available
where doxygen >nul 2>&1
if errorlevel 1 (
    echo Doxygen not found in PATH. Please install Doxygen from https://www.doxygen.nl/
    echo.
    echo On Windows 11, you can use:
    echo   winget install -e --id Doxygen.Doxygen
    echo.
    pause
    exit /b 1
)

echo Generating documentation with UML diagrams...
doxygen Doxyfile

if errorlevel 0 (
    echo.
    echo Documentation generated successfully!
    echo Open doxygen\html\index.html to view
    pause
) else (
    echo.
    echo Failed to generate documentation
    pause
    exit /b 1
)

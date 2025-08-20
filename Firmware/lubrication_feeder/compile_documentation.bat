@echo off
echo Compiling LaTeX documentation to PDF...
echo.

REM Check if pdflatex is available
pdflatex --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: pdflatex not found. Please install a LaTeX distribution like MiKTeX or TeX Live.
    echo.
    echo Download links:
    echo - MiKTeX: https://miktex.org/download
    echo - TeX Live: https://www.tug.org/texlive/
    pause
    exit /b 1
)

echo Found pdflatex. Compiling document...
echo.

REM Compile the LaTeX document (run twice for proper cross-references)
pdflatex -interaction=nonstopmode lubrication_feeder_documentation.tex
if %errorlevel% neq 0 (
    echo Error during first compilation pass.
    pause
    exit /b 1
)

echo First pass completed. Running second pass for cross-references...
pdflatex -interaction=nonstopmode lubrication_feeder_documentation.tex
if %errorlevel% neq 0 (
    echo Error during second compilation pass.
    pause
    exit /b 1
)

echo.
echo Compilation successful!
echo Output file: lubrication_feeder_documentation.pdf
echo.

REM Clean up auxiliary files
echo Cleaning up auxiliary files...
del *.aux *.log *.toc *.out *.fls *.fdb_latexmk 2>nul

echo.
echo Done! You can now open lubrication_feeder_documentation.pdf
pause
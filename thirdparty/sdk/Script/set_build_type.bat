@echo off

::…Ë÷√±‡“Î¿‡–Õ
set Configuration=%1
if "%Configuration%"=="" set Configuration=Release
if "%Configuration%"=="Debug" (
set LIBRARY_POSTFIX=d
set Q_Configuration=debug
) else (
set Q_Configuration=release
)

goto :eof

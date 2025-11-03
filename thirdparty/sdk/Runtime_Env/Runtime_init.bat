if "%1" == "" Goto install
if "%1" == "--install" Goto install
if "%1" == "--uninstall" Goto uninstall
echo invalid parameters
Goto :finish

:uninstall
echo.&echo 正在卸载Sdk运行环境！
wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" get VariableValue|more  +1>%tmp%\tmp
set /p CURRENT_PATH=<%tmp%\tmp
del %tmp%\tmp
set OldGenericActiveRuntimeRoot=%GENERIC_ACTIVE_ENVIRONMENT_ROOT%
if not "%OldGenericActiveRuntimeRoot%"=="" (
call set "CURRENT_PATH=%%CURRENT_PATH:%OldGenericActiveRuntimeRoot%\components;=%%"
call set "SYSTEM_PATH=%%CURRENT_PATH:%OldGenericActiveRuntimeRoot%;=%%"
) else (
set SYSTEM_PATH=%CURRENT_PATH%
)
wmic ENVIRONMENT where "name='GENERIC_RUNTIME_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT where "name='GENERIC_ACTIVE_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" set VariableValue="%SYSTEM_PATH%"
echo.&echo Sdk运行环境卸载完成！
Goto :finish

:install
echo.&echo 正在安装Sdk运行环境！
set OSName=windows
set Architecture=%2
set Toolchain=%3
set Platform=%4
set GenericRuntimeRoot=%~dp0
if "%Architecture%"=="" set Architecture=x86_64
if "%Platform%"=="" (
if "%PROCESSOR_ARCHITECTURE%"=="x86" set Platform=x86
if "%PROCESSOR_ARCHITECTURE%"=="AMD64" set Platform=x64
)
if "%Toolchain%"=="" set Toolchain=v141

wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" get VariableValue|more  +1>%tmp%\tmp
set /p CURRENT_PATH=<%tmp%\tmp
del %tmp%\tmp
set OldGenericActiveRuntimeRoot=%GENERIC_ACTIVE_ENVIRONMENT_ROOT%
if not "%OldGenericActiveRuntimeRoot%"=="" (
call set "CURRENT_PATH=%%CURRENT_PATH:%OldGenericActiveRuntimeRoot%\components;=%%"
call set "SYSTEM_PATH=%%CURRENT_PATH:%OldGenericActiveRuntimeRoot%;=%%"
) else (
set SYSTEM_PATH=%CURRENT_PATH%
)
wmic ENVIRONMENT where "name='GENERIC_RUNTIME_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT where "name='GENERIC_ACTIVE_ENVIRONMENT_ROOT'" delete
set GenericActiveRuntimeRoot=%GenericRuntimeRoot:~0,-1%\%OSName%\%Architecture%\%Platform%\%Toolchain%
wmic ENVIRONMENT create name="GENERIC_RUNTIME_ENVIRONMENT_ROOT",username="%ComputerName%\\%UserName%",VariableValue="%GenericRuntimeRoot:~0,-1%"
wmic ENVIRONMENT create name="GENERIC_ACTIVE_ENVIRONMENT_ROOT",username="%ComputerName%\\%UserName%",VariableValue="%GenericActiveRuntimeRoot%"
wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" set VariableValue="%GenericActiveRuntimeRoot%;%GenericActiveRuntimeRoot%\components;%SYSTEM_PATH%"
echo.&echo Sdk运行环境安装完成！
Goto :finish

:finish 
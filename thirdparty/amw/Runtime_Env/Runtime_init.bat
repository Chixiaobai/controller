if "%GENERIC_RUNTIME_ENVIRONMENT_ROOT%"=="" (
    echo Error: you should install sdk runtime environment first. 
    pause
    exit /B 
)

if "%1" == "" Goto install
if "%1" == "--install" Goto install
if "%1" == "--uninstall" Goto uninstall
echo invalid parameters
Goto :finish

:uninstall
echo.&echo 正在卸载Amw运行环境！
wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" get VariableValue|more  +1>%tmp%\tmp
set /p CURRENT_PATH=<%tmp%\tmp
del %tmp%\tmp
set OldAmwActiveRuntimeRoot=%AMW_ACTIVE_ENVIRONMENT_ROOT%
if not "%OldAmwActiveRuntimeRoot%"=="" (
    call set "CURRENT_PATH=%%CURRENT_PATH:%OldAmwActiveRuntimeRoot%\components;=%%"
    call set "SYSTEM_PATH=%%CURRENT_PATH:%OldAmwActiveRuntimeRoot%;=%%"
    if not "%LD_COMPONENTS_PATH%"=="" (
        call set "SYSTEM_COMPONENTS_PATH=%%LD_COMPONENTS_PATH:%OldAmwActiveRuntimeRoot%\components;=%%"
    )
) else (
    set SYSTEM_PATH=%CURRENT_PATH%
    set SYSTEM_COMPONENTS_PATH=%LD_COMPONENTS_PATH%
)
wmic ENVIRONMENT where "name='AMW_RUNTIME_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT where "name='AMW_ACTIVE_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT where "name='GENERATE_BASE_PATH'" delete
wmic ENVIRONMENT where "name='AMW_SERVICE_CENTER_URL'" delete
wmic ENVIRONMENT where "name='AMW_IDF_PATH'" delete
wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" set VariableValue="%SYSTEM_PATH%"
echo.&echo Amw运行环境卸载完成！
Goto :finish

:install
echo.&echo 正在安装Amw运行环境！
set OSName=windows
set Architecture=%2
set Toolchain=%3
set Platform=%4
set AmwRuntimeRoot=%~dp0
if "%Architecture%"=="" set Architecture=x86_64
if "%Platform%"=="" (
if "%PROCESSOR_ARCHITECTURE%"=="x86" set Platform=x86
if "%PROCESSOR_ARCHITECTURE%"=="AMD64" set Platform=x64
)
if "%Toolchain%"=="" set Toolchain=v141

wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" get VariableValue|more  +1>%tmp%\tmp
set /p CURRENT_PATH=<%tmp%\tmp
del %tmp%\tmp
set OldAmwActiveRuntimeRoot=%AMW_ACTIVE_ENVIRONMENT_ROOT%
if not "%OldAmwActiveRuntimeRoot%"=="" (
    call set "CURRENT_PATH=%%CURRENT_PATH:%OldAmwActiveRuntimeRoot%\components;=%%"
    call set "SYSTEM_PATH=%%CURRENT_PATH:%OldAmwActiveRuntimeRoot%;=%%"
    if not "%LD_COMPONENTS_PATH%"=="" (
        call set "SYSTEM_COMPONENTS_PATH=%%LD_COMPONENTS_PATH:%OldAmwActiveRuntimeRoot%\components;=%%"
    )
) else (
    set SYSTEM_PATH=%CURRENT_PATH%
    set SYSTEM_COMPONENTS_PATH=%LD_COMPONENTS_PATH%
)
wmic ENVIRONMENT where "name='AMW_RUNTIME_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT where "name='AMW_ACTIVE_ENVIRONMENT_ROOT'" delete
set AmwActiveRuntimeRoot=%AmwRuntimeRoot:~0,-1%\%OSName%\%Architecture%\%Platform%\%Toolchain%
wmic ENVIRONMENT create name="AMW_RUNTIME_ENVIRONMENT_ROOT",username="%ComputerName%\\%UserName%",VariableValue="%AmwRuntimeRoot:~0,-1%"
wmic ENVIRONMENT create name="AMW_ACTIVE_ENVIRONMENT_ROOT",username="%ComputerName%\\%UserName%",VariableValue="%AmwActiveRuntimeRoot%"
wmic ENVIRONMENT where "name='PATH' and username='%ComputerName%\\%UserName%'" set VariableValue="%AmwActiveRuntimeRoot%;%AmwActiveRuntimeRoot%\components;%SYSTEM_PATH%"
if not "%SYSTEM_COMPONENTS_PATH%"=="" (
    wmic ENVIRONMENT where "name='LD_COMPONENTS_PATH' and username='%ComputerName%\\%UserName%'" set VariableValue="%AmwActiveRuntimeRoot%\components;%SYSTEM_COMPONENTS_PATH%"
) else (
    wmic ENVIRONMENT create name="LD_COMPONENTS_PATH",username="%ComputerName%\\%UserName%",VariableValue="%AmwActiveRuntimeRoot%\components;"
)
wmic ENVIRONMENT create name="GENERATE_BASE_PATH",username="%ComputerName%\\%UserName%",VariableValue="%AMW_RUNTIME_ENVIRONMENT_ROOT%\\generate"
wmic ENVIRONMENT create name="AMW_SERVICE_CENTER_URL",username="%ComputerName%\\%UserName%",VariableValue="tcp://127.0.0.1:15000"
wmic ENVIRONMENT create name="AMW_IDF_PATH",username="%ComputerName%\\%UserName%",VariableValue="%AMW_RUNTIME_ENVIRONMENT_ROOT%/amw_msgs"
echo.&echo Amw运行环境安装完成！
Goto :finish

:finish 

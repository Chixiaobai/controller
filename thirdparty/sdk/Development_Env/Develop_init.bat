if "%1" == "" Goto install
if "%1" == "--install" Goto install
if "%1" == "--uninstall" Goto uninstall
echo invalid parameters
Goto :finish

:uninstall
echo.&echo 正在卸载Sdk开发环境！
wmic ENVIRONMENT where "name='OS_NAME'" delete
wmic ENVIRONMENT where "name='PLATFORM'" delete
wmic ENVIRONMENT where "name='ARCHITECTURE'" delete
wmic ENVIRONMENT where "name='GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT'" delete
echo.&echo Sdk开发环境卸载完成！
Goto :finish

:install
echo.&echo 正在安装Sdk开发环境！
set OSName=windows
set Architecture=%2
set Toolchain=%3
set Platform=%4
set GenericDevelopmentRoot=%~dp0
if "%Architecture%"=="" set Architecture=x86_64
if "%Platform%"=="" (
if "%PROCESSOR_ARCHITECTURE%"=="x86" set Platform=x86
if "%PROCESSOR_ARCHITECTURE%"=="AMD64" set Platform=x64
)
if "%Toolchain%"=="" set Toolchain=v141
wmic ENVIRONMENT where "name='OS_NAME'" delete
wmic ENVIRONMENT where "name='PLATFORM'" delete
wmic ENVIRONMENT where "name='ARCHITECTURE'" delete
wmic ENVIRONMENT where "name='GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT create name="OS_NAME",username="%ComputerName%\\%UserName%",VariableValue="%OSName%"
wmic ENVIRONMENT create name="PLATFORM",username="%ComputerName%\\%UserName%",VariableValue="%Platform%"
wmic ENVIRONMENT create name="ARCHITECTURE",username="%ComputerName%\\%UserName%",VariableValue="%Architecture%"
wmic ENVIRONMENT create name="GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT",username="%ComputerName%\\%UserName%",VariableValue="%GenericDevelopmentRoot:~0,-1%"
echo.&echo Sdk开发环境安装完成！
Goto :finish

:finish
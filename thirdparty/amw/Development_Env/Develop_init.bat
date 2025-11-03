if "%GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT%"=="" (
    echo Error: you should install sdk development environment first. 
    pause
    exit /B 
)

if "%1" == "" Goto install
if "%1" == "--install" Goto install
if "%1" == "--uninstall" Goto uninstall
echo Error: invalid parameters
Goto :finish

:uninstall
echo.&echo 正在卸载Amw开发环境！
wmic ENVIRONMENT where "name='AMW_DEVELOPMENT_ENVIRONMENT_ROOT'" delete
echo.&echo Amw开发环境卸载完成！
Goto :finish

:install
echo.&echo 正在安装Amw开发环境！
set AmwDevelopmentRoot=%~dp0
wmic ENVIRONMENT where "name='AMW_DEVELOPMENT_ENVIRONMENT_ROOT'" delete
wmic ENVIRONMENT create name="AMW_DEVELOPMENT_ENVIRONMENT_ROOT",username="%ComputerName%\\%UserName%",VariableValue="%AmwDevelopmentRoot:~0,-1%"
echo.&echo Amw开发环境安装完成！
Goto :finish

:finish

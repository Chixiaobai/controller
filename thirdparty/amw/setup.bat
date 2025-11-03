@echo off&pushd %~dp0 &title Amw软件包设置
:menu
echo.&echo 以下运行环境只能多选一
echo.&echo 1、安装64位Amw软件包(Visual Studio 2017)
echo.&echo 2、安装32位Amw软件包(Visual Studio 2017)
echo.&echo 3、卸载当前Amw软件包
echo.&echo 4、退出
echo.&echo.

set /p a=输入数字回车：
IF NOT "%a%"=="" SET a=%a:~0,1%

if "%a%"=="1" Goto install_v141_x64
if "%a%"=="2" Goto install_v141_x86
if "%a%"=="3" Goto uninstall
if "%a%"=="4" Goto finish
echo.&echo 输入无效，请重新输入！ 
PAUSE >NUL && CLS && GOTO Menu

:uninstall
call "%~dp0/Runtime_Env/Runtime_init.bat" --uninstall
pause
call "%~dp0/Development_Env/Develop_init.bat" --uninstall
cls && echo.&echo Amw软件包卸载完成，任意键返回！&&pause >nul && cls && goto menu

:install_v141_x64
call "%~dp0/Development_Env/Develop_init.bat" --install x86_64 v141 x64
pause
call "%~dp0/Runtime_Env/Runtime_init.bat" --install x86_64 v141 x64
cls && echo.&echo Amw软件包安装完成，任意键返回！&&pause >nul && cls && goto menu

:install_v141_x86
call "%~dp0/Development_Env/Develop_init.bat" --install x86_64 v141 x86
pause
call "%~dp0/Runtime_Env/Runtime_init.bat" --install x86_64 v141 x86
cls && echo.&echo Amw软件包安装完成，任意键返回！&&pause >nul && cls && goto menu

:finish
echo.


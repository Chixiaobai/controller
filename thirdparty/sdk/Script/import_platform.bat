@echo off

::判断在当前环境下该脚本是否是第一次执行
if "%PlatformInited%"=="" (
    echo Platform is initing!
) else (
    goto :eof
)

::初始配置为编译Release版本
call %~dp0\set_build_type.bat Release

set PlatformIniFile=%1

::从给定的ini配置文件中加载系统配置
if "%PlatformIniFile%"=="" (
    echo Default configuration will be used!
) else (
    for /f "delims=" %%a in ('call %~dp0\initool.cmd /s general /i Architecture %PlatformIniFile%') do (
        set Architecture=%%a
    )

    for /f "delims=" %%a in ('call %~dp0\initool.cmd /s general /i Platform %PlatformIniFile%') do (
        set Platform=%%a
    )

    for /f "delims=" %%a in ('call %~dp0\initool.cmd /s general /i ProcessorNum %PlatformIniFile%') do (
        set ProcessorNum=%%a
    )

    for /f "delims=" %%a in ('call %~dp0\initool.cmd /s general /i AdditionCMakeArg %PlatformIniFile%') do (
        set AdditionCMakeArg=%%a
    )

    for /f "delims=" %%a in ('call %~dp0\initool.cmd /s windows /i Toolchain %PlatformIniFile%') do (
        set Toolchain=%%a
    )

    for /f "delims=" %%a in ('call %~dp0\initool.cmd /s windows /i CompilePrefix %PlatformIniFile%') do (
        set CompilePrefix=%%a
    )
)

if "%Architecture%"=="" set Architecture=x86_64
if "%Platform%"=="" (
if "%PROCESSOR_ARCHITECTURE%"=="x86" set Platform=x86
if "%PROCESSOR_ARCHITECTURE%"=="AMD64" set Platform=x64
)
if "%Toolchain%"=="" set Toolchain=v141
if "%ProcessorNum%"=="" set ProcessorNum=%NUMBER_OF_PROCESSORS%
if not "%NUMBER_OF_COMPILE%"=="" (
    set ProcessorNum=%NUMBER_OF_COMPILE%
)
::根据系统配置初始化VS环境
if "%VISUAL_STUDIO_ROOT%"=="" (
  echo "VISUAL_STUDIO_ROOT环境变量未定义，将默认使用Visual Studio 2017企业版"
  set "VISUAL_STUDIO_ROOT=C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise"
)
echo "当前使用的Visual Studio为%VISUAL_STUDIO_ROOT%"

if "%Platform%"=="x64" (
call "%VISUAL_STUDIO_ROOT%\VC\Auxiliary\Build\vcvars64.bat"
) else (
call "%VISUAL_STUDIO_ROOT%\VC\Auxiliary\Build\vcvars32.bat"
)

::添加工具目录
set PATH=%~dp0tool\;%PATH%

::设置VS多核编译
if "%NOT_USE_JOM%"=="y" (
  set MAKE_TOOL=nmake.exe
  echo "当前make程序MAKE_TOOL被指定为nmake.exe"
) else (
  if exist %~dp0tool\jom.exe (
    set MAKE_TOOL=jom -j%ProcessorNum%
    echo "当前make程序MAKE_TOOL被指定为jom.exe"
  ) else (
    set MAKE_TOOL=nmake.exe
    echo "当前make程序MAKE_TOOL被指定为nmake.exe"
  )
)

::标记该脚本已被执行
set PlatformInited=1

goto :eof

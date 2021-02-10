@ECHO off
TITLE Update AwayJS Dev Enviroment
ECHO [32m Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build if required[0m
PAUSE

IF NOT [%1]==[] (
    set branch=%~1
    ECHO SET %~1
) ELSE (
    set branch=dev
)

ECHO Current branch: %branch%
PAUSE
cd..
cd..
cd @awayjs

call :process_module awayjs core || GOTO handle_fail

call :process_module awayjs stage || GOTO handle_fail

call :process_module awayjs view || GOTO handle_fail

call :process_module awayjs renderer || GOTO handle_fail

call :process_module awayjs graphics || GOTO handle_fail

call :process_module awayjs materials || GOTO handle_fail

call :process_module awayjs scene || GOTO handle_fail

cd..
cd @awayfl

call :process_module awayfl swf-loader || GOTO handle_fail

call :process_module awayfl avm1 || GOTO handle_fail

call :process_module awayfl avm2 || GOTO handle_fail

call :process_module awayfl playerglobal || GOTO handle_fail

cd..

call :process_module awayfl awayfl-player || GOTO handle_fail

PAUSE

EXIT /b

:process_module
ECHO [32m Pull and build @%1/%2[0m
cd %2
for /f "tokens=* USEBACKQ" %%g in (`git rev-parse --abbrev-ref HEAD`) do (set "modulebranch=%%g")

IF NOT %modulebranch%==%branch% (
    call git pull || EXIT /b 1
    call git checkout %branch% || ECHO Branch %branch% not exist, will used a %modulebranch%
    call npm run tsc:build
) ELSE (
    call git pull | findstr /C:"Already up to date." || call npm run tsc:build
)
cd..

EXIT /b

:handle_fail
EXIT /b 1

:end
EXIT /b
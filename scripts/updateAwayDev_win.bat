@ECHO off
TITLE Update AwayJS Dev Enviroment
ECHO [32m Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build if required[0m
PAUSE

for /f "tokens=* USEBACKQ" %%g in (`git rev-parse --abbrev-ref HEAD`) do (set "branch=%%g")

ECHO Current branch: %branch%

cd..
cd..
cd @awayjs

call :process_module awayjs core

call :process_module awayjs stage

call :process_module awayjs view

call :process_module awayjs renderer

call :process_module awayjs graphics

call :process_module awayjs materials

call :process_module awayjs scene

cd..
cd @awayfl

call :process_module awayfl swf-loader

call :process_module awayfl avm1

call :process_module awayfl avm2

call :process_module awayfl playerglobal

cd..

call :process_module awayfl awayfl-player

PAUSE

EXIT /b

:process_module
ECHO [32m Pull and build @%1/%2[0m
cd %2
for /f "tokens=* USEBACKQ" %%g in (`git rev-parse --abbrev-ref HEAD`) do (set "modulebranch=%%g")
IF NOT %modulebranch%==%branch% (
    git checkout %branch% | findstr "Your branch is up to date"
    IF %errorlevel%==1 call git pull
    call npm run tsc:build
) ELSE (
    git pull | findstr /C:"Already up to date."
    IF %errorlevel%==1 call npm run tsc:build
)
cd..

EXIT /b
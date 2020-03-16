@ECHO off
TITLE Update AwayJS Dev Enviroment
ECHO [32m Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build if required[0m
PAUSE

for /f "tokens=* USEBACKQ" %%g in (`git rev-parse --abbrev-ref HEAD`) do (set "branch=%%g")

ECHO Current branch: %branch%

cd..
cd..
cd @awayjs

ECHO [32m Pull and build @awayjs/core[0m
cd core
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayjs/stage[0m
cd stage
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayjs/view[0m
cd view
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayjs/renderer[0m
cd renderer
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayjs/graphics[0m
cd graphics
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayjs/materials[0m
cd materials
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayjs/scene[0m
cd scene
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

cd..
cd @awayfl

ECHO [32m Pull and build @awayfl/swf-loader[0m
cd swf-loader
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayfl/avm1[0m
cd avm1
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayfl/avm2[0m
cd avm2
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

ECHO [32m Pull and build @awayfl/playerglobal[0m
cd playerglobal
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build
cd..

cd..

ECHO [32m Pull and build awayfl-player[0m
cd awayfl-player
git checkout %branch%
git pull | findstr /C:"Already up to date."
IF %errorlevel%==1 call npm run tsc:build

PAUSE

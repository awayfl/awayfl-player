TITLE Update AwayJS Dev Enviroment
ECHO Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build
PAUSE
cd..
cd..
cd @awayjs

ECHO Pull and build core
cd core
git pull
call npm run tsc:build
cd..

cd stage
git pull
call npm run tsc:build
cd..

cd renderer
git pull
call npm run tsc:build
cd..

cd graphics
git pull
call npm run tsc:build
cd..

cd materials
git pull
call npm run tsc:build
cd..

cd scene
git pull
call npm run tsc:build
cd..

cd view
git pull
call npm run tsc:build
cd..

cd..
cd @awayfl

cd swf-loader
git pull
call npm run tsc:build
cd..

cd avm1
git pull
call npm run tsc:build
cd..

cd avm2
git pull
call npm run tsc:build
cd..

cd playerglobal
git pull
call npm run tsc:build
cd..

cd..
cd awayfl-player
git pull
call npm run tsc:build

PAUSE

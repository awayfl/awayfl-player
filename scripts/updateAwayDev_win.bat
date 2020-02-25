TITLE Update AwayJS Dev Enviroment
ECHO Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build
PAUSE
cd..
cd..
cd @awayjs

cd core
git pull
npm run tsc:build
cd..

cd stage
git pull
npm run tsc:build
cd..

cd view
git pull
npm run tsc:build
cd..

cd renderer
git pull
npm run tsc:build
cd..

cd graphics
git pull
npm run tsc:build
cd..

cd materials
git pull
npm run tsc:build
cd..

cd scene
git pull
npm run tsc:build
cd..

cd..
cd @awayfl

cd swf-loader
git pull
npm run tsc:build
cd..

cd avm1
git pull
npm run tsc:build
cd..

cd avm2
git pull
npm run tsc:build
cd..

cd playerglobal
git pull
npm run tsc:build
cd..

cd..
cd awayfl-player
git pull
npm run tsc:build

PAUSE

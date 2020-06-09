TITLE Init AwayFL Dev Enviroment
echo [32m Clones and links all AwayFL modules into a directory "@awayfl", and all and AwayJS modules into a directory "@awayjs" at the same level as the awayfl-player directory[0m
sleep
cd ..
cd ..

cd @awayjs

echo [32m Cloning "@awayjs" modules from Github[0m


echo [32m Checkout and link "@awayjs/core" module[0m
cd core
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayjs/stage" module[0m
cd stage
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayjs/view" module[0m
cd view
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayjs/renderer" module[0m
cd renderer
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayjs/graphics" module[0m
cd graphics
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayjs/materials" module[0m
cd materials
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayjs/scene" module[0m
cd scene
git pull origin dev
yarn
npm run tsc:build
cd ..

cd ..

echo [32m Cloning "@awayfl" modules from Github[0m
cd @awayfl

echo [32m Checkout and link "@awayfl/swf-loader" module[0m
cd swf-loader
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayfl/avm1" module[0m
cd avm1
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayfl/avm2" module[0m
cd avm2
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "@awayfl/playerglobal" module[0m
cd playerglobal
git pull origin dev
yarn
npm run tsc:build
cd ..

echo [32m Checkout and link "awayfl-player" module[0m
cd ..
cd awayfl-player
git pull origin dev
yarn
npm run tsc:build
yarn

sleep

set -euo pipefail

cd $(dirname "$0")

function updateIfNeeded () {
    set +e
    UP_TO_DATE=`git pull origin $1 2>/dev/null | grep "Already up to date" | wc -l`
    set -e
    if [[ "$UP_TO_DATE" == 0 ]]; then
        yarn
        npm run tsc:build
    fi
}

echo [32m Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build[0m
read -n 1 -s -r -p "Press any key to continue"
cd ..
cd ..

cd @awayjs

echo [32m Cloning "@awayjs" modules from Github[0m

echo [32m Checkout and build "@awayjs/core" module if needed[0m
cd core
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayjs/stage" module if needed[0m
cd stage
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayjs/view" module if needed[0m
cd view
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayjs/renderer" module if needed[0m
cd renderer
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayjs/graphics" module if needed[0m
cd graphics
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayjs/materials" module if needed[0m
cd materials
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayjs/scene" module if needed[0m
cd scene
updateIfNeeded dev
cd ..

cd ..

echo [32m Cloning "@awayfl" modules from Github[0m
cd @awayfl

echo [32m Checkout and build "@awayfl/swf-loader" module if needed[0m
cd swf-loader
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayfl/avm1" module if needed[0m
cd avm1
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayfl/avm2" module if needed[0m
cd avm2
updateIfNeeded dev
cd ..

echo [32m Checkout and build "@awayfl/playerglobal" module if needed[0m
cd playerglobal
updateIfNeeded dev
cd ..

echo [32m Checkout and build "awayfl-player" module if needed[0m
cd ..
cd awayfl-player
updateIfNeeded dev
cd ..

read -n 1 -s -r -p "Press any key to continue . . ."
exit
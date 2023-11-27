set -euo pipefail

cd $(dirname "$0")

function updateIfNeeded () {
    BRANCH=`git rev-parse --abbrev-ref HEAD`

    set +e
    UP_TO_DATE=`git pull origin $1 2>/dev/null | grep "Already up to date"`
    set -e
    if [ -z "$UP_TO_DATE" ]; then
        npm run tsc:build
    else
        echo Already up to date 
    fi
}

echo [32m Pulls all updates into the "@awayjs" and "@awayfl" directories, and runs tsc:build[0m
read -n 1 -s -r -p "Press any key to continue"
echo
echo
cd ..
cd ..

cd @awayjs

echo [32m Pull and build "@awayjs/core"[0m
cd core
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayjs/stage"[0m
cd stage
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayjs/view"[0m
cd view
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayjs/renderer"[0m
cd renderer
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayjs/graphics"[0m
cd graphics
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayjs/materials"[0m
cd materials
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayjs/scene"[0m
cd scene
updateIfNeeded dev
cd ..

cd ..

cd @awayfl

echo [32m Pull and build "@awayfl/swf-loader"[0m
cd swf-loader
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayfl/avm1"[0m
cd avm1
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayfl/avm2"[0m
cd avm2
updateIfNeeded dev
cd ..

echo [32m Pull and build "@awayfl/playerglobal"[0m
cd playerglobal
updateIfNeeded dev
cd ..

echo [32m Pull and build "awayfl-player"[0m
cd ..
cd awayfl-player
updateIfNeeded dev
cd ..

read -n 1 -s -r -p "Press any key to continue . . ."
exit
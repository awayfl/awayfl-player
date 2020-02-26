@ECHO off
TITLE Init AwayFL Dev Enviroment
ECHO [32m Clones and links all AwayFL modules into a directory "@awayfl", and all and AwayJS modules into a directory "@awayjs" at the same level as the awayfl-player directory[0m
PAUSE
cd..
cd..

ECHO [32m Cloning "@awayjs" modules from Github[0m

mkdir @awayjs
cd @awayjs
git clone https://github.com/awayjs/core.git
git clone https://github.com/awayjs/graphics.git
git clone https://github.com/awayjs/scene.git
git clone https://github.com/awayjs/stage.git
git clone https://github.com/awayjs/renderer.git
git clone https://github.com/awayjs/materials.git
git clone https://github.com/awayjs/view.git

ECHO [32m Checkout and link "@awayjs/core" module[0m
cd core
git checkout dev
call yarn
call yarn link
cd..

ECHO [32m Checkout and link "@awayjs/stage" module[0m
cd stage
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
cd..

ECHO [32m Checkout and link "@awayjs/view" module[0m
cd view
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
cd..

ECHO [32m Checkout and link "@awayjs/renderer" module[0m
cd renderer
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/view
cd..

ECHO [32m Checkout and link "@awayjs/graphics" module[0m
cd graphics
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/renderer
cd..

ECHO [32m Checkout and link "@awayjs/materials" module[0m
cd materials
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/renderer
call yarn link @awayjs/view
cd..

ECHO [32m Checkout and link "@awayjs/scene" module[0m
cd scene
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/view
call yarn link @awayjs/renderer
call yarn link @awayjs/graphics
call yarn link @awayjs/materials
cd..

cd..

ECHO [32m Cloning "@awayfl" modules from Github[0m

mkdir @awayfl
cd @awayfl
git clone https://github.com/awayfl/swf-loader.git
git clone https://github.com/awayfl/avm1.git
git clone https://github.com/awayfl/avm2.git
git clone https://github.com/awayfl/playerglobal.git

ECHO [32m Checkout and link "@awayfl/swf-loader" module[0m
cd swf-loader
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/renderer
call yarn link @awayjs/graphics
call yarn link @awayjs/materials
call yarn link @awayjs/scene
cd..

ECHO [32m Checkout and link "@awayfl/avm1" module[0m
cd avm1
call yarn
call yarn link
call yarn link @awayfl/swf-loader
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/renderer
call yarn link @awayjs/graphics
call yarn link @awayjs/materials
call yarn link @awayjs/scene
call yarn link @awayjs/view
cd..

ECHO [32m Checkout and link "@awayfl/avm2" module[0m
cd avm2
call yarn
call yarn link
call yarn link @awayfl/swf-loader
call yarn link @awayjs/core
call yarn link @awayjs/graphics
call yarn link @awayjs/scene
call yarn link @awayjs/stage
cd..

ECHO [32m Checkout and link "@awayfl/playerglobal" module[0m
cd playerglobal
call yarn
call yarn link
call yarn link @awayfl/swf-loader
call yarn link @awayfl/avm2
call yarn link @awayjs/core
call yarn link @awayjs/graphics
call yarn link @awayjs/materials
call yarn link @awayjs/scene
call yarn link @awayjs/stage
call yarn link @awayjs/view
cd..

ECHO [32m Checkout and link "awayfl-player" module[0m
cd..
cd awayfl-player
call yarn
call yarn link
call yarn link @awayfl/swf-loader
call yarn link @awayfl/avm1
call yarn link @awayfl/avm2
call yarn link @awayfl/playerglobal
call yarn link @awayjs/core
call yarn link @awayjs/scene

PAUSE

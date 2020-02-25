TITLE Init AwayFL Dev Enviroment
cd %~dp0
ECHO Clones and links all AwayFL modules into a directory "@awayfl", and all and AwayJS modules into a directory "@awayjs" at the same level as the awayfl-player directory
PAUSE
cd..
cd..

ECHO Cloning "@awayjs" modules from Github

mkdir @awayjs
cd @awayjs
git clone https://github.com/awayjs/core.git
git clone https://github.com/awayjs/graphics.git
git clone https://github.com/awayjs/scene.git
git clone https://github.com/awayjs/stage.git
git clone https://github.com/awayjs/renderer.git
git clone https://github.com/awayjs/materials.git
git clone https://github.com/awayjs/view.git

ECHO Linking "@awayjs" modules using yarn

cd core
git checkout dev
call yarn
call yarn link
cd..

cd stage
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
cd..

cd view
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
cd..

cd renderer
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/view
cd..

cd graphics
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/renderer
cd..

cd materials
git checkout dev
call yarn
call yarn link
call yarn link @awayjs/core
call yarn link @awayjs/stage
call yarn link @awayjs/renderer
call yarn link @awayjs/view
cd..

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

ECHO Cloning "@awayfl" modules from Github

mkdir @awayfl
cd @awayfl
git clone https://github.com/awayfl/swf-loader.git
git clone https://github.com/awayfl/avm1.git
git clone https://github.com/awayfl/avm2.git
git clone https://github.com/awayfl/playerglobal.git

ECHO Linking "@awayfl" modules using yarn

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

cd avm2
call yarn
call yarn link
call yarn link @awayfl/swf-loader
call yarn link @awayjs/core
call yarn link @awayjs/graphics
call yarn link @awayjs/scene
call yarn link @awayjs/stage
cd..

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

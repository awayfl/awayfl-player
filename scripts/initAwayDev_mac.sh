TITLE Init AwayFL Dev Enviroment
echo [32m Clones and links all AwayFL modules into a directory "@awayfl", and all and AwayJS modules into a directory "@awayjs" at the same level as the awayfl-player directory[0m
sleep
cd ..
cd ..

echo [32m Cloning "@awayjs" modules from Github[0m

mkdir @awayjs
cd @awayjs
git clone https://github.com/awayjs/core.git
git clone https://github.com/awayjs/graphics.git
git clone https://github.com/awayjs/scene.git
git clone https://github.com/awayjs/stage.git
git clone https://github.com/awayjs/renderer.git
git clone https://github.com/awayjs/materials.git
git clone https://github.com/awayjs/view.git

echo [32m Checkout and link "@awayjs/core" module[0m
cd core
git checkout dev
yarn
yarn link
cd ..

echo [32m Checkout and link "@awayjs/stage" module[0m
cd stage
git checkout dev
yarn
yarn link
yarn link @awayjs/core
cd ..

echo [32m Checkout and link "@awayjs/view" module[0m
cd view
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
cd ..

echo [32m Checkout and link "@awayjs/renderer" module[0m
cd renderer
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
yarn link @awayjs/view
cd ..

echo [32m Checkout and link "@awayjs/graphics" module[0m
cd graphics
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
yarn link @awayjs/view
yarn link @awayjs/renderer
cd ..

echo [32m Checkout and link "@awayjs/materials" module[0m
cd materials
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
yarn link @awayjs/renderer
yarn link @awayjs/view
cd ..

echo [32m Checkout and link "@awayjs/scene" module[0m
cd scene
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
yarn link @awayjs/view
yarn link @awayjs/renderer
yarn link @awayjs/graphics
yarn link @awayjs/materials
cd ..

cd ..

echo [32m Cloning "@awayfl" modules from Github[0m

mkdir @awayfl
cd @awayfl
git clone https://github.com/awayfl/swf-loader.git
git clone https://github.com/awayfl/avm1.git
git clone https://github.com/awayfl/avm2.git
git clone https://github.com/awayfl/playerglobal.git

echo [32m Checkout and link "@awayfl/swf-loader" module[0m
cd swf-loader
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/view
yarn link @awayjs/stage
yarn link @awayjs/renderer
yarn link @awayjs/graphics
yarn link @awayjs/materials
yarn link @awayjs/scene
cd ..

echo [32m Checkout and link "@awayfl/avm1" module[0m
cd avm1
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/view
yarn link @awayjs/stage
yarn link @awayjs/renderer
yarn link @awayjs/graphics
yarn link @awayjs/materials
yarn link @awayjs/scene
yarn link @awayfl/swf-loader
cd ..

echo [32m Checkout and link "@awayfl/avm2" module[0m
cd avm2
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/view
yarn link @awayjs/renderer
yarn link @awayjs/graphics
yarn link @awayjs/materials
yarn link @awayjs/scene
yarn link @awayjs/stage
yarn link @awayfl/swf-loader
cd ..

echo [32m Checkout and link "@awayfl/playerglobal" module[0m
cd playerglobal
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
yarn link @awayjs/view
yarn link @awayjs/renderer
yarn link @awayjs/graphics
yarn link @awayjs/materials
yarn link @awayjs/scene
yarn link @awayfl/swf-loader
yarn link @awayfl/avm2
cd ..

echo [32m Checkout and link "awayfl-player" module[0m
cd ..
cd awayfl-player
git checkout dev
yarn
yarn link
yarn link @awayjs/core
yarn link @awayjs/stage
yarn link @awayjs/view
yarn link @awayjs/renderer
yarn link @awayjs/graphics
yarn link @awayjs/materials
yarn link @awayjs/scene
yarn link @awayfl/swf-loader
yarn link @awayfl/avm1
yarn link @awayfl/avm2
yarn link @awayfl/playerglobal

sleep

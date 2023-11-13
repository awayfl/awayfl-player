echo [32m Unlinks all AwayFL modules from "@awayfl", and all and AwayJS modules from "@awayjs" at the same level as the awayfl-player directory[0m
read -n 1 -s -r -p "Press any key to continue"
cd $(dirname "$0")
cd..
cd..

echo [32m unlink "awayfl-player" module[0m
cd awayfl-player
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
yarn unlink @awayjs/view
yarn unlink @awayjs/renderer
yarn unlink @awayjs/graphics
yarn unlink @awayjs/materials
yarn unlink @awayjs/scene
yarn unlink @awayfl/swf-loader
yarn unlink @awayfl/avm1
yarn unlink @awayfl/avm2
yarn unlink @awayfl/playerglobal
cd..

cd @awayfl


echo [32m unlink "@awayfl/playerglobal" module[0m
cd playerglobal
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
yarn unlink @awayjs/view
yarn unlink @awayjs/renderer
yarn unlink @awayjs/graphics
yarn unlink @awayjs/materials
yarn unlink @awayjs/scene
yarn unlink @awayfl/swf-loader
yarn unlink @awayfl/avm2
cd..

echo [32m unlink "@awayfl/avm2" module[0m
cd avm2
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/view
yarn unlink @awayjs/renderer
yarn unlink @awayjs/graphics
yarn unlink @awayjs/materials
yarn unlink @awayjs/scene
yarn unlink @awayjs/stage
yarn unlink @awayfl/swf-loader
cd..

echo [32m unlink "@awayfl/avm1" module[0m
cd avm1
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/view
yarn unlink @awayjs/stage
yarn unlink @awayjs/renderer
yarn unlink @awayjs/graphics
yarn unlink @awayjs/materials
yarn unlink @awayjs/scene
yarn unlink @awayfl/swf-loader
cd..

echo [32m unlink "@awayfl/swf-loader" module[0m
cd swf-loader
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/view
yarn unlink @awayjs/stage
yarn unlink @awayjs/renderer
yarn unlink @awayjs/graphics
yarn unlink @awayjs/materials
yarn unlink @awayjs/scene

cd..

cd..

cd @awayjs


echo [32m unlink "@awayjs/scene" module[0m
cd scene
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
yarn unlink @awayjs/view
yarn unlink @awayjs/renderer
yarn unlink @awayjs/graphics
yarn unlink @awayjs/materials
cd..

echo [32m unlink "@awayjs/materials" module[0m
cd materials
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
yarn unlink @awayjs/renderer
yarn unlink @awayjs/view
cd..

echo [32m unlink "@awayjs/graphics" module[0m
cd graphics
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
yarn unlink @awayjs/view
yarn unlink @awayjs/renderer
cd..

echo [32m unlink "@awayjs/renderer" module[0m
cd renderer
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
yarn unlink @awayjs/view
cd..

echo [32m unlink "@awayjs/view" module[0m
cd view
yarn unlink
yarn unlink @awayjs/core
yarn unlink @awayjs/stage
cd..

echo [32m unlink "@awayjs/stage" module[0m
cd stage
yarn unlink
yarn unlink @awayjs/core
cd..

echo [32m unlink "@awayjs/core" module[0m
cd core
yarn unlink
cd..

read -n 1 -s -r -p "Press any key to continue . . ."
exit

echo [32m Unlinks all AwayFL modules from "@awayfl", and all and AwayJS modules from "@awayjs" at the same level as the awayfl-player directory[0m
read -n 1 -s -r -p "Press any key to continue"
cd $(dirname "$0")
cd..
cd..

echo [32m unlink "awayfl-player" module[0m
cd awayfl-player
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/view
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/graphics
pnpm unlink @awayjs/materials
pnpm unlink @awayjs/scene
pnpm unlink @awayfl/swf-loader
pnpm unlink @awayfl/avm1
pnpm unlink @awayfl/avm2
pnpm unlink @awayfl/playerglobal
cd..

cd @awayfl


echo [32m unlink "@awayfl/playerglobal" module[0m
cd playerglobal
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/view
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/graphics
pnpm unlink @awayjs/materials
pnpm unlink @awayjs/scene
pnpm unlink @awayfl/swf-loader
pnpm unlink @awayfl/avm2
cd..

echo [32m unlink "@awayfl/avm2" module[0m
cd avm2
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/view
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/graphics
pnpm unlink @awayjs/materials
pnpm unlink @awayjs/scene
pnpm unlink @awayjs/stage
pnpm unlink @awayfl/swf-loader
cd..

echo [32m unlink "@awayfl/avm1" module[0m
cd avm1
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/view
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/graphics
pnpm unlink @awayjs/materials
pnpm unlink @awayjs/scene
pnpm unlink @awayfl/swf-loader
cd..

echo [32m unlink "@awayfl/swf-loader" module[0m
cd swf-loader
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/view
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/graphics
pnpm unlink @awayjs/materials
pnpm unlink @awayjs/scene

cd..

cd..

cd @awayjs


echo [32m unlink "@awayjs/scene" module[0m
cd scene
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/view
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/graphics
pnpm unlink @awayjs/materials
cd..

echo [32m unlink "@awayjs/materials" module[0m
cd materials
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/renderer
pnpm unlink @awayjs/view
cd..

echo [32m unlink "@awayjs/graphics" module[0m
cd graphics
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/view
pnpm unlink @awayjs/renderer
cd..

echo [32m unlink "@awayjs/renderer" module[0m
cd renderer
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
pnpm unlink @awayjs/view
cd..

echo [32m unlink "@awayjs/view" module[0m
cd view
pnpm unlink
pnpm unlink @awayjs/core
pnpm unlink @awayjs/stage
cd..

echo [32m unlink "@awayjs/stage" module[0m
cd stage
pnpm unlink
pnpm unlink @awayjs/core
cd..

echo [32m unlink "@awayjs/core" module[0m
cd core
pnpm unlink
cd..

read -n 1 -s -r -p "Press any key to continue . . ."
exit

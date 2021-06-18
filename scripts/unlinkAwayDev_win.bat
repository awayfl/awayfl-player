@ECHO off
TITLE Init AwayFL Dev Enviroment
ECHO [32m Unlinks all AwayFL modules from "@awayfl", and all and AwayJS modules from "@awayjs" at the same level as the awayfl-player directory[0m
PAUSE
cd..
cd..

ECHO [32m unlink "awayfl-player" module[0m
cd awayfl-player
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/view
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/graphics
call yarn unlink @awayjs/materials
call yarn unlink @awayjs/scene
call yarn unlink @awayfl/swf-loader
call yarn unlink @awayfl/avm1
call yarn unlink @awayfl/avm2
call yarn unlink @awayfl/playerglobal
cd..

cd @awayfl


ECHO [32m unlink "@awayfl/playerglobal" module[0m
cd playerglobal
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/view
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/graphics
call yarn unlink @awayjs/materials
call yarn unlink @awayjs/scene
call yarn unlink @awayfl/swf-loader
call yarn unlink @awayfl/avm2
cd..

ECHO [32m unlink "@awayfl/avm2" module[0m
cd avm2
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/view
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/graphics
call yarn unlink @awayjs/materials
call yarn unlink @awayjs/scene
call yarn unlink @awayjs/stage
call yarn unlink @awayfl/swf-loader
cd..

ECHO [32m unlink "@awayfl/avm1" module[0m
cd avm1
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/view
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/graphics
call yarn unlink @awayjs/materials
call yarn unlink @awayjs/scene
call yarn unlink @awayfl/swf-loader
cd..

ECHO [32m unlink "@awayfl/swf-loader" module[0m
cd swf-loader
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/view
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/graphics
call yarn unlink @awayjs/materials
call yarn unlink @awayjs/scene

cd..

cd..

cd @awayjs


ECHO [32m unlink "@awayjs/scene" module[0m
cd scene
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/view
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/graphics
call yarn unlink @awayjs/materials
cd..

ECHO [32m unlink "@awayjs/materials" module[0m
cd materials
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/renderer
call yarn unlink @awayjs/view
cd..

ECHO [32m unlink "@awayjs/graphics" module[0m
cd graphics
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/view
call yarn unlink @awayjs/renderer
cd..

ECHO [32m unlink "@awayjs/renderer" module[0m
cd renderer
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
call yarn unlink @awayjs/view
cd..

ECHO [32m unlink "@awayjs/view" module[0m
cd view
call yarn unlink
call yarn unlink @awayjs/core
call yarn unlink @awayjs/stage
cd..

ECHO [32m unlink "@awayjs/stage" module[0m
cd stage
call yarn unlink
call yarn unlink @awayjs/core
cd..

ECHO [32m unlink "@awayjs/core" module[0m
cd core
call yarn unlink
cd..

PAUSE

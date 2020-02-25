/*
 * Copyright 2014 Mozilla Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
import { BrowserSystemResourcesLoadingService } from "./BrowserSystemResourcesLoadingService";

import { ABCFile, ABCCatalog } from "@awayfl/avm2";
import { release, assert, PromiseWrapper } from "@awayfl/swf-loader";
import { SecurityDomain } from '@awayfl/playerglobal';

export enum AVM2LoadLibrariesFlags {
  Builtin = 1,
  Playerglobal = 2,
  Shell = 4
}

export function createSecurityDomain(
  libraries: AVM2LoadLibrariesFlags
): Promise<SecurityDomain> {
  var result = new PromiseWrapper<SecurityDomain>();
  console.log("createSecurityDomain");
  release || assert(!!(libraries & AVM2LoadLibrariesFlags.Builtin));
  console.log("Load builton.abc file");
  BrowserSystemResourcesLoadingService.getInstance()
    .load("./assets/builtins/builtin.abc", "arraybuffer")
    .then(function(buffer) {
      var sec = new SecurityDomain();
      var env = { url: "builtin.abc", app: sec.system };
      var builtinABC = new ABCFile(env, new Uint8Array(buffer));
      sec.system.loadABC(builtinABC);
      sec.initialize();
      sec.system.executeABC(builtinABC);
      //SWF.leaveTimeline();

      //// If library is shell.abc, then just go ahead and run it now since
      //// it's not worth doing it lazily given that it is so small.
      //if (!!(libraries & AVM2LoadLibrariesFlags.Shell)) {
      //  var shellABC = new Shumway.AVMX.ABCFile(new Uint8Array(buffer));
      //  sec.system.loadAndExecuteABC(shellABC);
      //  result.resolve(sec);
      //  SystemResourcesLoadingService.instance.load(SystemResourceId.ShellAbc).then(function (buffer) {
      //    var shellABC = new Shumway.AVMX.ABCFile(new Uint8Array(buffer));
      //    sec.system.loadAndExecuteABC(shellABC);
      //    result.resolve(sec);
      //  }, result.reject);
      //  return;
      //}

      if (!!(libraries & AVM2LoadLibrariesFlags.Playerglobal)) {
        return Promise.all([
            BrowserSystemResourcesLoadingService.getInstance().load(
              "./assets/builtins/playerglobal.abcs", "arraybuffer"
          ),
          BrowserSystemResourcesLoadingService.getInstance().load(
            "./assets/builtins/playerglobal.json", "json"
          )
        ]).then(function(results) {
          console.log("Load playerglobal.abcs & playerglobal.json");
          var catalog = new ABCCatalog(
            sec.system,
            new Uint8Array(results[0]),
            results[1]
          );
          console.log("add playerglobals as ABCCatalog");
          sec.addCatalog(catalog);

          BrowserSystemResourcesLoadingService.getInstance()
            .load("./assets/builtins/avmplus.abc", "arraybuffer")
            .then(function(buffer) {
              //var sec = new AXSecurityDomain();
              var env = { url: "avmplus.File", app: sec.system };
              var avmPlusABC = new ABCFile(env, new Uint8Array(buffer));
              sec.system.loadABC(avmPlusABC);
              //sec.initialize();
              sec.system.executeABC(avmPlusABC);
              result.resolve(sec);
        }, result.reject);
        }, result.reject);
      }

      result.resolve(sec);
    }, result.reject);
  return result.promise;
}

export class BrowserSystemResourcesLoadingService {
  private static _instance: BrowserSystemResourcesLoadingService;
  public static getInstance() {
    if (BrowserSystemResourcesLoadingService._instance)
      return BrowserSystemResourcesLoadingService._instance;
    return (BrowserSystemResourcesLoadingService._instance = new BrowserSystemResourcesLoadingService());
  }
  public constructor() {}

  public load(url, type): Promise<any> {
    return this._promiseFile(url, type);
  }

  private _promiseFile(path, responseType) {
    return new Promise(function(resolve, reject) {
      //SWF.enterTimeline('Load file', path);
      var xhr = new XMLHttpRequest();
      xhr.open("GET", path);
      xhr.responseType = responseType;
      xhr.onload = function() {
        var response = xhr.response;
        if (response) {
          if (responseType === "json" && xhr.responseType !== "json") {
            // some browsers (e.g. Safari) have no idea what json is
            response = JSON.parse(response);
          }
          resolve(response);
        } else {
          reject("Unable to load " + path + ": " + xhr.statusText);
        }
      };
      xhr.onerror = function() {
        reject("Unable to load: xhr error");
      };
      xhr.send();
    });
  }
}

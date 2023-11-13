# AwayFL Player
Flash Player emulator for executing SWF files (published for FP versions 6 and up) in javascript

## Prerequistes ##
 - git ([installation instructions](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git))
 - Node.js ([installation instructions](https://nodejs.dev/learn/how-to-install-nodejs))

## Installing ##
Either use the template file [here](https://github.com/awayfl/awayfl-template) (recommended) or clone this repo and test your own content by typing the following into a terminal / cmd prompt / shell window:
```shell
git clone https://github.com/awayfl/awayfl-player
cd awayfl-player
npm install
```

## Configuration ##
Open the `awayfl.config.js` file and add some SWF files to `fileconfigs`, including a `rt_title` and `rt_filename` entry (without the .swf extension), eg:

```javascript
fileconfigs: [
    {
        rt_title: "my Awesome Flash Game",
        rt_filename: "my_awesome_flash_game",
    },
],
```

`awayfl.config.js` contains many additional configs (documented inline) that can be applied either global or locally to individual test SWFs.

## Preview ##

To run a preview of your SWFs, start up the webpack development server:

```shell
npm start
```
Once compilation is complete, you can view your SWFs through the links at http://localhost. Updating SWFs or any files in  `/src` will auto-reload the browser. However, any changes to `awayfl.config.js` will require a restart.
var riot = require('riot-compiler');

module.exports = function(source) {
    this.cacheable();
    return "import riot from 'riot';\n" + riot.compile(source);
};


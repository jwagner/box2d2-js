(function(){
var Engine = {};
var $ = jQuery;

Engine.log = function(arg) {
return;
    if(typeof(window.console) != 'undefined') {
        console.log(arg);
    }
}
var log = Engine.log;

Engine.Helper = {};
Engine.Helper.Canvas = function (w, h){
    var c = document.createElement('canvas');
    c.setAttribute('width', w);
    c.setAttribute('height', h);
    return c;
}

Engine.FPSCounter = function(ctx) {
    this.t = new Date().getTime()/1000.0;
    this.n = 0;
    this.fps = 0.0;
    this.draw = function() {
        this.n ++;
        if(this.n == 10) {
            this.n = 0;
            t = new Date().getTime()/1000.0;
            this.fps = Math.round(10.0/(t-this.t));
            this.t = t;
        }
        ctx.fillText('FPS: ' + this.fps, 1, 25);
    }
}

Engine.Helper.fillTextMultiline = function(ctx, text, x, y, h) {
    if(!$.isArray(text)) {
        text = text.split('\n');
    }
    for(var i = 0; i < text.length; i ++){
        ctx.fillText(text[i], x, y);
        y += h;
    }
}

Engine.Worker = function(script) {
    var worker = new Worker(script);
    this.output = null;
    worker.onmessage = function(event) {
        this.output = event.data;
    }
    worker.onerror = function(event) {
        log('Worker Error ' + event.filename + ':' + event.lineno + ' ' + event.message);
    }
    this.tick = function(input) {
        worker.postMessage(input);
        return this.output;
    }
}

Engine.FakeWorker =  function(f) {
    this.tick = f;
}

Engine.Scheduler = function(rate) {
    this.ontick = function(){};
    this.last = new Date().getTime()/1000.0;
    var self = this;
    this.tick = function () {
        var current = new Date().getTime()/1000.0;
        self.ontick(current-self.last);
        self.last = current;
    }
    this.setRate = function(rate) {
        this.rate = rate;
        if(self.interval)
            clearInterval(this.interval)
        if(rate > 0) {
            self.interval = setInterval(this.tick, 1000.0/rate)
        }
    }
    this.setRate(rate);
}

var keyname = {
    32: 'SPACE',
    13: 'ENTER',
    9: 'TAB',
    8: 'BACKSPACE',
    16: 'SHIFT',
    17: 'CTRL',
    18: 'ALT',
    20: 'CAPS_LOCK',
    144: 'NUM_LOCK',
    145: 'SCROLL_LOCK',
    37: 'LEFT',
    38: 'UP',
    39: 'RIGHT',
    40: 'DOWN',
    33: 'PAGE_UP',
    34: 'PAGE_DOWN',
    36: 'HOME',
    35: 'END',
    45: 'INSERT',
    46: 'DELETE',
    27: 'ESCAPE',
    19: 'PAUSE',
    222: "'"
};

Engine.KeyTracker = function(obj) {
    //TODO: focus is broken in chrome
    this.focus = true;
    this.onfocus = function() {}
    this.onblur = function() {}

    var self = this;

    this.reset = function() {
        var code;
        for(code in keyname) {
            this[keyname[code]] = false;
        }
    }

    var keydown = $(document).keydown(function(e){
        self[keyname[e.keyCode]] = true;
        return !self.focus;
    });

    var keyup = $(document).keyup(function(evt){
        self[keyname[evt.keyCode]] = false;
        return !self.focus;
    });

    // keep track of focus
    $(obj).click(function(e) {
        self.focus = true;
        $(obj).addClass('keytracked');
        self.onfocus();
    });

    $(obj).addClass('keytracked');

    $(window).click(function(e) {
        if(e.originalEvent.target != obj) {
            $(obj).removeClass('keytracked');
            self.focus = false;
            self.onblur();
        }
    });
 
}

Engine.MouseTracker = function(obj) {
    this.x = 0;
    this.y = 0;
    this.onclick = function() {}
    var self = this;
    // this does assume that the object does not move
    // to improve performance
    var o = $(obj).offset();
    // todo: add unregistration
    $(obj).click(function(e){
        var x = e.pageX-o.left;
        var y = e.pageY-o.top;
        self.onclick(e, x, y);
    });
    $(obj).mousemove(function(e){
        self.x = e.pageX-o.left;
        self.y = e.pageY-o.top;
    });
}



html5av = document.createElement('video').load != undefined;
Engine.ResourceLoader = function(resources, onready, onerror) {
    this.total = 0;
    this.loaded = 0;
    var self = this;
    this.load = function(type, name, url) {
        var data = document.createElement(type);
        this[name] = data;

        // TODO: proper error handling
        $(data).one('error', onerror);

        var callback = function() {
            self.loaded ++;
            log('ready', url, self.loaded, self.total);
            if(self.loaded == self.total) {
                onready();
            }
        }

        if(type == 'video' || type == 'audio') {
            $(data).one('canplaythrough', callback);
            data.setAttribute('autobuffer', 'autobuffer');
            data.setAttribute('src', url);
            data.load();
        }
        else {
            data.setAttribute('src', url);
            $(data).one('load', callback);
        }
        this.total ++;
    }

    for(var i = 0; i < resources.length; i++){
        this.load(resources[i][0], resources[i][1], resources[i][2]);
    }
}

window.Engine = Engine;
})();

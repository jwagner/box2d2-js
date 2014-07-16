jQuery(function($){

var canvas = document.getElementById('canvas');
var ctx = canvas.getContext('2d');
var c_width = 55.0;
var ppm = canvas.width/c_width;
var c_height = canvas.height/ppm;
ctx.setTransform(ppm, 0, 0, -ppm, 0, canvas.height);

var worldAABB = new b2AABB();
worldAABB.lowerBound.Set(-10000.0, -10000.0);
worldAABB.upperBound.Set(10000.0, 10000.0);
var gravity = new b2Vec2(0.0, -9.8);
var world = new b2World(worldAABB, gravity, true);
window.world = world;

var groundBodyDef = new b2BodyDef();
groundBodyDef.position.Set(c_width/2.0, 3.0);
var groundBody = world.CreateBody(groundBodyDef);
var groundShapeDef = new b2PolygonDef();
groundShapeDef.restitution = 0.0;
groundShapeDef.friction = 0.5;
groundShapeDef.density = 1.0;
groundBody.w = c_width*1.0
groundBody.h = 5.0
groundShapeDef.SetAsBox(groundBody.w, groundBody.h);
groundBody.CreateShape(groundShapeDef);
groundBody.SynchronizeShapes();

var bodies = [groundBody];
var explosionParticles = [];

function spawn(x, y) {
    var bodyDef = new b2BodyDef();
    bodyDef.position.Set(x, y);
    var body = world.CreateBody(bodyDef);
    var shapeDef = new b2PolygonDef();
    shapeDef.SetAsBox(1.0, 1.0);
    body.w = 1.0;
    body.h = 1.0;
    shapeDef.restitution = 0.0;
    shapeDef.density = 2.0;
    shapeDef.friction = 0.9;
    body.CreateShape(shapeDef);
    body.SetMassFromShapes();
    bodies.push(body);
}

function explode(x, y) {
    for(var i = 0; i < 10; i++) {
        var a = Math.PI/5*i;
        var vx = Math.cos(a);
        var vy = Math.sin(a);
        var bodyDef = new b2BodyDef();
        bodyDef.position.Set(x+vx, y+vy);
        bodyDef.isBullet = true;
        var body = world.CreateBody(bodyDef);
        var shapeDef = new b2CircleDef();
        shapeDef.radius = 0.1;
        //var shapeDef = new b2PolygonDef();
        //shapeDef.SetAsBox(1.0, 1.0);
        shapeDef.restitution = 0.0;
        shapeDef.density = 50.0;
        shapeDef.friction = 0.0;
        body.CreateShape(shapeDef);
        body.SetMassFromShapes();
        body.ApplyImpulse({x: vx*500, y:vy*500}, {x:x, y:y});
        body.w = 1.0;
        body.h = 1.0;
        explosionParticles.push(body);
    }
}
$(canvas).click(function (e){
    var o = $(canvas).offset();
    var x = (e.pageX-o.left)/ppm;
    var y = (canvas.height-e.pageY+o.top)/ppm;
    explode(x, y);
//    spawn((e.pageX-o.left)/ppm, (canvas.height-e.pageY+o.top)/ppm);
});
//debugger;

for(var i = 0; i < 20; i ++) {
    spawn(c_width/2, c_height);
}

var frame = 0;
var fps = new Engine.FPSCounter(ctx);
window.setInterval(function() {
    frame ++;
    if(frame%20 == 0) {
//        spawn(c_width/2, c_height);
    }
//    debugger;
    world.Step(1.0/60.0, 10);
    //ctx.clearRect(0.0, 0.0, canvas.width, canvas.height);
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, c_width, c_height);
    ctx.fillStyle = 'black';
    for(var i = 0; i < bodies.length; i++){
        var body = bodies[i];
        var t = body.m_xf;
        ctx.translate(t.position.x, t.position.y)
        ctx.rotate(body.GetAngle());
        ctx.fillRect(-body.w, -body.h, body.w*2, body.h*2);
        ctx.rotate(-body.GetAngle());
        ctx.translate(-t.position.x, -t.position.y)
    }
    ctx.fillStyle = 'red';
    for(var i = 0; i < explosionParticles.length; i++){
        var body = explosionParticles[i];
        var t = body.m_xf;
        ctx.translate(t.position.x, t.position.y)
        ctx.beginPath();
        ctx.arc(0, 0, 0.1, 0, Math.PI*2, true);
        ctx.closePath();
        ctx.fill();
        ctx.translate(-t.position.x, -t.position.y)
    }
    ctx.save();
    ctx.setTransform(1,0,0,1,0,0);
    fps.draw();
    ctx.restore();
}, 1000/30);



});
jQuery.noConflict();

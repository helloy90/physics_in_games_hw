import {createCube} from "./parts/cube.js";
import {createSphere} from "./parts/sphere.js";
import {TaskHandler} from "./taskHandler.js";

new p5((p) => {
  let task = new TaskHandler();
  let physics_delta = 1.0 / 60.0;
  let font;
  let camera;
  p.preload = () => { font = p.loadFont('assets/font.ttf'); };

  p.setup = () => {
    p.createCanvas(1280, 720, p.WEBGL);
    p.textFont(font);
    camera = p.createCamera();

    task.loadPart("Cube", createCube(p));
    task.loadPart("Sphere", createSphere(p));

    task.setPart("Cube");
  };

  p.draw = () => {
    p.ambientLight(255);
    p.background(200);

    task.update(physics_delta);

    drawPlane(p, 500, 0, -20, 0);
    drawGrid(p, 500, 50, 0, -20, 0);

    task.render();

    const camParams = [
      camera.eyeX,
      camera.eyeY,
      camera.eyeZ,
      camera.centerX,
      camera.centerY,
      camera.centerZ,
      0,
      -1,
      0,
    ];
    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
    p.camera();
    task.render2D();

    p.push();
    p.noStroke();
    p.translate(-p.width / 1.6, -p.height / 2.5);
    p.fill(255, 255, 220);
    p.textSize(20);
    p.textAlign(p.LEFT, p.BOTTOM);
    p.text(`Scene: ${task.currentPartName}`, 16, p.height - 40);
    p.text(`Press 1 to switch to Cube, 2 to switch to Sphere`, 16, p.height - 16);
    p.pop();

    p.camera(...camParams);
    p.drawingContext.enable(p.drawingContext.DEPTH_TEST);
  };

  p.keyPressed = () => {
    if (p.key === "1")
      task.setPart("Cube");
    if (p.key === "2")
      task.setPart("Sphere");
    task.keyPressed(p.key);
  };

  p.mousePressed = () => { task.mousePressed(p.mouseX, p.mouseY); };

  p.mouseDragged = () => { task.mouseDragged(p.mouseX, p.mouseY); };

  p.mouseReleased = () => { task.mouseReleased(p.mouseX, p.mouseY); };

  p.mouseWheel = (event) => { task.mouseWheel(event); };

  p.windowResized = () => { p.resizeCanvas(window.innerWidth, window.innerHeight); };
});

function drawPlane(p, size, offsetX, offsetY, offsetZ)
{
  p.push();
  p.translate(offsetX, offsetY, offsetZ);
  p.rotateX(p.HALF_PI);
  p.noStroke();
  p.ambientMaterial(150, 150, 150);
  p.plane(size, size);
  p.pop();
}

function drawGrid(p, size, divs, offsetX, offsetY, offsetZ)
{
  p.push();
  p.translate(offsetX, offsetY, offsetZ);
  p.stroke(0, 0, 0, 100);
  p.strokeWeight(1);
  let step = size / divs;
  for (let i = -size / 2; i <= size / 2; i += step)
  {
    p.line(i, 0, -size / 2, i, 0, size / 2);
    p.line(-size / 2, 0, i, size / 2, 0, i);
  }
  p.pop();
}

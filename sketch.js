import {createPart1} from "./parts/part1.js";
import {createPart2_1} from "./parts/part2.1.js";
import {createPart2_2} from "./parts/part2.2.js";
import {createPart2_3} from "./parts/part2.3.js";
import {createPart3_1} from "./parts/part3.1.js";
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

    task.loadPart("part1", createPart1(p));
    task.loadPart("part2.1", createPart2_1(p));
    task.loadPart("part2.2", createPart2_2(p));
    task.loadPart("part2.3", createPart2_3(p));
    task.loadPart("part3.1", createPart3_1(p));

    task.setPart("part3.1");
  };

  p.draw = () => {
    p.ambientLight(255);
    p.background(200);

    p.orbitControl(2.5, 2.5, 2.5);

    task.update(physics_delta);

    // drawPlane(p, 500, 0, -30, 0);
    drawGrid(p, 500, 50, 0, -30, 0);

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
    p.translate(-p.width / 2, -p.height / 2);
    p.fill(255, 255, 220);
    p.textSize(20);
    p.textAlign(p.LEFT, p.BOTTOM);
    p.text(`Scene: ${task.currentPartName}`, 16, p.height - 40);
    p.text(`Press 1, 2, 3, 4, 5 to switch to needed scene`, 16, p.height - 16);
    p.pop();

    p.camera(...camParams);
    p.drawingContext.enable(p.drawingContext.DEPTH_TEST);
  };

  p.keyPressed = () => {
    if (p.key === "1")
      task.setPart("part1");
    if (p.key === "2")
      task.setPart("part2.1");
    if (p.key === "3")
      task.setPart("part2.2");
    if (p.key === "4")
      task.setPart("part2.3");
    if (p.key === "5")
      task.setPart("part3.1");
    task.keyPressed(p.key);
  };

  p.mousePressed = () => { task.mousePressed(p.mouseX, p.mouseY); };

  p.mouseDragged = () => { task.mouseDragged(p.mouseX, p.mouseY); };

  p.mouseReleased = () => { task.mouseReleased(p.mouseX, p.mouseY); };

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

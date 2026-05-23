import * as mat4 from "../libraries/esm/mat4.js";
import * as vec3 from "../libraries/esm/vec3.js";
import * as vec4 from "../libraries/esm/vec4.js";

const sim_modes = [
  {label : "1) Hitman"},
  {label : "2) PBD"},
  {label : "3) XPBD"},
];

const sim_scenes = [
  {label : "1) Free (with some initial velocity)"},
  {label : "2) Hanged"},
  {label : "3) Stretched"},
];

export function createSphere(p)
{
  let mesh = {
    points : [],
    edges : [],
    tets : [],
    triangles : [],
  };
  let proj = mat4.create();
  let view = mat4.create();
  let camPos = vec3.fromValues(35, 17.5, 35);
  let camTarget = vec3.fromValues(0, 0, 0);
  let camUp = vec3.fromValues(0, -1, 0);

  let camDistance;
  let theta;
  let phi;

  let rotationSensitivity = 0.005;
  let rotatingCamera = false;
  let prevMouseX;
  let prevMouseY;

  let dragOffset = vec3.create();
  let dragPlanePoint = vec3.create();
  let dragPlaneNormal = vec3.create();
  let grabbedVertIdx = -1;

  let pointRadius = 1.5;

  // let stiffness = 0.5;
  // let compliance = 0.001;
  // let restitude = 0.7;
  // let friction = 0.9;

  let subStepsSlider;
  let stiffnessSlider;
  let complianceSlider;
  let restitudeSlider;
  let frictionSlider;

  // let subSteps = 10;
  let stopSim = true;

  let currentMode = 0;
  let currentScene = 0;

  let gravity = vec3.fromValues(0, -9.8, 0);

  return {
    init() { resetImpl(); },

    reset() { resetImpl(); },

    update(dt) {
      setCamMatrix();

      if (stopSim)
      {
        return;
      }

      for (let i = 0; i < mesh.points.length; i++)
      {
        const point = mesh.points[i];
        vec3.copy(point.prevWorldPos, point.currentWorldPos);

        if (point.isPinned || i === grabbedVertIdx)
        {
          continue;
        }
        vec3.scaleAndAdd(point.currentVelocity, point.currentVelocity, gravity, dt);

        vec3.scaleAndAdd(point.currentWorldPos, point.currentWorldPos, point.currentVelocity, dt);
      }

      if (grabbedVertIdx !== -1)
      {
        const grabbedPoint = mesh.points[grabbedVertIdx];
        const ray = getRay(p.mouseX, p.mouseY);

        const newHit =
          intersectRayPlane(ray.origin, ray.direction, dragPlanePoint, dragPlaneNormal);

        if (newHit)
        {
          vec3.add(grabbedPoint.currentWorldPos, newHit, dragOffset);
        }

        if (!grabbedPoint.isPinned)
        {
          grabbedPoint.invMass = 0;
        }
      }

      switch (currentMode)
      {
      case 0:
        hitmanStep(dt);
        break;
      case 1:
        pbdStep(dt);
        break;
      case 2:
        xpbdStep(dt);
        break;
      }

      if (grabbedVertIdx !== -1)
      {
        const grabbedPoint = mesh.points[grabbedVertIdx];

        vec3.copy(grabbedPoint.prevWorldPos, grabbedPoint.currentWorldPos);

        if (!grabbedPoint.isPinned)
        {
          grabbedPoint.invMass = 1;
        }
      }

      floorCollisionCheck();
      selfCollisionCheck();

      for (let i = 0; i < mesh.points.length; i++)
      {
        const point = mesh.points[i];
        if (point.isPinned || i === grabbedVertIdx)
        {
          continue;
        }
        vec3.scaleAndAdd(point.currentVelocity, point.currentWorldPos, point.prevWorldPos, -1);
        vec3.scale(point.currentVelocity, point.currentVelocity, 1 / dt);
      }
    },

    render() { drawTask(); },

    render2D() { drawOverlay(); },

    keyPressed(key) {
      if (key.toLowerCase() === "m")
      {
        currentMode = (currentMode + 1) % sim_modes.length;
        this.reset();
      }
      if (key.toLowerCase() === "s")
      {
        currentScene = (currentScene + 1) % sim_scenes.length;
        this.reset();
      }
      if (key.toLowerCase() === "r")
      {
        this.reset();
      }
      if (key.toLowerCase() === "t")
      {
        stopSim = !stopSim;
      }
    },

    mousePressed(x, y) {
      if (p.mouseButton === p.LEFT && !rotatingCamera)
      {
        let ray = getRay(x, y);
        let picked = pickPoint(ray.origin, ray.direction);
        if (picked.index !== -1)
        {
          grabbedVertIdx = picked.index;

          dragPlanePoint = vec3.clone(picked.point);

          vec3.subtract(dragOffset, mesh.points[picked.index].currentWorldPos, dragPlanePoint);

          vec3.subtract(dragPlaneNormal, camTarget, camPos);
          vec3.normalize(dragPlaneNormal, dragPlaneNormal);
        }
      }
      else if (p.mouseButton === p.RIGHT && grabbedVertIdx === -1)
      {
        rotatingCamera = true;
        prevMouseX = x;
        prevMouseY = y;
      }

      return false;
    },

    mouseDragged(x, y) {
      if (rotatingCamera)
      {
        const dx = x - prevMouseX;
        const dy = y - prevMouseY;

        theta += dx * rotationSensitivity;
        phi += dy * rotationSensitivity;

        const max = p.PI / 2 - 0.01;
        phi = p.constrain(phi, -max, max);

        prevMouseX = x;
        prevMouseY = y;
        updateViewMatrix();
      }
    },

    mouseReleased(x, y) {
      grabbedVertIdx = -1;
      rotatingCamera = false;
    },

    mouseWheel(event) {
      camDistance += event.delta * 0.05;
      camDistance = p.constrain(camDistance, 50, 2000);
      updateViewMatrix();
    },
  };

  function resetImpl()
  {
    mesh = makeSphere();

    p.removeElements();


    const stepsLabel = p.createP("Substeps Slider (1-40)");
    stepsLabel.position(900, 0);
    subStepsSlider = p.createSlider(1, 40, 10, 1);
    subStepsSlider.position(900, 30);
    subStepsSlider.size(200);

    const stiffnessLabel = p.createP("Stiffness Slider (0-2, substep - 0.001) (for PBD only)");
    stiffnessLabel.position(900, 40);

    stiffnessSlider = p.createSlider(0, 2, 0.5, 0.001);
    stiffnessSlider.position(900, 70);
    stiffnessSlider.size(200);

    const complianceLabel = p.createP("Compliance Slider (0-1, substep - 0.0001) (for XPBD only)");
    complianceLabel.position(900, 80);

    complianceSlider = p.createSlider(0, 1, 0.001, 0.0001);
    complianceSlider.position(900, 110);
    complianceSlider.size(200);

    const restitudeLabel = p.createP("Restitude Slider (0-2.5, substep - 0.001)");
    restitudeLabel.position(900, 120);
    restitudeSlider = p.createSlider(0, 2.5, 0.7, 0.001);
    restitudeSlider.position(900, 150);
    restitudeSlider.size(200);

    const frictionLabel = p.createP("Friction Slider (0-2.5, substep - 0.001)");
    frictionLabel.position(900, 160);
    frictionSlider = p.createSlider(0, 2.5, 0.9, 0.001);
    frictionSlider.position(900, 190);
    frictionSlider.size(200);

    switch (currentScene)
    {
    case 0:
      setupFirstScene();
      break;
    case 1:
      setupSecondScene();
      break;
    case 2:
      setupThirdScene();
      break;
    }

    camPos = vec3.fromValues(35, 17.5, 35);
    camTarget = vec3.fromValues(0, 0, 0);
    camUp = vec3.fromValues(0, -1, 0);

    camDistance = vec3.distance(camTarget, camPos);
    const dir = vec3.create();
    vec3.subtract(dir, camTarget, camPos);
    theta = Math.atan2(dir[0], dir[2]);
    phi = Math.asin(-dir[1] / camDistance);

    mat4.perspective(proj, p.radians(60), p.width / p.height, 0.1, 10000);
    mat4.lookAt(view, camPos, camTarget, camUp);

    updateViewMatrix();
    setCamMatrix();
  }

  function setCamMatrix()
  {
    p.perspective(p.radians(60), p.width / p.height, 0.1, 10000);
    p.camera(...camPos, ...camTarget, ...camUp);
  }

  function updateViewMatrix()
  {
    camPos[0] = camTarget[0] + camDistance * Math.sin(theta) * Math.cos(phi);
    camPos[1] = camTarget[1] + camDistance * Math.sin(phi);
    camPos[2] = camTarget[2] + camDistance * Math.cos(theta) * Math.cos(phi);

    mat4.lookAt(view, camPos, camTarget, camUp);
  }

  function setupFirstScene()
  {
    const startSpeed = vec3.fromValues(-5, 0, 10);

    for (const point of mesh.points)
    {
      vec3.add(point.currentVelocity, point.currentVelocity, startSpeed);
    }
  }

  function setupSecondScene()
  {
    const {top, bottom} = getCoordRange(1);
    const height = top - bottom;

    let count = 0;
    const topEdge = bottom + height * 0.8;

    for (const point of mesh.points)
    {
      if (point.currentWorldPos[1] >= topEdge)
      {
        pinPoint(point);
        count++;
      }

      if (count >= 1)
      {
        break;
      }
    }
  }

  function setupThirdScene()
  {
    for (const point of mesh.points)
    {
      vec3.scale(point.currentWorldPos, point.currentWorldPos, 1.2);
    }

    const {top, bottom} = getCoordRange(0);
    const span = top - bottom;

    const topThreshold = top - span * 0.3;
    const bottonThreshold = bottom + span * 0.4;

    for (const point of mesh.points)
    {
      if (point.currentWorldPos[0] <= topThreshold && point.currentWorldPos[0] >= bottonThreshold)
      {
        pinPoint(point);
      }
    }
  }

  function pinPoint(point_idx)
  {
    mesh.points[point_idx].isPinned = true;
    mesh.points[point_idx].invMass = 0;
  }

  function pinPoint(point)
  {
    point.isPinned = true;
    point.invMass = 0;
  }

  // 0 - x, 1 - y, 2 - z
  function getCoordRange(coord_idx)
  {
    let bottom = Infinity;
    let top = -Infinity;

    for (const point of mesh.points)
    {
      const coord = point.currentWorldPos[coord_idx];
      if (coord > top)
      {
        top = coord;
      }
      if (coord < bottom)
      {
        bottom = coord;
      }
    }

    return {top : top, bottom : bottom};
  }

  function hitmanStep(dt)
  {
    const subSteps = subStepsSlider.value();
    const subDt = dt / subSteps;

    for (let step = 0; step < subSteps; step++)
    {
      for (const edge of mesh.edges)
      {
        const first = edge.first;
        const second = edge.second;
        const firstHasMass = mesh.points[first].invMass > 0;
        const secondHasMass = mesh.points[second].invMass > 0;
        if (!firstHasMass && !secondHasMass)
        {
          continue;
        }

        const fromFirstToSecond = vec3.create();
        vec3.subtract(
          fromFirstToSecond,
          mesh.points[second].currentWorldPos,
          mesh.points[first].currentWorldPos);

        const length = vec3.length(fromFirstToSecond);
        if (length < 1e-9)
        {
          continue;
        }

        const relativeStretch = (length - edge.restLen) / length;

        if (firstHasMass && secondHasMass)
        {
          vec3.scaleAndAdd(
            mesh.points[first].currentWorldPos,
            mesh.points[first].currentWorldPos,
            fromFirstToSecond,
            0.5 * relativeStretch);
          vec3.scaleAndAdd(
            mesh.points[second].currentWorldPos,
            mesh.points[second].currentWorldPos,
            fromFirstToSecond,
            -0.5 * relativeStretch);
        }
        else if (firstHasMass)
        {
          vec3.scaleAndAdd(
            mesh.points[first].currentWorldPos,
            mesh.points[first].currentWorldPos,
            fromFirstToSecond,
            relativeStretch);
        }
        else
        {
          vec3.scaleAndAdd(
            mesh.points[second].currentWorldPos,
            mesh.points[second].currentWorldPos,
            fromFirstToSecond,
            -relativeStretch);
        }
      }
    }
  }

  function pbdStep(dt)
  {
    const subSteps = subStepsSlider.value();
    const subDt = dt / subSteps;

    for (let step = 0; step < subSteps; step++)
    {
      for (const edge of mesh.edges)
      {
        const first = edge.first;
        const second = edge.second;
        const firstInvMass = mesh.points[first].isPinned ? 0 : mesh.points[first].invMass;
        const secondInvMass = mesh.points[second].isPinned ? 0 : mesh.points[second].invMass;
        const invEffMass = firstInvMass + secondInvMass;
        if (invEffMass < 1e-9)
        {
          continue;
        }

        const fromFirstToSecond = vec3.create();
        vec3.subtract(
          fromFirstToSecond,
          mesh.points[second].currentWorldPos,
          mesh.points[first].currentWorldPos);

        const length = vec3.length(fromFirstToSecond);
        if (length < 1e-9)
        {
          continue;
        }

        const stretch = (length - edge.restLen);

        const correction = stiffnessSlider.value() * stretch / length / invEffMass;

        if (!mesh.points[first].isPinned)
        {
          vec3.scaleAndAdd(
            mesh.points[first].currentWorldPos,
            mesh.points[first].currentWorldPos,
            fromFirstToSecond,
            firstInvMass * correction);
        }
        if (!mesh.points[second].isPinned)
        {
          vec3.scaleAndAdd(
            mesh.points[second].currentWorldPos,
            mesh.points[second].currentWorldPos,
            fromFirstToSecond,
            -secondInvMass * correction);
        }
      }
    }
  }

  function xpbdStep(dt)
  {
    const subSteps = subStepsSlider.value();
    const subDt = dt / subSteps;
    const complianceScaled = complianceSlider.value() / (dt * dt);

    const lambdas = new Float32Array(mesh.edges.length);

    for (let step = 0; step < subSteps; step++)
    {
      for (let edgeIdx = 0; edgeIdx < mesh.edges.length; edgeIdx++)
      {
        const edge = mesh.edges[edgeIdx];
        const first = edge.first;
        const second = edge.second;
        const firstInvMass = mesh.points[first].isPinned ? 0 : mesh.points[first].invMass;
        const secondInvMass = mesh.points[second].isPinned ? 0 : mesh.points[second].invMass;
        const invEffMass = firstInvMass + secondInvMass;
        if (invEffMass < 1e-9)
        {
          continue;
        }

        const fromFirstToSecond = vec3.create();
        vec3.subtract(
          fromFirstToSecond,
          mesh.points[second].currentWorldPos,
          mesh.points[first].currentWorldPos);

        const length = vec3.length(fromFirstToSecond);
        if (length < 1e-9)
        {
          continue;
        }

        const stretch = (length - edge.restLen);

        const deltaLambda =
          -(stretch + complianceScaled * lambdas[edgeIdx]) / (invEffMass + complianceScaled);
        lambdas[edgeIdx] += deltaLambda;

        const correction = deltaLambda / length;
        if (!mesh.points[first].isPinned)
        {
          vec3.scaleAndAdd(
            mesh.points[first].currentWorldPos,
            mesh.points[first].currentWorldPos,
            fromFirstToSecond,
            -firstInvMass * correction);
        }
        if (!mesh.points[second].isPinned)
        {
          vec3.scaleAndAdd(
            mesh.points[second].currentWorldPos,
            mesh.points[second].currentWorldPos,
            fromFirstToSecond,
            secondInvMass * correction);
        }
      }
    }
  }

  function floorCollisionCheck()
  {
    const floor = -20;
    for (const point of mesh.points)
    {
      if (point.isPinned || point.currentWorldPos[1] > floor)
      {
        continue;
      }

      const posChange = vec3.create();
      vec3.subtract(posChange, point.currentWorldPos, point.prevWorldPos);
      if (posChange[1] >= 0)
      {
        continue;
      }

      point.prevWorldPos[1] = floor - restitudeSlider.value() * posChange[1];

      const horShiftLen = Math.sqrt(posChange[0] * posChange[0] + posChange[2] * posChange[2]);
      if (horShiftLen > 1e-9)
      {
        const scale = Math.max(0, 1 + frictionSlider.value() * posChange[1]);
        point.currentWorldPos[0] = point.prevWorldPos[0] + posChange[0] * scale;
        point.currentWorldPos[2] = point.prevWorldPos[2] + posChange[2] * scale;
      }

      point.currentWorldPos[1] = floor;
    }
  }

  function selfCollisionCheck()
  {
    const radius = 1;
    const diameter = radius * 2;
    const diameterSq = diameter * diameter;

    for (let first = 0; first < mesh.points.length; first++)
    {
      for (let second = first + 1; second < mesh.points.length; second++)
      {
        const fromFirstToSecond = vec3.create();
        vec3.subtract(
          fromFirstToSecond,
          mesh.points[second].currentWorldPos,
          mesh.points[first].currentWorldPos);

        const lengthSq = vec3.squaredLength(fromFirstToSecond);
        if (lengthSq >= diameterSq || lengthSq < 1e-9)
        {
          continue;
        }

        const length = Math.sqrt(lengthSq);
        const firstInvMass = mesh.points[first].isPinned ? 0 : mesh.points[first].invMass;
        const secondInvMass = mesh.points[second].isPinned ? 0 : mesh.points[second].invMass;
        const invEffMass = firstInvMass + secondInvMass;
        if (invEffMass < 1e-9)
        {
          continue;
        }

        const correction = (diameter - length) / length / invEffMass;

        if (!mesh.points[first].isPinned)
        {
          vec3.scaleAndAdd(
            mesh.points[first].currentWorldPos,
            mesh.points[first].currentWorldPos,
            fromFirstToSecond,
            -firstInvMass * correction);
        }
        if (!mesh.points[second].isPinned)
        {
          vec3.scaleAndAdd(
            mesh.points[second].currentWorldPos,
            mesh.points[second].currentWorldPos,
            fromFirstToSecond,
            secondInvMass * correction);
        }
      }
    }
  }

  function drawTask()
  {
    for (const edge of mesh.edges)
    {
      const currentVecFromFirstToSecond = vec3.create();
      vec3.subtract(
        currentVecFromFirstToSecond,
        mesh.points[edge.second].currentWorldPos,
        mesh.points[edge.first].currentWorldPos)
      const stretch = Math.abs(vec3.length(currentVecFromFirstToSecond) - edge.restLen) /
        Math.max(edge.restLen, 0.0001);
      p.push();
      p.noFill();
      p.strokeWeight(0.5);
      p.stroke(
        Math.min(255, 50 + stretch * 1400),
        Math.max(0, 70 - stretch * 900),
        Math.max(30, 70 - stretch * 200));
      p.line(
        ...mesh.points[edge.first].currentWorldPos, ...mesh.points[edge.second].currentWorldPos);
      p.pop();
    }

    for (let i = 0; i < mesh.points.length; i++)
    {
      p.push();
      p.noStroke();
      if (i === grabbedVertIdx)
        p.ambientMaterial(255, 220, 0);
      else if (mesh.points[i].isPinned)
        p.ambientMaterial(255, 70, 70);
      else
        p.ambientMaterial(100, 150, 255);
      p.translate(...mesh.points[i].currentWorldPos);
      p.sphere(pointRadius, 10, 10);
      p.pop();
    }
  }

  function drawOverlay()
  {
    p.push();
    p.noStroke();
    p.fill(40, 40, 42, 220);
    p.translate(-p.width / 2, -p.height / 2);
    p.rect(16, 16, 550, 200, 8);
    p.fill(229, 231, 235);
    p.textSize(14);
    p.textAlign(p.LEFT, p.TOP);
    p.text("Sphere", 32, 30);
    p.text(`Mode: ${sim_modes[currentMode].label}`, 32, 56);
    p.text(`Scene: ${sim_scenes[currentScene].label}`, 32, 74);
    p.text("Press R to reset, T to stop/continue simulation", 32, 96);
    p.text("M to switch simulation mode, S to switch scene (free, hanged, stretched)", 32, 114);
    p.pop();
  }
  function getRay(screenX, screenY)
  {
    let ndcX = (2.0 * screenX) / p.width - 1.0;
    let ndcY = (2.0 * screenY) / p.height - 1.0;

    let nearPoint = vec4.fromValues(ndcX, ndcY, -1.0, 1.0);
    let farPoint = vec4.fromValues(ndcX, ndcY, 1.0, 1.0);

    let invProj = mat4.create();
    let invView = mat4.create();
    mat4.invert(invProj, proj);
    mat4.invert(invView, view);

    vec4.transformMat4(nearPoint, nearPoint, invProj);
    vec4.transformMat4(farPoint, farPoint, invProj);

    nearPoint = vec4.scale(nearPoint, nearPoint, 1.0 / nearPoint[3]);
    farPoint = vec4.scale(farPoint, farPoint, 1.0 / farPoint[3]);

    vec4.transformMat4(nearPoint, nearPoint, invView);
    vec4.transformMat4(farPoint, farPoint, invView);

    let origin = vec3.fromValues(nearPoint[0], nearPoint[1], nearPoint[2]);
    let direction = vec3.create();
    vec3.subtract(direction, vec3.fromValues(farPoint[0], farPoint[1], farPoint[2]), origin);
    vec3.normalize(direction, direction);

    return {origin, direction};
  }

  function intersectRayPlane(ray_origin, ray_dir, plane_point, plane_normal)
  {
    let denom = vec3.dot(plane_normal, ray_dir);
    if (Math.abs(denom) < 1e-6)
      return null;
    let diff = vec3.create();
    vec3.subtract(diff, plane_point, ray_origin);
    let t = vec3.dot(diff, plane_normal) / denom;
    if (t < 0)
      return null;
    let point = vec3.create();
    vec3.scaleAndAdd(point, ray_origin, ray_dir, t);
    return point;
  }

  function pickPoint(ray_origin, ray_dir)
  {
    let closestDist = Infinity;
    let closestIndex = -1;
    let closestPoint = null;

    for (let i = 0; i < mesh.points.length; i++)
    {
      const hit =
        intersectRaySphere(ray_origin, ray_dir, mesh.points[i].currentWorldPos, pointRadius);

      if (hit)
      {
        const dist = vec3.distance(ray_origin, hit);
        if (dist > 0 && dist < closestDist)
        {
          closestDist = dist;
          closestIndex = i;
          closestPoint = hit;
        }
      }
    }

    return {
      index : closestIndex,
      point : closestPoint,
    };
  }

  function intersectRaySphere(ray_origin, ray_dir, sphere_center, radius)
  {
    let originToSphereCenter = vec3.create();
    vec3.subtract(originToSphereCenter, ray_origin, sphere_center);
    let a = vec3.dot(ray_dir, ray_dir);
    let b = 2.0 * vec3.dot(originToSphereCenter, ray_dir);
    let c = vec3.dot(originToSphereCenter, originToSphereCenter) - radius * radius;
    let discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
      return null;
    let t = (-b - Math.sqrt(discriminant)) / (2.0 * a);
    if (t < 0)
    {
      t = (-b + Math.sqrt(discriminant)) / (2.0 * a);
    }
    if (t < 0)
    {
      return null;
    }
    let point = vec3.create();
    vec3.scaleAndAdd(point, ray_origin, ray_dir, t);
    return point;
  }
}

function makeSphere()
{
  const subDivs = 1;
  const radius = 15;

  const ratio = (1 + Math.sqrt(5)) / 2;
  let verts = [
    vec3.normalize(vec3.create(), vec3.fromValues(-1, ratio, 0)),
    vec3.normalize(vec3.create(), vec3.fromValues(1, ratio, 0)),
    vec3.normalize(vec3.create(), vec3.fromValues(-1, -ratio, 0)),
    vec3.normalize(vec3.create(), vec3.fromValues(1, -ratio, 0)),
    vec3.normalize(vec3.create(), vec3.fromValues(0, -1, ratio)),
    vec3.normalize(vec3.create(), vec3.fromValues(0, 1, ratio)),
    vec3.normalize(vec3.create(), vec3.fromValues(0, -1, -ratio)),
    vec3.normalize(vec3.create(), vec3.fromValues(0, 1, -ratio)),
    vec3.normalize(vec3.create(), vec3.fromValues(ratio, 0, -1)),
    vec3.normalize(vec3.create(), vec3.fromValues(ratio, 0, 1)),
    vec3.normalize(vec3.create(), vec3.fromValues(-ratio, 0, -1)),
    vec3.normalize(vec3.create(), vec3.fromValues(-ratio, 0, 1)),
  ];
  let faces = [
    [ 0, 11, 5 ], [ 0, 5, 1 ],  [ 0, 1, 7 ],   [ 0, 7, 10 ], [ 0, 10, 11 ],
    [ 1, 5, 9 ],  [ 5, 11, 4 ], [ 11, 10, 2 ], [ 10, 7, 6 ], [ 7, 1, 8 ],
    [ 3, 9, 4 ],  [ 3, 4, 2 ],  [ 3, 2, 6 ],   [ 3, 6, 8 ],  [ 3, 8, 9 ],
    [ 4, 9, 5 ],  [ 2, 4, 11 ], [ 6, 2, 10 ],  [ 8, 6, 7 ],  [ 9, 8, 1 ],
  ];
  for (let s = 0; s < subDivs; s++)
  {
    const mid = new Map();
    const subdivide = (first, second) => {
      const key = first < second ? `${first},${second}` : `${second},${first}`;
      if (mid.has(key))
      {
        return mid.get(key);
      }
      const index = verts.length;
      const newVert = vec3.create();
      vec3.add(newVert, verts[first], verts[second]);
      vec3.scale(newVert, newVert, 0.5);
      vec3.normalize(newVert, newVert);
      verts.push(newVert);
      mid.set(key, index);
      return index;
    };
    const newFaces = [];
    for (const [a, b, c] of faces)
    {
      const ab = subdivide(a, b), bc = subdivide(b, c), ca = subdivide(c, a);
      newFaces.push([ a, ab, ca ], [ b, bc, ab ], [ c, ca, bc ], [ ab, bc, ca ]);
    }
    faces = newFaces;
  }

  const points = [];
  points.push({
    mass : 1,
    invMass : 1,
    prevWorldPos : vec3.fromValues(0, 0, 0),
    currentWorldPos : vec3.fromValues(0, 0, 0),
    prevVelocity : vec3.fromValues(0, 0, 0),
    currentVelocity : vec3.fromValues(0, 0, 0),
    isPinned : false,
  });
  for (const vert of verts)
  {
    vec3.scale(vert, vert, radius);

    points.push({
      mass : 1,
      invMass : 1,
      prevWorldPos : vec3.clone(vert),
      currentWorldPos : vert,
      prevVelocity : vec3.fromValues(0, 0, 0),
      currentVelocity : vec3.fromValues(0, 0, 0),
      isPinned : false,
    });
  }
  const mesh = {
    points : points,
    edges : [],
    tets : [],
    triangles : [],
  };

  const seenEdge = new Set();
  const addEdge = (first, second) => {
    const key = first < second ? `${first},${second}` : `${second},${first}`;
    if (seenEdge.has(key))
      return;
    seenEdge.add(key);
    const vecFromFirstToSecond = vec3.create();
    vec3.subtract(
      vecFromFirstToSecond,
      mesh.points[second].currentWorldPos,
      mesh.points[first].currentWorldPos);
    const length = vec3.length(vecFromFirstToSecond);
    mesh.edges.push({first : first, second : second, restLen : length});
  };

  const offset = 1;
  for (const [a, b, c] of faces)
  {
    const indA = a + offset;
    const indB = b + offset;
    const indC = c + offset;

    mesh.triangles.push({a : indA, b : indB, c : indC});

    mesh.tets.push({a : 0, b : indA, c : indB, d : indC});
    addEdge(0, indA);
    addEdge(0, indB);
    addEdge(0, indC);

    addEdge(indA, indB);
    addEdge(indB, indC);
    addEdge(indC, indA);
  }

  return mesh;
}

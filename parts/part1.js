import * as mat3 from "../libraries/esm/mat3.js";
import * as mat4 from "../libraries/esm/mat4.js";
import * as quat from "../libraries/esm/quat.js";
import * as vec3 from "../libraries/esm/vec3.js";

const sim_modes = [
  {label : "1) Global coordinate system"},
  {label : "2.1) Local coordinate system without gyroscopic term"},
  {label : "2.2.1) Local coordinate system with gyroscopic term, explicit"},
  {label : "2.2.2) Local coordinate system with gyroscopic term, implicit"},
];
/**
 * Get the skew matrix from vector
 * @param {ReadonlyVec3} vec The vector
 * @returns {mat3} The skew matrix
 */
function skew(vec)
{
  let [x, y, z] = vec;
  // clang-format off
  // might seem transposed, but its right because matrices are column-major in gl-matrix!
  return mat3.fromValues(
    0, z, -y, // column 1
    -z, 0, x, // column 2
    y, -x, 0  // column 3
  );
  // clang-format on
}

export function createPart1(p)
{
  let renderParams = {
    pos : vec3.fromValues(0, 0, 0),
    width : 0,
    height : 0,
    depth : 0,
    rotMatrix : [ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 ],
  };
  let physicsParams = {
    mass : 0,
    localInertiaTensor : mat3.create(),
    worldInertiaTensor : mat3.create(),
    currentQuat : quat.create(),
    nextQuat : quat.create(),
    currentAngleSpeed : vec3.fromValues(0, 0, 0),
    nextAngleSpeed : vec3.fromValues(0, 0, 0),
    initialAngularMomentum : vec3.create(),
    currentAngularMomentum : vec3.create(),
    initialEnergy : 0,
    currentEnergy : 0,
    stopSim : false,
  };
  let currentMode = 0;

  return {
    init() {
      let body = makeBody(currentMode);
      renderParams = body.renderParams;
      physicsParams = body.physicsParams;
      p.camera(100, 50, 100, 0, 0, 0, 0, -1, 0);
      p.perspective();
    },

    reset() {
      let body = makeBody(currentMode);
      renderParams = body.renderParams;
      physicsParams = body.physicsParams;
      p.camera(100, 50, 100, 0, 0, 0, 0, -1, 0);
      p.perspective();
    },

    update(dt) {
      if (physicsParams.stopSim)
      {
        return;
      }
      switch (currentMode)
      {
      case 0:
        globalCoordsSim(dt);
        break;
      case 1:
        localCoordsSim(dt);
        break;
      case 2:
        localCoordsSemiExplicitWithGyroSim(dt);
        break;
      case 3:
        localCoordsImplicitWithGyroSim(dt);
        break;
      }
      updateEnergy();
      updateRenderParams();
    },

    render() { drawTask(); },

    render2D() { drawOverlay(); },

    keyPressed(key) {
      if (key.toLowerCase() === "m")
      {
        currentMode = (currentMode + 1) % sim_modes.length;
        this.reset();
      }
      if (key.toLowerCase() === "r")
      {
        this.reset();
      }
      if (key.toLowerCase() === "t")
      {
        physicsParams.stopSim = !physicsParams.stopSim;
      }
    },
  };

  function globalCoordsSim(dt)
  {
    vec3.copy(physicsParams.currentAngleSpeed, physicsParams.nextAngleSpeed);
    quat.copy(physicsParams.currentQuat, physicsParams.nextQuat);

    updateWorldInertia();
    const worldInertiaInv = mat3.create()

    mat3.invert(worldInertiaInv, physicsParams.worldInertiaTensor);
    vec3.transformMat3(
      physicsParams.nextAngleSpeed, physicsParams.currentAngularMomentum, worldInertiaInv);

    updateNextQuat(dt, true);

    vec3.transformMat3(
      physicsParams.currentAngularMomentum,
      physicsParams.nextAngleSpeed,
      physicsParams.worldInertiaTensor);
  }

  function updateWorldInertia()
  {
    const rotMatrix = mat3.create();
    const rotMatrixTransposed = mat3.create();
    mat3.fromQuat(rotMatrix, physicsParams.currentQuat);
    mat3.transpose(rotMatrixTransposed, rotMatrix);

    const temp = mat3.create();

    mat3.multiply(temp, physicsParams.localInertiaTensor, rotMatrixTransposed);
    mat3.multiply(physicsParams.worldInertiaTensor, rotMatrix, temp);
  }

  function localCoordsSim(dt)
  {
    const Jw = vec3.create();

    vec3.transformMat3(Jw, physicsParams.currentAngleSpeed, physicsParams.localInertiaTensor);

    vec3.copy(
      physicsParams.currentAngleSpeed,
      physicsParams.nextAngleSpeed); // doesn't change in this scenario

    updateNextQuat(dt, false);

    vec3.transformMat3(
      physicsParams.currentAngularMomentum,
      physicsParams.currentAngleSpeed,
      physicsParams.localInertiaTensor);
    quat.copy(physicsParams.currentQuat, physicsParams.nextQuat);
  }

  function localCoordsSemiExplicitWithGyroSim(dt)
  {
    const Jw = vec3.create();
    const gyroTerm = vec3.create();
    vec3.transformMat3(Jw, physicsParams.currentAngleSpeed, physicsParams.localInertiaTensor);
    vec3.cross(gyroTerm, physicsParams.currentAngleSpeed, Jw);
    gyroTerm[0] = -dt * gyroTerm[0] / physicsParams.localInertiaTensor[0];
    gyroTerm[1] = -dt * gyroTerm[1] / physicsParams.localInertiaTensor[4];
    gyroTerm[2] = -dt * gyroTerm[2] / physicsParams.localInertiaTensor[8];

    vec3.add(physicsParams.nextAngleSpeed, physicsParams.currentAngleSpeed, gyroTerm);

    vec3.copy(physicsParams.currentAngleSpeed, physicsParams.nextAngleSpeed);
    updateNextQuat(dt, false);

    vec3.transformMat3(
      physicsParams.currentAngularMomentum,
      physicsParams.nextAngleSpeed,
      physicsParams.localInertiaTensor);
    quat.copy(physicsParams.currentQuat, physicsParams.nextQuat);
  }

  function localCoordsImplicitWithGyroSim(dt)
  {
    const Jw = vec3.create();
    const gyroTerm = vec3.create();

    vec3.transformMat3(Jw, physicsParams.currentAngleSpeed, physicsParams.localInertiaTensor);
    vec3.cross(gyroTerm, Jw, physicsParams.currentAngleSpeed);
    vec3.scale(gyroTerm, gyroTerm, dt);

    const skewedOmega = skew(physicsParams.currentAngleSpeed);
    const skewedJw = skew(Jw);

    const skewedOmegaTimesJ = mat3.create();
    mat3.multiply(skewedOmegaTimesJ, skewedOmega, physicsParams.localInertiaTensor);

    const yakobi = mat3.create();
    mat3.subtract(yakobi, skewedOmegaTimesJ, skewedJw);
    mat3.multiplyScalar(yakobi, yakobi, dt);
    mat3.add(yakobi, yakobi, physicsParams.localInertiaTensor);

    const yakobiInv = mat3.create();
    mat3.invert(yakobiInv, yakobi);

    const angleCorrection = vec3.create();
    vec3.transformMat3(angleCorrection, gyroTerm, yakobiInv);

    vec3.add(physicsParams.nextAngleSpeed, physicsParams.currentAngleSpeed, angleCorrection);
    vec3.copy(physicsParams.currentAngleSpeed, physicsParams.nextAngleSpeed);

    updateNextQuat(dt, false);

    vec3.transformMat3(
      physicsParams.currentAngularMomentum,
      physicsParams.nextAngleSpeed,
      physicsParams.localInertiaTensor);
    quat.copy(physicsParams.currentQuat, physicsParams.nextQuat);
  }

  function updateNextQuat(dt, isWorld)
  {
    const angleQuat = quat.fromValues(
      0.5 * dt * physicsParams.nextAngleSpeed[0],
      0.5 * dt * physicsParams.nextAngleSpeed[1],
      0.5 * dt * physicsParams.nextAngleSpeed[2],
      0);
    const finalAddQuat = quat.create();
    if (isWorld === true)
    {
      quat.multiply(finalAddQuat, angleQuat, physicsParams.currentQuat);
    }
    else
    {
      quat.multiply(finalAddQuat, physicsParams.currentQuat, angleQuat);
    }
    quat.add(physicsParams.nextQuat, physicsParams.currentQuat, finalAddQuat);
    quat.normalize(physicsParams.nextQuat, physicsParams.nextQuat);
  }

  function updateEnergy()
  {
    const Jw = vec3.create();
    if (currentMode === 0)
    {
      vec3.transformMat3(Jw, physicsParams.currentAngleSpeed, physicsParams.worldInertiaTensor);
      physicsParams.currentEnergy = 0.5 * vec3.dot(Jw, physicsParams.currentAngleSpeed);
    }
    else
    {
      vec3.transformMat3(Jw, physicsParams.currentAngleSpeed, physicsParams.localInertiaTensor);
      physicsParams.currentEnergy = 0.5 * vec3.dot(Jw, physicsParams.currentAngleSpeed);
    }
  }

  function updateRenderParams()
  {
    const rotMat = mat4.create();
    mat4.fromQuat(rotMat, physicsParams.nextQuat);
    renderParams.rotMatrix = [...rotMat ];
  }

  function drawTask()
  {
    p.push();
    p.applyMatrix(renderParams.rotMatrix);
    p.ambientMaterial(65, 130, 255);
    p.box(renderParams.width, renderParams.height, renderParams.depth);
    p.pop();

    drawPlane(500, 0, -30, 0);
    
    const scaledL = vec3.create();
    vec3.normalize(scaledL, physicsParams.initialAngularMomentum);
    vec3.multiply(scaledL, scaledL, vec3.fromValues(50, 50, 50));

    const L = [...scaledL ];

    drawArrow(128, 20, 20, L[0], L[1], L[2]);

    vec3.normalize(scaledL, physicsParams.currentAngularMomentum);
    vec3.multiply(scaledL, scaledL, vec3.fromValues(50, 50, 50));
    const currentL = [...scaledL ];

    drawArrow(200, 50, 50, currentL[0], currentL[1], currentL[2]);

    drawCoordAxis();

  }

  function drawOverlay()
  {
    p.push();
    p.noStroke();
    p.fill(40, 40, 42, 220);
    p.translate(-p.width / 2, -p.height / 2);
    p.rect(16, 16, 500, 350, 8);
    p.fill(220, 230, 230);
    p.textSize(14);
    p.textAlign(p.LEFT, p.TOP);
    p.text("Part 1: 3D rectangular rigid body rotation", 32, 30);
    p.text(`Mode: ${sim_modes[currentMode].label}`, 32, 56);
    p.text("Press M to switch mode, R to reset. T to stop/continue simulation", 32, 82);
    p.text(
      `Inertia tensor (I) xx:${physicsParams.localInertiaTensor[0].toFixed(4)}, yy:${
        physicsParams.localInertiaTensor[4].toFixed(
          4)}, zz:${physicsParams.localInertiaTensor[8].toFixed(4)}`,
      32,
      144);
    p.text(
      `Initial Angular Momentum (L) x:${physicsParams.initialAngularMomentum[0].toFixed(4)}, y:${
        physicsParams.initialAngularMomentum[1].toFixed(
          4)}, z:${physicsParams.initialAngularMomentum[2].toFixed(4)}`,
      32,
      168);
    p.text(
      `Current Angular Momentum (L) x:${physicsParams.currentAngularMomentum[0].toFixed(4)}, y:${
        physicsParams.currentAngularMomentum[1].toFixed(
          4)}, z:${physicsParams.currentAngularMomentum[2].toFixed(4)}`,
      32,
      192);
    p.text(`Initial Energy (E) :${physicsParams.initialEnergy.toFixed(4)}`, 32, 216);
    p.text(`Current Energy (E) :${physicsParams.currentEnergy.toFixed(4)}`, 32, 240);

    p.fill(175, 50, 50);
    p.text(`Bleak red arrow - Initial L`, 32, 270);
    p.fill(250, 50, 50);
    p.text(`Red arrow - Current L`, 32, 290);
    p.pop();
  }

  function drawArrow(colorR, colorG, colorB, vecX, vecY, vecZ)
  {
    const up = vec3.fromValues(0, 1, 0);
    const dir = vec3.fromValues(vecX, vecY, vecZ);
    vec3.normalize(dir, dir);
    const angle = vec3.angle(up, dir);
    let axis = vec3.create();
    vec3.cross(axis, up, dir);
    if (vec3.squaredLength(axis) < 0.0001)
    {
      axis = vec3.fromValues(1, 0, 0);
    }

    const renderAxis = [...axis ];
    p.push();
    p.stroke(colorR, colorG, colorB);
    p.strokeWeight(2);
    p.line(0, 0, 0, vecX, vecY, vecZ);
    p.translate(vecX, vecY, vecZ);
    p.rotate(angle, renderAxis);
    p.cone(2, 5);
    p.pop();
  }

  function drawCoordAxis()
  {
    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
    p.push();
    p.applyMatrix(renderParams.rotMatrix);
    p.strokeWeight(2);
    p.stroke(255, 50, 50, 100);
    p.line(0, 0, 0, 50, 0, 0);
    p.stroke(50, 255, 50, 100);
    p.line(0, 0, 0, 0, 50, 0);
    p.stroke(50, 50, 255, 100);
    p.line(0, 0, 0, 0, 0, 50);
    p.pop();
    p.drawingContext.enable(p.drawingContext.DEPTH_TEST);
  }

  function drawPlane(size, offsetX, offsetY, offsetZ)
  {
    p.push();
    p.translate(offsetX, offsetY, offsetZ);
    p.rotateX(p.HALF_PI);
    p.noStroke();
    p.ambientMaterial(150, 150, 150);
    p.plane(size, size);
    p.pop();
  }
}


function makeBody(currentMode)
{
  const mass = 1.0;
  const width = 60;
  const height = 20;
  const depth = 20;

  const inertiaX = mass * (depth * depth + height * height) / 12.0;
  const inertiaY = mass * (depth * depth + width * width) / 12.0;
  const inertiaZ = mass * (width * width + height * height) / 12.0;

  // clang-format off
  const inertia = mat3.fromValues(
    inertiaX, 0, 0, // column 1
    0, inertiaY, 0, // column 2
    0, 0, inertiaZ, // column 3
  );
  // clang-format on
  let wx = 0.2;
  let wy = 0.2;
  let wz = 5.0;
  if (currentMode === 0 || currentMode === 1)
  {
    wx = 0.0;
    wy = 0.1;
    wz = 5.0;
  }

  const currentAngleSpeed = vec3.fromValues(wx, wy, wz);
  const nextAngleSpeed = vec3.clone(currentAngleSpeed);

  const initialQuat = quat.create();
  const nextQuat = quat.clone(initialQuat);

  // making L global in first scene
  const initialAngularMomentum = vec3.create();
  const worldInertia = mat3.create();
  if (currentMode === 0)
  {
    const rotMatrix = mat3.create();
    const rotMatrixTransposed = mat3.create();
    mat3.fromQuat(rotMatrix, initialQuat);
    mat3.transpose(rotMatrixTransposed, rotMatrix);

    const temp = mat3.create();

    mat3.multiply(temp, inertia, rotMatrixTransposed);
    mat3.multiply(worldInertia, rotMatrix, temp);

    vec3.transformMat3(initialAngularMomentum, currentAngleSpeed, worldInertia);
  }
  else
  {
    vec3.transformMat3(initialAngularMomentum, currentAngleSpeed, inertia);
  }

  const currentAngularMomentum = vec3.clone(initialAngularMomentum)

  const energy = 0.5 * vec3.dot(initialAngularMomentum, currentAngleSpeed);

  return {
    renderParams : {
      pos : vec3.fromValues(0, 0, 0),
      width : width,
      height : height,
      depth : depth,
      rotMatrix : [ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 ],
    },
    physicsParams : {
      mass : mass,
      localInertiaTensor : inertia,
      worldInertiaTensor : worldInertia,
      currentQuat : initialQuat,
      nextQuat : nextQuat,
      currentAngleSpeed : currentAngleSpeed,
      nextAngleSpeed : nextAngleSpeed,
      initialAngularMomentum : initialAngularMomentum,
      currentAngularMomentum : currentAngularMomentum,
      initialEnergy : energy,
      currentEnergy : energy,
      stopSim : false,
    },
  };
}

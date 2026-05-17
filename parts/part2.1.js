import * as mat3 from "../libraries/esm/mat3.js";
import * as mat4 from "../libraries/esm/mat4.js";
import * as quat from "../libraries/esm/quat.js";
import * as vec3 from "../libraries/esm/vec3.js";

const sim_modes = [
  {label : "1.1) Rigid Body connected to spring (calculated as force)"},
  {label : "1.2) Rigid Body connected to spring (calculated as budda spring)"},
];

export function createPart2_1(p)
{
  let body = {
    mass : 0,
    invMass : 0,
    width : 0,
    height : 0,
    depth : 0,
    currentWorldPos : vec3.fromValues(0, 0, 0),
    nextWorldPos : vec3.fromValues(0, 0, 0),
    currentVelocity : vec3.fromValues(0, 0, 0),
    nextVelocity : vec3.fromValues(0, 0, 0),
    connectionPos : vec3.fromValues(0, 0, 0),
    localInertiaTensor : mat3.create(),
    currentAngularMomentum : vec3.create(),
    currentQuat : quat.create(),
    nextQuat : quat.create(),
    currentAngleSpeed : vec3.fromValues(0, 0, 0),
    nextAngleSpeed : vec3.fromValues(0, 0, 0),
    currentTransformMatrix : mat4.create(),
  };
  let spring = {
    worldPos : vec3.create(),
    restLength : 30,
    stiffness : 1,
    damping : 0.1,
  };
  let frameInfo = {
    force : vec3.create(),
    torque : vec3.create(),
    correction : 0,
  };
  let stopSim = true;
  let currentMode = 0;

  return {
    init() {
      body = makeBody();

      if (currentMode == 0)
      {
        spring = makeForceSpring();
      }
      else
      {
        spring = makeBuddaSpring()
      }

      p.camera(150, 75, 150, 0, 0, 0, 0, -1, 0);
      p.perspective();
    },

    reset() {
      body = makeBody();

      if (currentMode == 0)
      {
        spring = makeForceSpring();
      }
      else
      {
        spring = makeBuddaSpring()
      }

      p.camera(150, 75, 150, 0, 0, 0, 0, -1, 0);
      p.perspective();
    },

    update(dt) {
      if (stopSim)
      {
        return;
      }

      vec3.copy(body.currentWorldPos, body.nextWorldPos);
      vec3.copy(body.currentVelocity, body.nextVelocity);
      vec3.copy(body.currentAngleSpeed, body.nextAngleSpeed);
      vec3.copy(body.currentQuat, body.nextQuat);

      let params = {};
      switch (currentMode)
      {
      case 0:
        params = updateSpringForcesAndTorque(dt);
        break;
      case 1:
        params = updateSpringImpulse(dt);
        break;
      }

      vec3.add(
        body.nextWorldPos, body.currentWorldPos, vec3.scale(vec3.create(), body.nextVelocity, dt));

      updateNextQuat(dt, false);

      mat4.fromRotationTranslation(body.currentTransformMatrix, body.nextQuat, body.nextWorldPos);
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
        stopSim = !stopSim;
      }
    },
  };

  function updateSpringForcesAndTorque(dt)
  {
    const params = getSpringCurrentParams();

    const force = vec3.clone(params.vecToSpringPosNorm);
    vec3.scale(force, force, spring.stiffness * params.stretch);
    vec3.scaleAndAdd(force, force, params.connectionPointVelocity, -spring.damping);

    const torque = vec3.create();
    vec3.cross(torque, params.vecToConnectionPos, force);

    vec3.copy(frameInfo.force, force);
    vec3.copy(frameInfo.torque, torque);

    vec3.scale(force, force, body.invMass * dt);
    vec3.add(body.nextVelocity, body.currentVelocity, force);

    vec3.scale(torque, torque, params.invRotMass * dt);
    vec3.add(body.currentAngularMomentum, body.currentAngularMomentum, torque);
    vec3.transformMat3(body.nextAngleSpeed, body.currentAngularMomentum, params.worldInertiaInv);
  }

  function updateSpringImpulse(dt)
  {
    const params = getSpringCurrentParams();

    const effMass = 1.0 / params.invEffMass;
    const Jv = -vec3.dot(params.vecToSpringPosNorm, body.currentVelocity) -
      vec3.dot(params.rCrossN, body.currentAngleSpeed);
    // const Jw = -vec3.dot(params.rCrossN, body.currentAngleSpeed);

    // const omega = 2.0 * Math.PI * spring.hzFreq;
    // const k = effMass * omega * omega;
    // const c = 2.0 * effMass * spring.damping * omega;
    // const denom = c + dt * k;
    // const beta = (dt * k) / denom;
    // const gamma = 1.0 / denom;

    const lambda = getLambda(effMass, params.stretch, Jv, dt);
    // const angleLambda = getLambda(1.0 / params.invRotMass, params.stretch, Jw, dt);

    const force = vec3.clone(params.vecToSpringPosNorm);
    vec3.scale(force, force, -lambda);
    const torque = vec3.clone(params.rCrossN);
    vec3.scale(torque, torque, -lambda);
    // vec3.scale(torque, torque, -lambda);

    vec3.copy(frameInfo.force, force);
    vec3.copy(frameInfo.torque, torque);
    frameInfo.correction = lambda;

    vec3.scale(force, force, body.invMass * dt);
    vec3.add(body.nextVelocity, body.currentVelocity, force);

    vec3.transformMat3(torque, torque, params.worldInertiaInv);
    vec3.scale(torque, torque, dt);
    // vec3.add(body.currentAngularMomentum, body.currentAngularMomentum, torque);
    vec3.add(body.nextAngleSpeed, body.currentAngleSpeed, torque);
  }

  function getLambda(mass, stretch, Jv, dt)
  {
    const omega = 2.0 * Math.PI * spring.hzFreq;
    const k = mass * omega * omega;
    const c = 2.0 * mass * spring.damping * omega;
    const denom = c + dt * k;
    const beta = (dt * k) / denom;
    const gamma = 1.0 / denom;

    const lambda = -(Jv + (beta * stretch) / dt) / (gamma);

    return lambda;
  }

  function getSpringCurrentParams()
  {
    const rotMatrix = mat3.create();
    const rotMatrixTransposed = mat3.create();
    mat3.fromQuat(rotMatrix, body.currentQuat);
    mat3.transpose(rotMatrixTransposed, rotMatrix);

    const temp = mat3.create();
    const worldInertia = mat3.create()
    const worldInertiaInv = mat3.create()

    mat3.multiply(temp, rotMatrixTransposed, body.localInertiaTensor);
    mat3.multiply(worldInertia, rotMatrix, temp);
    mat3.invert(worldInertiaInv, worldInertia);

    const worldConnectionPos = vec3.create();
    vec3.transformMat4(worldConnectionPos, body.connectionPos, body.currentTransformMatrix);
    const vecToConnectionPos = vec3.create();
    vec3.subtract(vecToConnectionPos, worldConnectionPos, body.currentWorldPos);


    const vecToSpringPos = vec3.create();
    vec3.subtract(vecToSpringPos, spring.worldPos, worldConnectionPos);

    const vecLength = vec3.length(vecToSpringPos);
    const stretch = vecLength - spring.restLength;

    const vecToSpringPosNorm = vec3.create();
    vec3.normalize(vecToSpringPosNorm, vecToSpringPos);

    const rCrossN = vec3.create();
    vec3.cross(rCrossN, vecToConnectionPos, vecToSpringPosNorm);

    const connectionPointVelocity = vec3.create();
    vec3.cross(connectionPointVelocity, body.currentAngleSpeed, vecToConnectionPos);
    vec3.add(connectionPointVelocity, connectionPointVelocity, body.currentVelocity);

    const tempVec = vec3.create();
    vec3.transformMat3(tempVec, rCrossN, worldInertiaInv);
    const invRotMass = vec3.dot(rCrossN, tempVec);
    const invEffMass = body.invMass + invRotMass;

    return {
      vecToConnectionPos : vecToConnectionPos,
      vecToSpringPosNorm : vecToSpringPosNorm,
      rCrossN : rCrossN,
      connectionPointVelocity : connectionPointVelocity,
      stretch : stretch,
      invEffMass : invEffMass,
      invRotMass : invRotMass,
      worldInertiaInv : worldInertiaInv,
    };
  }

  function updateNextQuat(dt, isWorld)
  {
    const angleQuat = quat.fromValues(
      0.5 * dt * body.nextAngleSpeed[0],
      0.5 * dt * body.nextAngleSpeed[1],
      0.5 * dt * body.nextAngleSpeed[2],
      0);
    const finalAddQuat = quat.create();
    if (isWorld === true)
    {
      quat.multiply(finalAddQuat, angleQuat, body.currentQuat);
    }
    else
    {
      quat.multiply(finalAddQuat, body.currentQuat, angleQuat);
    }
    quat.add(body.nextQuat, body.currentQuat, finalAddQuat);
    quat.normalize(body.nextQuat, body.nextQuat);
  }

  function drawTask()
  {
    const worldConnectionPos = vec3.create();
    vec3.transformMat4(worldConnectionPos, body.connectionPos, body.currentTransformMatrix);

    p.push();
    p.applyMatrix([...body.currentTransformMatrix ]);
    p.ambientMaterial(65, 130, 255);
    p.box(body.width, body.height, body.depth);
    p.pop();

    drawPlane(500, 0, -30, 0);

    drawSpring(worldConnectionPos);

    const shiftedForce = vec3.clone(frameInfo.force);
    const shiftedTorque = vec3.clone(frameInfo.torque);

    vec3.add(shiftedForce, shiftedForce, worldConnectionPos);
    vec3.add(shiftedTorque, shiftedTorque, worldConnectionPos);

    drawArrow(255, 0, 0, ...worldConnectionPos, ...shiftedForce);
    drawArrow(0, 0, 255, ...worldConnectionPos, ...shiftedTorque);
    drawCoordAxis();
  }

  function drawSpring(worldConnectionPos)
  {
    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
    p.push();
    p.stroke(220, 150, 50, 150);
    p.strokeWeight(3);
    p.line(...spring.worldPos, ...worldConnectionPos);
    p.pop();

    p.push();
    p.translate(...spring.worldPos);
    p.noStroke();
    p.ambientMaterial(
      200,
      200,
      50,
    );
    p.sphere(2, 10, 10);
    p.pop();

    p.push();
    p.translate(...worldConnectionPos);
    p.noStroke();
    p.ambientMaterial(
      200,
      200,
      50,
    );
    p.sphere(2, 10, 10);
    p.pop();

    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
  }

  function drawArrow(colorR, colorG, colorB, fromVecX, fromVecY, fromVecZ, toVecX, toVecY, toVecZ)
  {
    const up = vec3.fromValues(0, 1, 0);
    const dir = vec3.fromValues(toVecX - fromVecX, toVecY - fromVecY, toVecZ - fromVecZ);
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
    p.strokeWeight(1);
    p.line(fromVecX, fromVecY, fromVecZ, toVecX, toVecY, toVecZ);
    p.translate(toVecX, toVecY, toVecZ);
    p.rotate(angle, renderAxis);
    p.cone(1, 3);
    p.pop();
  }
  function drawCoordAxis()
  {
    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
    p.push();
    p.applyMatrix([...body.currentTransformMatrix ]);
    p.strokeWeight(2);
    p.stroke(255, 50, 50, 100);
    p.line(0, 0, 0, 25, 0, 0);
    p.stroke(50, 255, 50, 100);
    p.line(0, 0, 0, 0, 25, 0);
    p.stroke(50, 50, 255, 100);
    p.line(0, 0, 0, 0, 0, 25);
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

  function drawOverlay()
  {
    p.push();
    p.noStroke();
    p.fill(40, 40, 42, 220);
    p.translate(-p.width / 2, -p.height / 2);
    p.rect(16, 16, 500, 350, 8);
    p.fill(229, 231, 235);
    p.textSize(14);
    p.textAlign(p.LEFT, p.TOP);
    p.text("Part 2.1: 3D rectangular rigid bodies connected to spring", 32, 30);
    p.text(`Mode: ${sim_modes[currentMode].label}`, 32, 56);
    p.text("Press M to switch mode, R to reset. T to stop/continue simulation", 32, 82);
    p.text(
      `Inertia tensor (I) xx:${body.localInertiaTensor[0].toFixed(4)}, yy:${
        body.localInertiaTensor[4].toFixed(4)}, zz:${body.localInertiaTensor[8].toFixed(4)}`,
      32,
      144);
    p.text(
      `Current World Position x:${body.currentWorldPos[0].toFixed(4)}, y:${
        body.currentWorldPos[1].toFixed(4)}, z:${body.currentWorldPos[2].toFixed(4)}`,
      32,
      168);
    p.text(
      `Current Force (F) x:${frameInfo.force[0].toFixed(4)}, y:${
        frameInfo.force[1].toFixed(4)}, z:${frameInfo.force[2].toFixed(4)}`,
      32,
      192);
    p.text(
      `Current Torque (T) x:${frameInfo.torque[0].toFixed(4)}, y:${
        frameInfo.torque[1].toFixed(4)}, z:${frameInfo.torque[2].toFixed(4)}`,
      32,
      216);
    if (currentMode === 1)
    {
      p.text(`Current Correction (lambda):${frameInfo.correction.toFixed(4)}`, 32, 240);
    }

    p.fill(250, 50, 50);
    p.text(`Red arrow - Current Force`, 32, 270);
    p.fill(50, 150, 250);
    p.text(`Blue Arrow - Current Torque`, 32, 290);
    p.pop();
  }
}

function makeBody()
{
  const mass = 5.0;
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
  const currentAngleSpeed = vec3.fromValues(0.0, 0.0, 0.0);
  const nextAngleSpeed = vec3.clone(currentAngleSpeed);

  const initialQuat = quat.create();
  const nextQuat = quat.clone(initialQuat);
  const worldPos = vec3.fromValues(0, 0, 0);
  const transform = mat4.create();
  mat4.fromRotationTranslation(transform, initialQuat, worldPos);

  const initialAngularMomentum = vec3.create();
  vec3.transformMat3(initialAngularMomentum, currentAngleSpeed, inertia);

  return {
    mass : mass,
    invMass : 1.0 / mass,
    width : width,
    height : height,
    depth : depth,
    currentWorldPos : worldPos,
    nextWorldPos : worldPos,
    currentVelocity : vec3.fromValues(0, 0, 0),
    nextVelocity : vec3.fromValues(0, 0, 0),
    connectionPos : vec3.fromValues(width / 4, height / 4, depth / 4),
    localInertiaTensor : inertia,
    currentAngularMomentum : initialAngularMomentum,
    currentQuat : initialQuat,
    nextQuat : nextQuat,
    currentAngleSpeed : currentAngleSpeed,
    nextAngleSpeed : nextAngleSpeed,
    currentTransformMatrix : transform,
  };
}

function makeForceSpring()
{
  return {
    worldPos : vec3.fromValues(0, 0, 80),
    restLength : 20,
    stiffness : 5,
    damping : 4,
  };
}

function makeBuddaSpring()
{
  return {
    worldPos : vec3.fromValues(0, 0, 80),
    restLength : 20,
    hzFreq : 0.1,
    damping : 0.5,
  };
}

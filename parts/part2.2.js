import * as mat3 from "../libraries/esm/mat3.js";
import * as mat4 from "../libraries/esm/mat4.js";
import * as quat from "../libraries/esm/quat.js";
import * as vec3 from "../libraries/esm/vec3.js";

const sim_modes = [
  {label : "2.1) Connected with spring (XPBD)"},
];

export function createPart2_2(p)
{
  let body1 = {
    mass : 0,
    invMass : 0,
    width : 0,
    height : 0,
    depth : 0,
    prevWorldPos : vec3.create(),
    currentWorldPos : vec3.create(),
    currentVelocity : vec3.create(),
    connectionPos : vec3.create(),
    localInertiaTensor : mat3.create(),
    worldInertiaTensor : mat3.create(),
    currentAngularMomentum : vec3.create(),
    prevQuat : quat.create(),
    currentQuat : quat.create(),
    currentAngleSpeed : vec3.create(),
    currentTransformMatrix : mat4.create(),
  };
  let body2 = {
    mass : 0,
    invMass : 0,
    width : 0,
    height : 0,
    depth : 0,
    prevWorldPos : vec3.create(),
    currentWorldPos : vec3.create(),
    currentVelocity : vec3.create(),
    connectionPos : vec3.create(),
    localInertiaTensor : mat3.create(),
    worldInertiaTensor : mat3.create(),
    currentAngularMomentum : vec3.create(),
    prevQuat : quat.create(),
    currentQuat : quat.create(),
    currentAngleSpeed : vec3.create(),
    currentTransformMatrix : mat4.create(),
  };
  let spring = {
    restLength : 30,
    stiffness : 1,
    compliance : 0.001,
    dampingLin : 0.3,
    dampingAng : 0.3,
  };
  let lambda = 0;
  let stopSim = true;
  let subSteps = 20;
  let currentMode = 0;

  return {
    init() { resetImpl(); },

    reset() { resetImpl(); },

    update(dt) {
      if (stopSim)
      {
        return;
      }
      switch (currentMode)
      {
      case 0:
        xpbdStep(dt);
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
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

  function resetImpl()
  {
    body1 = makeBody(5.0, 60, 20, 20, vec3.fromValues(0, 0, 0), vec3.fromValues(15, 5, 5));
    body2 = makeBody(5.0, 60, 20, 20, vec3.fromValues(0, 0, 60), vec3.fromValues(15, 5, -5));
    lambda = 0;
    spring = makeForceSpring();

    p.camera(150, 75, 150, 0, 0, 0, 0, -1, 0);
  }

  function xpbdStep(dt)
  {
    const subDt = dt / subSteps;
    for (let step = 0; step < subSteps; step++)
    {
      prePosSolve(body1, subDt);
      prePosSolve(body2, subDt);

      solvePosConstraints(subDt);

      postPosSolve(body1, subDt);
      postPosSolve(body2, subDt);
    }

    mat4.fromRotationTranslation(
      body1.currentTransformMatrix, body1.currentQuat, body1.currentWorldPos);
    mat4.fromRotationTranslation(
      body2.currentTransformMatrix, body2.currentQuat, body2.currentWorldPos);
  }

  function prePosSolve(body, dt)
  {
    vec3.copy(body.prevWorldPos, body.currentWorldPos);

    vec3.scaleAndAdd(body.currentWorldPos, body.currentWorldPos, body.currentVelocity, dt);

    quat.copy(body.prevQuat, body.currentQuat);

    updateWorldInertia(body);
    const worldInertiaInv = mat3.create();
    mat3.invert(worldInertiaInv, body.worldInertiaTensor);

    const Jw = vec3.create();
    const temp = vec3.create();
    const gyroTerm = vec3.create();
    vec3.transformMat3(Jw, body.currentAngleSpeed, body.worldInertiaTensor);
    vec3.cross(temp, body.currentAngleSpeed, Jw);
    vec3.transformMat3(gyroTerm, temp, worldInertiaInv);
    vec3.scale(gyroTerm, gyroTerm, -dt);

    vec3.add(body.currentAngleSpeed, body.currentAngleSpeed, gyroTerm);

    const angleQuat = quat.fromValues(
      0.5 * dt * body.currentAngleSpeed[0],
      0.5 * dt * body.currentAngleSpeed[1],
      0.5 * dt * body.currentAngleSpeed[2],
      0);
    const finalAddQuat = quat.create();
    quat.multiply(finalAddQuat, angleQuat, body.currentQuat);

    quat.add(body.currentQuat, body.currentQuat, finalAddQuat);
    quat.normalize(body.currentQuat, body.currentQuat);
  }

  function solvePosConstraints(dt)
  {
    const worldConnectionPosBody1 = vec3.create();
    vec3.transformMat4(worldConnectionPosBody1, body1.connectionPos, body1.currentTransformMatrix);
    const worldConnectionPosBody2 = vec3.create();
    vec3.transformMat4(worldConnectionPosBody2, body2.connectionPos, body2.currentTransformMatrix);

    const firstBodyParams = getSpringCurrentParams(body1, worldConnectionPosBody2);
    const secondBodyParams = getSpringCurrentParams(body2, worldConnectionPosBody1);
    // stretch should be the same
    const complianceScaled = spring.compliance / (dt * dt);
    const deltaLambda = (-firstBodyParams.stretch - complianceScaled * lambda) /
      (firstBodyParams.invEffMass + secondBodyParams.invEffMass + complianceScaled);
    lambda += deltaLambda;

    const impulse = vec3.clone(firstBodyParams.vecToSpringPosNorm);
    vec3.scale(impulse, impulse, deltaLambda);

    updatePosAndQuatFromConstraint(body1, firstBodyParams, impulse, -1);
    updatePosAndQuatFromConstraint(body2, secondBodyParams, impulse, 1);
  }

  function updatePosAndQuatFromConstraint(body, params, impulse, sign)
  {
    vec3.scaleAndAdd(body.currentWorldPos, body.currentWorldPos, impulse, sign * 1.0 / body.mass);

    const temp = vec3.create();
    const rCrossP = vec3.create();
    vec3.cross(temp, params.vecToConnectionPos, impulse);
    vec3.transformMat3(rCrossP, temp, params.worldInertiaInv);

    const angleQuat = quat.fromValues(0.5 * rCrossP[0], 0.5 * rCrossP[1], 0.5 * rCrossP[2], 0);
    const finalAddQuat = quat.create();
    quat.multiply(finalAddQuat, angleQuat, body.currentQuat);

    if (sign < 0)
    {
      quat.scale(finalAddQuat, finalAddQuat, -1);
    }
    quat.add(body.currentQuat, body.currentQuat, finalAddQuat);
    quat.normalize(body.currentQuat, body.currentQuat);
  }

  function postPosSolve(body, dt)
  {
    vec3.subtract(body.currentVelocity, body.currentWorldPos, body.prevWorldPos);
    vec3.scale(body.currentVelocity, body.currentVelocity, 1 / dt);
    const deltaQuat = quat.create();
    const prevQuatInv = quat.create();
    quat.invert(prevQuatInv, body.prevQuat);
    quat.multiply(deltaQuat, body.currentQuat, prevQuatInv);

    body.currentAngleSpeed = vec3.fromValues(deltaQuat[0], deltaQuat[1], deltaQuat[2]);
    vec3.scale(body.currentAngleSpeed, body.currentAngleSpeed, 2 / dt);

    if (deltaQuat[3] < 0)
    {
      vec3.scale(body.currentAngleSpeed, -1);
    }
  }

  function updateWorldInertia(body)
  {
    const rotMatrix = mat3.create();
    const rotMatrixTransposed = mat3.create();
    mat3.fromQuat(rotMatrix, body.currentQuat);
    mat3.transpose(rotMatrixTransposed, rotMatrix);

    const temp = mat3.create();

    mat3.multiply(temp, body.localInertiaTensor, rotMatrixTransposed);
    mat3.multiply(body.worldInertiaTensor, rotMatrix, temp);
  }

  function getSpringCurrentParams(body, otherConPos)
  {
    const worldInertiaInv = mat3.create()

    mat3.invert(worldInertiaInv, body.worldInertiaTensor);

    const worldConnectionPos = vec3.create();
    vec3.transformMat4(worldConnectionPos, body.connectionPos, body.currentTransformMatrix);
    const vecToConnectionPos = vec3.create();
    vec3.subtract(vecToConnectionPos, worldConnectionPos, body.currentWorldPos);

    const vecToSpringPos = vec3.create();
    vec3.subtract(vecToSpringPos, otherConPos, worldConnectionPos);

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
    const invEffMass = body.invMass + vec3.dot(rCrossN, tempVec);

    return {
      invEffMass : invEffMass,
      stretch : stretch,
      worldInertiaInv : worldInertiaInv,
      vecToConnectionPos : vecToConnectionPos,
      vecToSpringPosNorm : vecToSpringPosNorm,
    };
  }

  function drawTask()
  {
    const worldConnectionPosBody1 = vec3.create();
    vec3.transformMat4(worldConnectionPosBody1, body1.connectionPos, body1.currentTransformMatrix);
    const worldConnectionPosBody2 = vec3.create();
    vec3.transformMat4(worldConnectionPosBody2, body2.connectionPos, body2.currentTransformMatrix);

    p.push();
    p.applyMatrix([...body1.currentTransformMatrix ]);
    p.ambientMaterial(65, 130, 255);
    p.box(body1.width, body1.height, body1.depth);
    p.pop();
    p.push();
    p.applyMatrix([...body2.currentTransformMatrix ]);
    p.ambientMaterial(65, 130, 255);
    p.box(body2.width, body2.height, body2.depth);
    p.pop();

    drawSpring(worldConnectionPosBody1, worldConnectionPosBody2);

    drawCoordAxis();
  }

  function drawSpring(worldConnectionPosBody1, worldConnectionPosBody2)
  {
    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
    p.push();
    p.stroke(220, 150, 50, 150);
    p.strokeWeight(3);
    p.line(...worldConnectionPosBody1, ...worldConnectionPosBody2);
    p.pop();

    p.push();
    p.translate(...worldConnectionPosBody1);
    p.noStroke();
    p.ambientMaterial(
      200,
      200,
      50,
    );
    p.sphere(2, 10, 10);
    p.pop();

    p.push();
    p.translate(...worldConnectionPosBody2);
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

  function drawCoordAxis(body)
  {
    p.drawingContext.disable(p.drawingContext.DEPTH_TEST);
    p.push();
    p.applyMatrix([...body1.currentTransformMatrix ]);
    p.strokeWeight(2);
    p.stroke(255, 50, 50, 100);
    p.line(0, 0, 0, 25, 0, 0);
    p.stroke(50, 255, 50, 100);
    p.line(0, 0, 0, 0, 25, 0);
    p.stroke(50, 50, 255, 100);
    p.line(0, 0, 0, 0, 0, 25);
    p.pop();
    p.push();
    p.applyMatrix([...body2.currentTransformMatrix ]);
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
    p.text("Part 2.2: Two rigid bodies", 32, 30);
    p.text(`Mode: ${sim_modes[currentMode].label}`, 32, 56);
    p.text("Press R to reset. T to stop/continue simulation", 32, 82);
    p.text(
      `Body1 position x:${body1.currentWorldPos[0].toFixed(4)}, y:${
        body1.currentWorldPos[1].toFixed(4)}, z:${body1.currentWorldPos[2].toFixed(4)}`,
      32,
      144);
    p.text(
      `Body2 position x:${body2.currentWorldPos[0].toFixed(4)}, y:${
        body2.currentWorldPos[1].toFixed(4)}, z:${body2.currentWorldPos[2].toFixed(4)}`,
      32,
      168);
    p.pop();
  }
}

function makeBody(mass, width, height, depth, worldPos, conPos)
{
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

  const initialQuat = quat.create();
  const nextQuat = quat.clone(initialQuat);

  const transform = mat4.create();
  mat4.fromRotationTranslation(transform, initialQuat, worldPos);

  const worldInertia = mat3.create();

  const rotMatrix = mat3.create();
  const rotMatrixTransposed = mat3.create();
  mat3.fromQuat(rotMatrix, initialQuat);
  mat3.transpose(rotMatrixTransposed, rotMatrix);

  const temp = mat3.create();

  mat3.multiply(temp, inertia, rotMatrixTransposed);
  mat3.multiply(worldInertia, rotMatrix, temp);

  const initialAngularMomentum = vec3.create();
  vec3.transformMat3(initialAngularMomentum, currentAngleSpeed, worldInertia);

  return {
    mass : mass,
    invMass : 1.0 / mass,
    width : width,
    height : height,
    depth : depth,
    prevWorldPos : worldPos,
    currentWorldPos : vec3.clone(worldPos),
    currentVelocity : vec3.fromValues(0, 0, 0),
    connectionPos : conPos,
    localInertiaTensor : inertia,
    worldInertiaTensor : worldInertia,
    currentAngularMomentum : initialAngularMomentum,
    prevQuat : initialQuat,
    currentQuat : nextQuat,
    currentAngleSpeed : currentAngleSpeed,
    currentTransformMatrix : transform,
  };
}

function makeForceSpring()
{
  return {
    restLength : 30,
    stiffness : 0.5,
    compliance : 0.001,
    dampingLin : 0.7,
    dampingAng : 0.3,
  };
}

import * as mat3 from "../libraries/esm/mat3.js";
import * as mat4 from "../libraries/esm/mat4.js";
import * as quat from "../libraries/esm/quat.js";
import * as vec3 from "../libraries/esm/vec3.js";

const sim_modes = [
  {label : "2.2.1)  Connected with spring (Sequential impulses with baumgarte stabilization)"},
  {label : "2.2.2)  Connected with spring (Sequential impulses with Nonlinear Gauss-Seidel)"},
  {label : "2.2.3)  Connected with spring (Sequential impulses with budda string)"},
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

export function createPart2_3(p)
{
  let body1 = {
    mass : 0,
    invMass : 0,
    width : 0,
    height : 0,
    depth : 0,
    currentWorldPos : vec3.create(),
    nextWorldPos : vec3.create(),
    currentVelocity : vec3.create(),
    nextVelocity : vec3.create(),
    connectionPos : vec3.create(),
    localInertiaTensor : mat3.create(),
    worldInertiaTensor : mat3.create(),
    currentAngularMomentum : vec3.create(),
    currentQuat : quat.create(),
    nextQuat : quat.create(),
    currentAngleSpeed : vec3.create(),
    nextAngleSpeed : vec3.create(),
    currentTransformMatrix : mat4.create(),
    frameInfo : {
      force : vec3.create(),
      torque : vec3.create(),
    },
  };
  let body2 = {
    mass : 0,
    invMass : 0,
    width : 0,
    height : 0,
    depth : 0,
    currentWorldPos : vec3.create(),
    nextWorldPos : vec3.create(),
    currentVelocity : vec3.create(),
    nextVelocity : vec3.create(),
    connectionPos : vec3.create(),
    localInertiaTensor : mat3.create(),
    worldInertiaTensor : mat3.create(),
    currentAngularMomentum : vec3.create(),
    currentQuat : quat.create(),
    nextQuat : quat.create(),
    currentAngleSpeed : vec3.create(),
    nextAngleSpeed : vec3.create(),
    currentTransformMatrix : mat4.create(),
    frameInfo : {
      force : vec3.create(),
      torque : vec3.create(),
    },
  };
  let spring = {
    worldPos : vec3.fromValues(0, 0, 80),
    restLength : 30,
    baumgarte_betta : 0.1,
    budda_hzFreq : 0.1,
    budda_damping : 3,
  };
  let totalLambda = 0;
  let stopSim = true;
  let subSteps = 5;
  let currentMode = 0;

  return {
    init() { resetImpl(); },

    reset() { resetImpl(); },

    update(dt) {
      if (stopSim)
      {
        return;
      }

      const subDt = dt / subSteps;
      for (let step = 0; step < subSteps; step++)
      {
        preSolve(body1);
        preSolve(body2);

        const worldConnectionPosBody1 = vec3.create();
        vec3.transformMat4(
          worldConnectionPosBody1, body1.connectionPos, body1.currentTransformMatrix);
        const worldConnectionPosBody2 = vec3.create();
        vec3.transformMat4(
          worldConnectionPosBody2, body2.connectionPos, body2.currentTransformMatrix);

        const firstBodyParams = getSpringCurrentParams(body1, worldConnectionPosBody2);
        const secondBodyParams = getSpringCurrentParams(body2, worldConnectionPosBody1);

        const effMass = 1.0 / (firstBodyParams.invEffMass + secondBodyParams.invEffMass);
        const Jv = -vec3.dot(firstBodyParams.vecToSpringPosNorm, body1.currentVelocity) -
          vec3.dot(firstBodyParams.rCrossN, body1.currentAngleSpeed) +
          vec3.dot(secondBodyParams.vecToSpringPosNorm, body2.currentVelocity) +
          vec3.dot(secondBodyParams.rCrossN, body2.currentAngleSpeed);

        let lambda;
        switch (currentMode)
        {
        case 0:
          lambda = getBaumgarteLambda(effMass, firstBodyParams.stretch, Jv, subDt);
          break;
        case 1:
          lambda = getNGSLambda(effMass, firstBodyParams.stretch, Jv, subDt);
          break;
        case 2:
          lambda = getBuddaLambda(effMass, firstBodyParams.stretch, Jv, subDt);
          break;
        }

        postSolve(body1, firstBodyParams, lambda, subDt);
        postSolve(body2, secondBodyParams, lambda, subDt);
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
    body1 = makeBody(1.0, 60, 20, 20, vec3.fromValues(0, 0, 0), vec3.fromValues(15, 5, 5));
    body2 = makeBody(1.0, 60, 20, 20, vec3.fromValues(0, 0, 60), vec3.fromValues(15, 5, -5));
    totalLambda = 0;
    spring = makeSpring();

    p.camera(150, 75, 150, 0, 0, 0, 0, -1, 0);
  }

  function preSolve(body)
  {
    vec3.copy(body.currentWorldPos, body.nextWorldPos);
    vec3.copy(body.currentVelocity, body.nextVelocity);
    vec3.copy(body.currentAngleSpeed, body.nextAngleSpeed);
    vec3.copy(body.currentQuat, body.nextQuat);

    updateWorldInertia(body);
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

  function getBaumgarteLambda(mass, stretch, Jv, dt)
  {
    const lambda = -(Jv + (spring.baumgarte_betta * stretch) / dt) / (mass);

    return lambda;
  }

  function getNGSLambda(mass, stretch, Jv, dt)
  {
    const lambda = -(stretch) / (mass);

    return lambda;
  }

  function getBuddaLambda(mass, stretch, Jv, dt)
  {
    const omega = 2.0 * Math.PI * spring.budda_hzFreq;
    const k = mass * omega * omega;
    const c = 2.0 * mass * spring.budda_damping * omega;
    const denom = c + dt * k;
    const beta = (dt * k) / denom;
    const gamma = 1.0 / denom;

    const lambda = -(Jv + (beta * stretch) / dt) / (gamma);

    return lambda;
  }

  function postSolve(body, params, lambda, dt)
  {
    const force = vec3.clone(params.vecToSpringPosNorm);
    vec3.scale(force, force, -lambda);
    const torque = vec3.clone(params.rCrossN);
    vec3.scale(torque, torque, -lambda);

    vec3.copy(body.frameInfo.force, force);
    vec3.copy(body.frameInfo.torque, torque);

    vec3.scale(force, force, body.invMass * dt);
    vec3.add(body.nextVelocity, body.currentVelocity, force);

    vec3.transformMat3(torque, torque, params.worldInertiaInv);
    vec3.scale(torque, torque, dt);
    vec3.add(body.nextAngleSpeed, body.currentAngleSpeed, torque);
    vec3.add(body.nextAngleSpeed, body.nextAngleSpeed, getGyroTerm(body, dt));

    vec3.add(
      body.nextWorldPos, body.currentWorldPos, vec3.scale(vec3.create(), body.nextVelocity, dt));

    const finalAddQuat = quat.create();
    const angleQuat = quat.fromValues(
      0.5 * dt * body.nextAngleSpeed[0],
      0.5 * dt * body.nextAngleSpeed[1],
      0.5 * dt * body.nextAngleSpeed[2],
      0);

    // for nonlinear gauss seidel
    if (currentMode === 1)
    {
      const temp = quat.create();
      quat.exp(temp, angleQuat);
      quat.multiply(finalAddQuat, temp, body.currentQuat);

      quat.add(body.nextQuat, body.currentQuat, finalAddQuat);
    }
    else
    {
      quat.multiply(finalAddQuat, angleQuat, body.currentQuat);

      quat.add(body.nextQuat, body.currentQuat, finalAddQuat);
    }
    quat.normalize(body.nextQuat, body.nextQuat);

    mat4.fromRotationTranslation(body.currentTransformMatrix, body.nextQuat, body.nextWorldPos);
  }

  function getGyroTerm(body, dt)
  {
    const Jw = vec3.create();
    const gyroTerm = vec3.create();

    const worldInertiaInv = mat3.create();
    mat3.invert(worldInertiaInv, body.worldInertiaTensor);

    vec3.transformMat3(Jw, body.currentAngleSpeed, worldInertiaInv);
    vec3.cross(gyroTerm, Jw, body.currentAngleSpeed);
    vec3.scale(gyroTerm, gyroTerm, dt);

    const skewedOmega = skew(body.currentAngleSpeed);
    const skewedJw = skew(Jw);

    const skewedOmegaTimesJ = mat3.create();
    mat3.multiply(skewedOmegaTimesJ, skewedOmega, body.worldInertiaTensor);

    const yakobi = mat3.create();
    mat3.subtract(yakobi, skewedOmegaTimesJ, skewedJw);
    mat3.multiplyScalar(yakobi, yakobi, dt);
    mat3.add(yakobi, yakobi, body.worldInertiaTensor);

    const yakobiInv = mat3.create();
    mat3.invert(yakobiInv, yakobi);

    const angleCorrection = vec3.create();
    vec3.transformMat3(angleCorrection, gyroTerm, yakobiInv);

    return angleCorrection;
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

    drawPlane(500, 0, -30, 0);

    drawSpring(worldConnectionPosBody1, worldConnectionPosBody2);

    drawForces(body1, worldConnectionPosBody1);
    drawForces(body2, worldConnectionPosBody2);

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

  function drawForces(body, pos)
  {
    const shiftedForce = vec3.clone(body.frameInfo.force);
    const shiftedTorque = vec3.clone(body.frameInfo.torque);

    vec3.add(shiftedForce, shiftedForce, pos);
    vec3.add(shiftedTorque, shiftedTorque, pos);

    drawArrow(255, 0, 0, ...pos, ...shiftedForce);
    drawArrow(0, 0, 255, ...pos, ...shiftedTorque);
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
    p.rect(16, 16, 600, 250, 8);
    p.fill(229, 231, 235);
    p.textSize(14);
    p.textAlign(p.LEFT, p.TOP);
    p.text("Part 2.2: Two rigid bodies", 32, 30);
    p.text(`Mode: ${sim_modes[currentMode].label}`, 32, 56);
    p.text("Press M to switch mode, R to reset. T to stop/continue simulation", 32, 82);
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
    p.fill(250, 50, 50);
    p.text(`Red arrow - Current Force`, 32, 210);
    p.fill(50, 150, 250);
    p.text(`Blue Arrow - Current Torque`, 32, 230);
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
    currentWorldPos : worldPos,
    nextWorldPos : vec3.clone(worldPos),
    currentVelocity : vec3.fromValues(0, 0, 0),
    nextVelocity : vec3.fromValues(0, 0, 0),
    connectionPos : conPos,
    localInertiaTensor : inertia,
    worldInertiaTensor : worldInertia,
    currentAngularMomentum : initialAngularMomentum,
    currentQuat : initialQuat,
    nextQuat : nextQuat,
    currentAngleSpeed : currentAngleSpeed,
    nextAngleSpeed : vec3.clone(currentAngleSpeed),
    currentTransformMatrix : transform,
    frameInfo : {
      force : vec3.create(),
      torque : vec3.create(),
    },
  };
}

function makeSpring()
{
  return {
    worldPos : vec3.fromValues(0, 0, 80),
    restLength : 30,
    baumgarte_betta : 0.01,
    budda_hzFreq : 0.1,
    budda_damping : 3,
  };
}

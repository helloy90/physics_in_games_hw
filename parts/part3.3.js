import * as mat3 from "../libraries/esm/mat3.js";
import * as mat4 from "../libraries/esm/mat4.js";
import * as quat from "../libraries/esm/quat.js";
import * as vec3 from "../libraries/esm/vec3.js";

const sim_modes = [
  {label : "1) Sequential Impulses"},
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

export function createPart3_2(p)
{
  let bodies = [];
  let contacts = [];

  const baumgarteBetta = 0.3;

  const extAcceleration = vec3.fromValues(0, -20, 0);
  const maxDeltaLambda = 99999999;

  let stopSim = true;
  let subSteps = 10;
  let currentMode = 0;

  return {
    init() { resetImpl(); },

    reset() { resetImpl(); },

    update(dt) {
      if (stopSim)
      {
        return;
      }

      seqImpulseStep(dt);
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
    let positions = [
      vec3.fromValues(0, -40, 0), // plane
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 30, -3),
      vec3.fromValues(-3, 60, 0),
      vec3.fromValues(10, 30, 30),
      vec3.fromValues(12, 60, 30),
      vec3.fromValues(-20, 90, 60),
      vec3.fromValues(20, 60, 60),
      vec3.fromValues(30, 90, 60),
      vec3.fromValues(0, 120, 5),
    ];

    let rotations = [
      quat.create(), // plane
      quat.create(),
      quat.create(),
      quat.create(),
      quat.create(),
      quat.create(),
      quat.create(),
      quat.create(),
      quat.create(),
      quat.create(),
    ];

    quat.fromEuler(rotations[0], 0, 0, 0);
    quat.fromEuler(rotations[1], 0, 0, 0);
    quat.fromEuler(rotations[2], 0, 30, 0);
    quat.fromEuler(rotations[3], 0, 60, 0);
    quat.fromEuler(rotations[4], 0, 30, 0);
    quat.fromEuler(rotations[5], 0, 60, 0);
    quat.fromEuler(rotations[6], 0, 90, 0);
    quat.fromEuler(rotations[7], 0, 60, 0);
    quat.fromEuler(rotations[8], 0, 30, 0);
    quat.fromEuler(rotations[9], 0, 120, 0);

    bodies = [];
    contacts = [];

    bodies.push(makeBody(0, 500, 20, 500, positions[0], rotations[0], true)); // plane

    for (let i = 1; i < 10; i++)
    {
      bodies.push(makeBody(5, 30, 20, 20, positions[i], rotations[i], false));
    }

    p.camera(300, 150, 300, 0, 0, 0, 0, -1, 0);
  }

  function seqImpulseStep(dt)
  {
    const subDt = dt / subSteps;

    contacts = collectContacts();

    for (let step = 0; step < subSteps; step++)
    {
      for (const body of bodies)
      {
        prePosSolve(body, subDt);
      }

      for (const contact of contacts)
      {
        solvePosConstraints(contact, subDt);
      }

      for (const body of bodies)
      {
        postPosSolve(body, subDt);
      }
    }
  }

  function collectContacts()
  {
    const currentContacts = [];

    for (let i = 0; i < bodies.length; i++)
    {
      for (let j = i + 1; j < bodies.length; j++)
      {
        if (bodies[i].isStatic && bodies[j].isStatic)
          continue;

        const obbContacts = computeOBBContacts(bodies[i], bodies[j]); // sat
        for (const c of obbContacts)
        {
          currentContacts.push(c);
        }
      }
    }
    return currentContacts;
  }

  function computeOBBContacts(bodyA, bodyB)
  {
    const axisA = getBoxAxis(bodyA);
    const axisB = getBoxAxis(bodyB);
    const halfA = vec3.fromValues(bodyA.width / 2, bodyA.height / 2, bodyA.depth / 2);
    const halfB = vec3.fromValues(bodyB.width / 2, bodyB.height / 2, bodyB.depth / 2);
    const centerA = bodyA.currentWorldPos;
    const centerB = bodyB.currentWorldPos;
    const fromAtoB = vec3.create();
    vec3.subtract(fromAtoB, centerB, centerA);

    const R = mat3.create();
    const axisAMat = fromColumns(axisA[0], axisA[1], axisA[2]);
    const axisAMatTransposed = mat3.create();
    mat3.transpose(axisAMatTransposed, axisAMat);
    const axisBMat = fromColumns(axisB[0], axisB[1], axisB[2]);

    mat3.multiply(R, axisAMatTransposed, axisBMat);

    const tA = vec3.create();
    vec3.transformMat3(tA, fromAtoB, axisAMatTransposed);

    let minOverlap = Infinity;
    let bestAxis = vec3.create();

    for (let i = 0; i < 3; i++)
    {
      const axis = axisA[i];
      const projA = halfA[i];
      const projB = Math.abs(R[0 + 3 * i]) * halfB[0] + Math.abs(R[1 + 3 * i]) * halfB[1] +
        Math.abs(R[2 + 3 * i]) * halfB[2];
      const d = Math.abs(vec3.dot(fromAtoB, axis));
      const overlap = projA + projB - d;
      if (overlap <= 0)
        return [];
      if (overlap < minOverlap)
      {
        minOverlap = overlap;
        vec3.copy(bestAxis, axis);
      }
    }

    for (let i = 0; i < 3; i++)
    {
      const axis = axisB[i];
      const projA = Math.abs(R[0 + i]) * halfA[0] + Math.abs(R[3 + i]) * halfA[1] +
        Math.abs(R[6 + i]) * halfA[2];
      const projB = halfB[i];
      const d = Math.abs(vec3.dot(fromAtoB, axis));
      const overlap = projA + projB - d;
      if (overlap <= 0)
        return [];
      if (overlap < minOverlap)
      {
        minOverlap = overlap;
        vec3.copy(bestAxis, axis);
      }
    }

    for (let i = 0; i < 3; i++)
    {
      for (let j = 0; j < 3; j++)
      {
        const axis = vec3.create();
        vec3.cross(axis, axisA[i], axisB[j]);
        const lenSq = vec3.squaredLength(axis);
        if (lenSq < 1e-8)
          continue;
        vec3.scale(axis, axis, 1 / Math.sqrt(lenSq));

        let projA = 0;
        for (let k = 0; k < 3; k++)
        {
          projA += halfA[k] * Math.abs(vec3.dot(axis, axisA[k]));
        }

        let projB = 0;
        for (let k = 0; k < 3; k++)
        {
          projB += halfB[k] * Math.abs(vec3.dot(axis, axisB[k]));
        }
        const d = Math.abs(vec3.dot(fromAtoB, axis));
        const overlap = projA + projB - d;
        if (overlap <= 0)
          return [];
        if (overlap < minOverlap)
        {
          minOverlap = overlap;
          vec3.copy(bestAxis, axis);
        }
      }
    }

    const dotTA = vec3.dot(vec3.subtract(vec3.create(), centerA, centerB), bestAxis);
    if (dotTA < 0)
    {
      vec3.negate(bestAxis, bestAxis);
    }

    let refBody, incBody;
    let refAxes, incAxes;
    let refHalf, incHalf;
    let refNormalLocal;
    let flip;

    const absDotA = [
      Math.abs(vec3.dot(bestAxis, axisA[0])),
      Math.abs(vec3.dot(bestAxis, axisA[1])),
      Math.abs(vec3.dot(bestAxis, axisA[2])),
    ];
    const absDotB = [
      Math.abs(vec3.dot(bestAxis, axisB[0])),
      Math.abs(vec3.dot(bestAxis, axisB[1])),
      Math.abs(vec3.dot(bestAxis, axisB[2])),
    ];
    const maxA = Math.max(absDotA[0], absDotA[1], absDotA[2]);
    const maxB = Math.max(absDotB[0], absDotB[1], absDotB[2]);

    if (maxA >= maxB)
    {
      refBody = bodyA;
      incBody = bodyB;
      refAxes = axisA;
      incAxes = axisB;
      refHalf = halfA;
      incHalf = halfB;
      refNormalLocal = absDotA.indexOf(maxA);
      flip = false;
    }
    else
    {
      refBody = bodyB;
      incBody = bodyA;
      refAxes = axisB;
      incAxes = axisA;
      refHalf = halfB;
      incHalf = halfA;
      refNormalLocal = absDotB.indexOf(maxB);
      flip = true;
    }

    const refNormal = vec3.clone(refAxes[refNormalLocal]);

    const refToInc = vec3.subtract(vec3.create(), incBody.currentWorldPos, refBody.currentWorldPos);
    if (vec3.dot(refNormal, refToInc) < 0)
    {
      vec3.negate(refNormal, refNormal);
    }

    let refTan1Idx = (refNormalLocal + 1) % 3;
    let refTan2Idx = (refNormalLocal + 2) % 3;
    const refTan1 = refAxes[refTan1Idx];
    const refTan2 = refAxes[refTan2Idx];
    const refCenter = vec3.copy(vec3.create(), refBody.currentWorldPos);
    vec3.scaleAndAdd(refCenter, refCenter, refNormal, vec3.dot(refNormal, refHalf));

    let incNormalLocal = 0;
    let maxDot = -Infinity;
    for (let i = 0; i < 3; i++)
    {
      const d = vec3.dot(incAxes[i], refNormal);
      if (Math.abs(d) > maxDot)
      {
        maxDot = Math.abs(d);
        incNormalLocal = i;
      }
    }

    let incNormal = vec3.clone(incAxes[incNormalLocal]);
    if (vec3.dot(incNormal, refNormal) > 0)
    {
      vec3.negate(incNormal, incNormal);
    }

    let incTan1Idx = (incNormalLocal + 1) % 3;
    let incTan2Idx = (incNormalLocal + 2) % 3;
    const incTan1 = incAxes[incTan1Idx];
    const incTan2 = incAxes[incTan2Idx];

    const incVerticesLocal = [ [ 1, 1 ], [ 1, -1 ], [ -1, -1 ], [ -1, 1 ] ];
    let incWorldVerts = [];
    for (const v of incVerticesLocal)
    {
      const pt = vec3.copy(vec3.create(), incBody.currentWorldPos);
      vec3.scaleAndAdd(pt, pt, incNormal, vec3.dot(incNormal, incHalf));
      vec3.scaleAndAdd(pt, pt, incTan1, v[0] * incHalf[incTan1Idx]);
      vec3.scaleAndAdd(pt, pt, incTan2, v[1] * incHalf[incTan2Idx]);
      incWorldVerts.push(pt);
    }

    const clipPlanes = [
      {normal : vec3.clone(refTan1), dist : vec3.dot(refTan1, refCenter) + refHalf[refTan1Idx]},
      {
        normal : vec3.negate(vec3.create(), refTan1),
        dist : -vec3.dot(refTan1, refCenter) + refHalf[refTan1Idx],
      },
      {normal : vec3.clone(refTan2), dist : vec3.dot(refTan2, refCenter) + refHalf[refTan2Idx]},
      {
        normal : vec3.negate(vec3.create(), refTan2),
        dist : -vec3.dot(refTan2, refCenter) + refHalf[refTan2Idx],
      },
    ];

    let clippedVerts = incWorldVerts;
    for (const plane of clipPlanes)
    {
      const newVerts = [];
      for (let i = 0; i < clippedVerts.length; i++)
      {
        const v1 = clippedVerts[i];
        const v2 = clippedVerts[(i + 1) % clippedVerts.length];
        const d1 = vec3.dot(v1, plane.normal) - plane.dist;
        const d2 = vec3.dot(v2, plane.normal) - plane.dist;
        if (d1 <= 0)
          newVerts.push(vec3.clone(v1));
        if (d1 * d2 < 0)
        {
          const t = d1 / (d1 - d2);
          const newPt = vec3.lerp(vec3.create(), v1, v2, t);
          newVerts.push(newPt);
        }
      }
      if (newVerts.length < 3)
        break;
      clippedVerts = newVerts;
    }

    let currentContacts = [];
    for (const pt of clippedVerts)
    {
      const penetration =
        vec3.dot(refNormal, vec3.subtract(vec3.create(), pt, refCenter)) + refHalf[refNormalLocal];
      const colliderContact =
        vec3.subtract(vec3.create(), pt, vec3.scale(vec3.create(), refNormal, penetration));

      let worldNormal;
      if (!flip)
      {
        worldNormal = vec3.negate(vec3.create(), refNormal);
      }
      else
      {
        worldNormal = vec3.clone(refNormal);
      }

      const penMoveDir = vec3.clone(worldNormal);
      const colliderMoveDir = vec3.negate(vec3.create(), worldNormal);

      const penBody = flip ? incBody : refBody;
      const colliderBody = flip ? refBody : incBody;

      if (!flip)
      {
        currentContacts.push({
          penBody : penBody,
          penContact : vec3.clone(colliderContact),
          penMoveDir,
          colliderBody : colliderBody,
          colliderContact : vec3.clone(pt),
          colliderMoveDir,
          penetration,
          lambda : 0,
        });
      }
      else
      {
        currentContacts.push({
          penBody,
          penContact : vec3.clone(pt),
          penMoveDir,
          colliderBody,
          colliderContact : vec3.clone(colliderContact),
          colliderMoveDir,
          penetration,
          lambda : 0,
        });
      }
    }

    return currentContacts;
  }

  function fromColumns(c0, c1, c2)
  {
    const out = mat3.create();
    out[0] = c0[0];
    out[1] = c0[1];
    out[2] = c0[2];
    out[3] = c1[0];
    out[4] = c1[1];
    out[5] = c1[2];
    out[6] = c2[0];
    out[7] = c2[1];
    out[8] = c2[2];
    return out;
  };


  function getBoxAxis(body)
  {
    const rotMat = mat3.create();
    mat3.fromQuat(rotMat, body.currentQuat);

    return [
      vec3.fromValues(rotMat[0], rotMat[1], rotMat[2]),
      vec3.fromValues(rotMat[3], rotMat[4], rotMat[5]),
      vec3.fromValues(rotMat[6], rotMat[7], rotMat[8]),
    ];
  }

  function prePosSolve(body, dt)
  {
    if (body.isStatic === true)
    {
      return;
    }

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

    mat3.invert(body.worldInertiaInvTensor, body.worldInertiaTensor);
  }

  function solvePosConstraints(contact, dt)
  {
    const penBodyParams =
      getParamsForImpulse(contact.penBody, contact.penContact, contact.penMoveDir);

    const colliderBodyParams =
      getParamsForImpulse(contact.colliderBody, contact.colliderContact, contact.colliderMoveDir);

    const effMass = 1.0 / (penBodyParams.invEffMass + colliderBodyParams.invEffMass);
    const Jv = -vec3.dot(contact.penMoveDir, contact.penBody.currentVelocity) -
      vec3.dot(penBodyParams.rCrossN, contact.penBody.currentAngleSpeed) +
      vec3.dot(contact.colliderMoveDir, contact.colliderBody.currentVelocity) +
      vec3.dot(colliderBodyParams.rCrossN, contact.colliderBody.currentAngleSpeed);

    let oldLambda = contact.lambda;

    let deltaLambda = getBaumgarteLambda(effMass, contact.penetration, Jv, dt);
    let newLambda = oldLambda + deltaLambda;
    deltaLambda = newLambda - oldLambda;
    if (deltaLambda > maxDeltaLambda)
    {
      deltaLambda = maxDeltaLambda;
      newLambda = oldLambda + deltaLambda;
    }
    contact.lambda = newLambda;

    updatePosAndQuatFromConstraint(
      contact.penBody, contact.penMoveDir, penBodyParams.rCrossN, deltaLambda, dt);
    updatePosAndQuatFromConstraint(
      contact.colliderBody, contact.colliderMoveDir, colliderBodyParams.rCrossN, deltaLambda, dt);
  }

  function getBaumgarteLambda(mass, stretch, Jv, dt)
  {
    const lambda = -(Jv + (baumgarteBetta * stretch) / dt) / (mass);

    return lambda;
  }

  function getParamsForImpulse(body, contact_point, move_dir)
  {
    const r = vec3.create();
    vec3.subtract(r, contact_point, body.currentWorldPos);

    const rCrossN = vec3.create();
    vec3.cross(rCrossN, r, move_dir);

    const temp = vec3.create();
    vec3.transformMat3(temp, rCrossN, body.worldInertiaInvTensor);
    const invRotMass = body.mass > 0 ? vec3.dot(rCrossN, temp) : 0;
    const invEffMass = body.invMass + invRotMass;

    return {
      vecToContactPoint : r,
      rCrossN : rCrossN,
      invrotMass : invRotMass,
      invEffMass : invEffMass,
    };
  }

  function updatePosAndQuatFromConstraint(body, mode_dir, r_cross_n, lambda, dt)
  {
    if (body.isStatic === true)
    {
      return;
    }
    const force = vec3.clone(mode_dir);
    vec3.scale(force, force, -lambda);
    const torque = vec3.clone(r_cross_n);
    vec3.scale(torque, torque, -lambda);

    vec3.scaleAndAdd(body.nextVelocity, body.currentVelocity, force, body.invMass * dt);

    vec3.transformMat3(torque, torque, body.worldInertiaInvTensor);
    vec3.scale(torque, torque, dt);
    vec3.add(body.nextAngleSpeed, body.currentAngleSpeed, torque);
    vec3.add(body.nextAngleSpeed, body.nextAngleSpeed, getGyroTerm(body, dt));
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

  function postPosSolve(body, dt)
  {
    if (body.isStatic === true)
    {
      return;
    }

    vec3.scaleAndAdd(body.nextVelocity, body.nextVelocity, extAcceleration, dt);

    vec3.add(
      body.nextWorldPos, body.currentWorldPos, vec3.scale(vec3.create(), body.nextVelocity, dt));

    const finalAddQuat = quat.create();
    const angleQuat = quat.fromValues(
      0.5 * dt * body.nextAngleSpeed[0],
      0.5 * dt * body.nextAngleSpeed[1],
      0.5 * dt * body.nextAngleSpeed[2],
      0);

    quat.multiply(finalAddQuat, angleQuat, body.currentQuat);

    quat.add(body.nextQuat, body.currentQuat, finalAddQuat);
    quat.normalize(body.nextQuat, body.nextQuat);

    mat4.fromRotationTranslation(body.currentTransformMatrix, body.nextQuat, body.nextWorldPos);
    mat4.invert(body.invCurrentTransformMatrix, body.currentTransformMatrix);
  }

  function drawTask()
  {
    for (const body of bodies)
    {
      p.push();
      p.applyMatrix([...body.currentTransformMatrix ]);
      p.ambientMaterial(65, 130, 255);
      p.box(body.width, body.height, body.depth);
      p.pop();

      drawCoordAxis(body);
    }
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


  function drawOverlay()
  {
    p.push();
    p.noStroke();
    p.fill(40, 40, 42, 220);
    p.translate(-p.width / 2, -p.height / 2);
    p.rect(16, 16, 500, 200, 8);
    p.fill(229, 231, 235);
    p.textSize(14);
    p.textAlign(p.LEFT, p.TOP);
    p.text("Part 3: 1000 rigid bodies collision (Sequentinal Impulses)", 32, 30);
    p.text("Press R to reset. T to stop/continue simulation", 32, 56);
    p.pop();
  }
}

function makeBody(mass, width, height, depth, worldPos, initRotation, isStatic)
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

  const initialQuat = quat.clone(initRotation);
  const nextQuat = quat.clone(initialQuat);

  const transform = mat4.create();
  mat4.fromRotationTranslation(transform, initialQuat, worldPos);

  const invTransform = mat4.create();
  mat4.invert(invTransform, transform);

  const worldInertia = mat3.create();
  const worldInertiaInv = mat3.create();

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
    isStatic : isStatic,
    mass : mass,
    invMass : mass > 0 ? 1.0 / mass : 0,
    width : width,
    height : height,
    depth : depth,
    currentWorldPos : worldPos,
    nextWorldPos : vec3.clone(worldPos),
    currentVelocity : vec3.fromValues(0, 0, 0),
    nextVelocity : vec3.fromValues(0, 0, 0),
    localInertiaTensor : inertia,
    worldInertiaTensor : worldInertia,
    worldInertiaInvTensor : worldInertiaInv,
    currentAngularMomentum : initialAngularMomentum,
    currentQuat : initialQuat,
    nextQuat : nextQuat,
    currentAngleSpeed : currentAngleSpeed,
    nextAngleSpeed : vec3.clone(currentAngleSpeed),
    currentTransformMatrix : transform,
    invCurrentTransformMatrix : invTransform,

    localVertices : [
      [ -width / 2, -height / 2, -depth / 2 ],
      [ -width / 2, -height / 2, depth / 2 ],
      [ -width / 2, height / 2, -depth / 2 ],
      [ -width / 2, height / 2, depth / 2 ],
      [ width / 2, -height / 2, -depth / 2 ],
      [ width / 2, -height / 2, depth / 2 ],
      [ width / 2, height / 2, -depth / 2 ],
      [ width / 2, height / 2, depth / 2 ],
    ],
  };
}

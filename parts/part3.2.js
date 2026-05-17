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
  let renderEnclosure = false;

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
      if (key.toLowerCase() === "e")
      {
        renderEnclosure = !renderEnclosure;
      }
    },
  };

  function resetImpl()
  {
    bodies = [];
    contacts = [];

    let encThickness = 250;
    let encSize = 1200 - encThickness / 2;

    {
      let enclosure =
        [
          {pos : vec3.fromValues(0, -30 - encThickness / 2, 0), rot : quat.create()},
          {
            pos :
              vec3.fromValues(encSize / 2 - encThickness / 2, -30 + encSize / 2 - encThickness, 0),
            rot : quat.create(),
          },
          {
            pos :
              vec3.fromValues(0, -30 + encSize / 2 - encThickness, encSize / 2 - encThickness / 2),
            rot : quat.create(),
          },
          {
            pos :
              vec3.fromValues(-encSize / 2 + encThickness / 2, -30 + encSize / 2 - encThickness, 0),
            rot : quat.create(),
          },
          {
            pos :
              vec3.fromValues(0, -30 + encSize / 2 - encThickness, -encSize / 2 + encThickness / 2),
            rot : quat.create(),
          },
          {pos : vec3.fromValues(0, -30 + encSize - 1.5 * encThickness, 0), rot : quat.create()},
        ]

        let angles = [
          vec3.fromValues(0, 0, 0),
          vec3.fromValues(0, 0, 90),
          vec3.fromValues(90, 0, 0),
          vec3.fromValues(0, 0, 90),
          vec3.fromValues(90, 0, 0),
          vec3.fromValues(0, 0, 0),
        ];

      for (let i = 0; i < enclosure.length; i++)
      {
        quat.fromEuler(enclosure[i].rot, angles[i][0], angles[i][1], angles[i][2]);
        bodies.push(
          makeBody(0, encSize, encThickness, encSize, enclosure[i].pos, enclosure[i].rot, true));
      }
    }

    // bodies
    let positions = [
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 40),
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
      quat.create(),
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

    for (let i = 0; i < positions.length; i++)
    {
      bodies.push(makeBody(5, 30, 20, 20, positions[i], rotations[i], false));
    }

    p.camera(300, 150, 300, 0, 0, 0, 0, -1, 0);
    p.perspective();
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

        const obbContacts = satContacts(bodies[i], bodies[j]); // sat
        for (const c of obbContacts)
        {
          currentContacts.push(c);
        }
      }
    }
    return currentContacts;
  }

  function satContacts(bodyA, bodyB)
  {
    const axesA = getBoxAxis(bodyA);
    const halfSizeA = vec3.fromValues(bodyA.width / 2, bodyA.height / 2, bodyA.depth / 2);
    const axesB = getBoxAxis(bodyB);
    const halfSizeB = vec3.fromValues(bodyB.width / 2, bodyB.height / 2, bodyB.depth / 2);

    const fromAtoB = vec3.create();
    vec3.subtract(fromAtoB, bodyB.currentWorldPos, bodyA.currentWorldPos);

    let minOverlap = Infinity;
    let bestAxis = vec3.create();

    const tryAxis = (axis) => {
      const len = vec3.length(axis);
      if (len < 1e-9)
      {
        return true;
      }

      const currentAxis = vec3.clone(axis);
      vec3.scale(currentAxis, currentAxis, 1.0 / len);

      const projOnAxis = (axis, bodyAxes, halfSize) => {
        return halfSize[0] * Math.abs(vec3.dot(axis, bodyAxes[0])) +
          halfSize[1] * Math.abs(vec3.dot(axis, bodyAxes[1])) +
          halfSize[2] * Math.abs(vec3.dot(axis, bodyAxes[2]));
      };

      const projA = projOnAxis(currentAxis, axesA, halfSizeA);
      const projB = projOnAxis(currentAxis, axesB, halfSizeB);

      const signedDist = vec3.dot(fromAtoB, currentAxis);
      const dist = Math.abs(signedDist);
      const overlap = projA + projB - dist;
      if (overlap < 0)
      {
        return false;
      }

      if (overlap < minOverlap)
      {
        minOverlap = overlap;
        const sign = signedDist >= 0 ? 1 : -1;
        bestAxis = vec3.clone(currentAxis);
        vec3.scale(bestAxis, bestAxis, sign);
      }

      return true;
    };


    for (let i = 0; i < 3; i++)
    {
      if (!tryAxis(axesA[i]))
      {
        return [];
      }
    }

    for (let i = 0; i < 3; i++)
    {
      if (!tryAxis(axesB[i]))
      {
        return [];
      }
    }

    for (let i = 0; i < 3; i++)
    {
      for (let j = 0; j < 3; j++)
      {
        const axis = vec3.create();
        vec3.cross(axis, axesA[i], axesB[j]);
        if (!tryAxis(axis))
        {
          return [];
        }
      }
    }

    let currentContacts = [];

    const vertsA = getWorldVertices(bodyA);
    const vertsB = getWorldVertices(bodyB);

    const normAToB = vec3.clone(bestAxis);
    const normBToA = vec3.clone(bestAxis);
    vec3.scale(normBToA, normBToA, -1);

    for (const vert of vertsA)
    {
      if (pointInsideBoxCheck(vert, bodyB, halfSizeB))
      {
        const secondContact = vec3.create();
        vec3.scaleAndAdd(secondContact, vert, normBToA, minOverlap);

        currentContacts.push({
          firstBody : bodyA,
          firstContact : vert,
          firstMoveDir : vec3.clone(normBToA),
          secondBody : bodyB,
          secondContact : secondContact,
          secondMoveDir : vec3.clone(normAToB),
          penetration : minOverlap,
          lambda : 0,
        });
      }
    }

    for (const vert of vertsB)
    {
      if (pointInsideBoxCheck(vert, bodyA, halfSizeA))
      {
        const secondContact = vec3.create();
        vec3.scaleAndAdd(secondContact, vert, normAToB, minOverlap);

        currentContacts.push({
          firstBody : bodyB,
          firstContact : vert,
          firstMoveDir : vec3.clone(normAToB),
          secondBody : bodyA,
          secondContact : secondContact,
          secondMoveDir : vec3.clone(normBToA),
          penetration : minOverlap,
          lambda : 0,
        });
      }
    }

    if (currentContacts.length === 0)
    {
      const firstContact = vec3.clone(normAToB);
      vec3.scale(firstContact, firstContact, halfSizeA[0]);
      vec3.transformMat4(firstContact, firstContact, bodyA.currentTransformMatrix);

      const secondContact = vec3.clone(normBToA);
      vec3.scale(secondContact, secondContact, halfSizeB[0]);
      vec3.transformMat4(secondContact, secondContact, bodyB.currentTransformMatrix);

      currentContacts.push({
        firstBody : bodyA,
        firstContact : firstContact,
        firstMoveDir : vec3.clone(normBToA),
        secondBody : bodyB,
        secondContact : secondContact,
        secondMoveDir : vec3.clone(normAToB),
        penetration : minOverlap,
        lambda : 0,
      });
    }

    return currentContacts;
  }

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

  function getWorldVertices(body)
  {
    return body.localVertices.map(v => {
      const worldVert = vec3.create();
      vec3.transformMat4(worldVert, v, body.currentTransformMatrix);
      return worldVert;
    });
  }

  function pointInsideBoxCheck(
    vert,
    body,
    halfSize,
  )
  {
    const localVert = vec3.create();
    vec3.transformMat4(localVert, vert, body.invCurrentTransformMatrix);
    const absLocalVert = vec3.create();
    for (let i = 0; i < 3; i++)
    {
      absLocalVert[i] = Math.abs(localVert[i]);
    }

    const dists = vec3.create();

    vec3.subtract(dists, halfSize, absLocalVert);

    return dists[0] >= 0 && dists[1] >= 0 && dists[2] >= 0;
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
    const firstBodyParams =
      getParamsForImpulse(contact.firstBody, contact.firstContact, contact.firstMoveDir);

    const secondBodyParams =
      getParamsForImpulse(contact.secondBody, contact.secondContact, contact.secondMoveDir);

    const effMass = 1.0 / (firstBodyParams.invEffMass + secondBodyParams.invEffMass);
    const Jv = -vec3.dot(contact.firstMoveDir, contact.firstBody.currentVelocity) -
      vec3.dot(firstBodyParams.rCrossN, contact.firstBody.currentAngleSpeed) +
      vec3.dot(contact.secondMoveDir, contact.secondBody.currentVelocity) +
      vec3.dot(secondBodyParams.rCrossN, contact.secondBody.currentAngleSpeed);

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
      contact.firstBody, contact.firstMoveDir, firstBodyParams.rCrossN, deltaLambda, dt);
    updatePosAndQuatFromConstraint(
      contact.secondBody, contact.secondMoveDir, secondBodyParams.rCrossN, deltaLambda, dt);
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
      if (body.isStatic === true && renderEnclosure !== true)
      {
        continue;
      }
      p.push();
      p.applyMatrix([...body.currentTransformMatrix ]);
      if (body.isStatic === true)
      {
        p.ambientMaterial(90, 90, 90);
      }
      else
      {
        p.ambientMaterial(65, 130, 255);
      }
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
    p.text("Part 3: 10 rigid bodies collision (Sequentinal Impulses)", 32, 30);
    p.text("Press R to reset, T to stop/continue simulation,", 32, 56);
    p.text("E to render/not render enclosure", 32, 72);
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

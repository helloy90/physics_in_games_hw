import * as mat3 from "../libraries/esm/mat3.js";
import * as mat4 from "../libraries/esm/mat4.js";
import * as quat from "../libraries/esm/quat.js";
import * as vec3 from "../libraries/esm/vec3.js";

const sim_modes = [
  {label : "1) XPBD"},
  {label : "2) Sequential Impulses"},
];

export function createPart3_1(p)
{
  let bodies = [];
  let contacts = [];

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
    let positions = [
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 30, 0),
      vec3.fromValues(0, 60, 0),
      vec3.fromValues(10, 30, 30),
      vec3.fromValues(10, 60, 30),
      vec3.fromValues(-20, 90, 60),
      vec3.fromValues(20, 60, 60),
      vec3.fromValues(0, 150, 30),
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
    quat.fromEuler(rotations[1], 0, 30, 0);
    quat.fromEuler(rotations[2], 0, 60, 0);
    quat.fromEuler(rotations[3], 0, 30, 0);
    quat.fromEuler(rotations[4], 0, 60, 0);
    quat.fromEuler(rotations[5], 0, 90, 0);
    quat.fromEuler(rotations[6], 0, 60, 0);
    quat.fromEuler(rotations[7], 0, 90, 0);
    quat.fromEuler(rotations[8], 0, 30, 0);
    quat.fromEuler(rotations[9], 0, 120, 0);

    bodies = [];
    contacts = [];
    for (let i = 0; i < 10; i++)
    {
      bodies.push(makeBody(5.0, 60, 20, 20, positions[i], rotations[i]));
    }
    lambda = 0;

    p.camera(300, 150, 300, 0, 0, 0, 0, -1, 0);
  }

  function xpbdStep(dt)
  {
    const subDt = dt / subSteps;


    for (let step = 0; step < subSteps; step++)
    {
      for (const body of bodies)
      {
        prePosSolve(body, subDt);
      }

      solvePosConstraints(subDt);

      for (const body of bodies)
      {
        postPosSolve(body, subDt);
      }
    }
  }

  function collectContacts()
  {
    const res = [];

    const worldData =
      bodies.map(body => { return {axis : getBoxAxis(body), vertices : getWorldVertices(body)}; });

    for (let i = 0; i < bodies.length; i++)
    {
      const firstBody = bodies[i];
      const firstBodyData = worldData[i];
      for (let j = i + 1; j < bodies.length; j++)
      {
        const secondBody = bodies[i];
        const secondBodyData = worldData[i];

        for (const vert of firstBodyData.vertices)
        {
          // TODO
        }
      }
    }
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
      vec3.transformMat4(worldVert, body.currentTransformMatrix);
      return worldVert
    })
  }

  function pointInsideBoxCheck(body, vert)
  {
    const half = vec3.fromValues(body.width / 2, body.height / 2, body.depth / 2);

    const localVert = vec3.create();
    vec3.transformMat4(localVert, vert, body.invCurrentTransformMatrix);
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

    // TODO
    const complianceScaled = 0;
    const deltaLambda = 0;
    // ? / (dt * dt);
    // const deltaLambda = (-firstBodyParams.stretch - complianceScaled * lambda) /
    //   (firstBodyParams.invEffMass + secondBodyParams.invEffMass + complianceScaled);
    // lambda += deltaLambda;

    // const impulse = vec3.clone(firstBodyParams.vecToSpringPosNorm);
    // vec3.scale(impulse, impulse, deltaLambda);

    // updatePosAndQuatFromConstraint(body1, firstBodyParams, impulse, -1);
    // updatePosAndQuatFromConstraint(body2, secondBodyParams, impulse, 1);
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

    mat4.fromRotationTranslation(
      body.currentTransformMatrix, body.currentQuat, body.currentWorldPos);
    mat4.invert(body.invCurrentTransformMatrix, body.currentTransformMatrix);
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
    p.text("Part 3: 10 rigid bodies collision", 32, 30);
    p.text(`Mode: ${sim_modes[currentMode].label}`, 32, 56);
    p.text("Press R to reset. T to stop/continue simulation", 32, 82);
    p.pop();
  }
}

function makeBody(mass, width, height, depth, worldPos, initRotation)
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
    localInertiaTensor : inertia,
    worldInertiaTensor : worldInertia,
    currentAngularMomentum : initialAngularMomentum,
    prevQuat : initialQuat,
    currentQuat : nextQuat,
    currentAngleSpeed : currentAngleSpeed,
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

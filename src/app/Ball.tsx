import * as THREE from "three";
import * as CANNON from "cannon-es";
import Soccer from "../../public/Soccer.jpg";

export const createBouncingBall = (
  scene: THREE.Scene,
  world: CANNON.World,
  radius: number,
  initialPosition: THREE.Vector3,
  initialVelocity: THREE.Vector3,
  mass: number
) => {
  const textureLoader = new THREE.TextureLoader();
  const texture = textureLoader.load(Soccer.src);
  const geometry = new THREE.SphereGeometry(radius, 32, 32);
  const material = new THREE.MeshBasicMaterial({ map: texture });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.position.copy(initialPosition);
  scene.add(mesh);

  // 创建物理体
  const shape = new CANNON.Sphere(radius);
  const body = new CANNON.Body({ mass });
  body.addShape(shape);
  body.position.copy(initialPosition as any);
  body.velocity.copy(initialVelocity as any);
  world.addBody(body);

  body.addEventListener("collide", (event: any) => {
    const contact = event.contact;

    if (contact.bi.id === body.id || contact.bj.id === body.id) {
      const normal = contact.ni.clone();
      const relativeVelocity = contact.bi.velocity.vsub(contact.bj.velocity);
      const velocityAlongNormal = relativeVelocity.dot(normal);

      if (velocityAlongNormal < 0) {
        const restitution = 0.9;
        const impulse = normal.scale(-velocityAlongNormal * (1 + restitution));
        body.applyImpulse(impulse, contact.ri);
      }
    }
  });

  const update = () => {
    mesh.position.copy(body.position as any);
    mesh.quaternion.copy(body.quaternion as any);
  };

  return { mesh, body, update };
};

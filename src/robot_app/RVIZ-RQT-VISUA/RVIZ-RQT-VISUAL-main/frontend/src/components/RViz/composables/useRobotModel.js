import { ref, shallowRef } from 'vue'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader'

export function useRobotModel(scene, tfClient) {
  const robotMesh = shallowRef(null)
  
  // 默认创建一个简单的机器人模型（如果无法加载URDF）
  const createDefaultRobot = () => {
    const group = new THREE.Group()
    
    //底盘
    const bodyGeo = new THREE.BoxGeometry(0.4, 0.3, 0.2)
    const bodyMat = new THREE.MeshLambertMaterial({ color: 0x2563eb })
    const body = new THREE.Mesh(bodyGeo, bodyMat)
    body.position.z = 0.1
    group.add(body)
    
    // 轮子
    const wheelGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.05, 32)
    const wheelMat = new THREE.MeshLambertMaterial({ color: 0x333333 })
    const positions = [
      { x: 0.12, y: 0.18 }, { x: -0.12, y: 0.18 },
      { x: 0.12, y: -0.18 }, { x: -0.12, y: -0.18 }
    ]
    
    positions.forEach(pos => {
      const wheel = new THREE.Mesh(wheelGeo, wheelMat)
      wheel.rotation.z = Math.PI / 2
      wheel.position.set(pos.x, pos.y, 0.08)
      group.add(wheel)
    })
    
    // 激光雷达指示
    const lidarGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.06, 16)
    const lidarMat = new THREE.MeshLambertMaterial({ color: 0x10b981 })
    const lidar = new THREE.Mesh(lidarGeo, lidarMat)
    lidar.position.set(0.1, 0, 0.23)
    group.add(lidar)
    
    // 添加箭头指示方向
    const arrowHelper = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0.25),
      0.3,
      0xffff00
    )
    group.add(arrowHelper)

    return group
  }

  const initRobot = () => {
    // 移除旧模型
    if (robotMesh.value) {
      scene.remove(robotMesh.value)
    }

    robotMesh.value = createDefaultRobot()
    scene.add(robotMesh.value)
    
    // TODO: 实现URDF加载逻辑
    console.log('Robot model initialized')
  }

  const updateRobotPose = (position, orientation) => {
    if (!robotMesh.value) return

    robotMesh.value.position.set(position.x, position.y, position.z)
    
    if (orientation) {
      // 如果 orientation 是四元数对象 {x,y,z,w}
      robotMesh.value.quaternion.set(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      )
    }
  }

  return {
    robotMesh,
    initRobot,
    updateRobotPose
  }
}

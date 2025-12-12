<template>
  <div class="scene-container" ref="containerRef">
    <!-- 3D Canvas 挂载点 -->
    <canvas ref="canvasRef" class="scene-canvas"></canvas>
    
    <!-- 加载指示器 -->
    <div v-if="loading" class="loading-overlay">
      <div class="spinner"></div>
      <div class="loading-text">Loading 3D Scene...</div>
    </div>
  </div>
</template>

<script>
import { ref, onMounted, onUnmounted, shallowRef, watch } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
import { useConnectionStore } from '../../composables/useConnectionStore'
import { useUIStore } from '../../composables/useUIStore'
import { useRobotModel } from './composables/useRobotModel'
import { usePointCloud } from './composables/usePointCloud'
import Stats from 'three/examples/jsm/libs/stats.module'

export default {
  name: 'Scene3D',
  setup() {
    const containerRef = ref(null)
    const canvasRef = ref(null)
    const loading = ref(true)
    
    // Three.js 核心对象
    const scene = shallowRef(null)
    const camera = shallowRef(null)
    const renderer = shallowRef(null)
    const controls = shallowRef(null)
    const stats = shallowRef(null)
    
    // 状态管理
    const connectionStore = useConnectionStore()
    const uiStore = useUIStore() // UI Store for layers
    
    // Composables
    let robotModel = null
    let pointCloudTools = null
    
    // Grid Ref for visibility toggling
    let gridHelper = null
    let axesHelper = null

    // Watch UI Layer Changes
    watch(() => uiStore.layers, (newLayers) => {
        // Toggle Grid
        if (gridHelper) gridHelper.visible = newLayers.grid
        if (axesHelper) axesHelper.visible = newLayers.tf

        // Toggle Robot
        if (robotModel) robotModel.setVisible(newLayers.robot) // Assuming setVisible exists or need to access group

        // Toggle Point Cloud
        if (pointCloudTools) pointCloudTools.setVisible(newLayers.points) 
        
        // Map/Laser need their own handling if implemented
    }, { deep: true })

    const initScene = () => {
      // 1. Scene
      scene.value = new THREE.Scene()
      scene.value.background = new THREE.Color(0x111827) // Dark Pro Background
      
      // 2. Camera
      const width = containerRef.value.clientWidth
      const height = containerRef.value.clientHeight
      camera.value = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000)
      camera.value.position.set(-5, 0, 8) // Modified view
      camera.value.up.set(0, 0, 1) // ROS Z-up
      
      // 3. Renderer
      renderer.value = new THREE.WebGLRenderer({ 
        canvas: canvasRef.value, 
        antialias: true,
        alpha: true 
      })
      renderer.value.setSize(width, height)
      renderer.value.setPixelRatio(window.devicePixelRatio)
      
      // 4. Controls
      controls.value = new OrbitControls(camera.value, renderer.value.domElement)
      controls.value.enableDamping = true
      controls.value.dampingFactor = 0.1
      
      // 5. Lights
      const ambientLight = new THREE.AmbientLight(0xffffff, 0.4)
      scene.value.add(ambientLight)
      const dirLight = new THREE.DirectionalLight(0xffffff, 0.8)
      dirLight.position.set(10, 10, 10)
      scene.value.add(dirLight)
      
      // 6. Helpers
      // Grid: size 40, divisions 40, centerColor, gridColor
      gridHelper = new THREE.GridHelper(40, 40, 0x475569, 0x1e293b)
      gridHelper.rotation.x = Math.PI / 2
      scene.value.add(gridHelper)
      
      axesHelper = new THREE.AxesHelper(1)
      scene.value.add(axesHelper)
      
      // 7. Stats
      stats.value = new Stats()
      // containerRef.value.appendChild(stats.value.dom)
      
      // 初始化 Composables
      // Robot Model
      robotModel = useRobotModel(scene.value)
      robotModel.initRobot()
      
      // Point Cloud
      pointCloudTools = usePointCloud(scene.value)
      
      // Initial Visibility Set
      if(uiStore.layers) {
         gridHelper.visible = uiStore.layers.grid
         axesHelper.visible = uiStore.layers.tf
         // robotModel.setVisible(uiStore.layers.robot)
      }

      loading.value = false
      animate()
    }
    
    const animate = () => {
      requestAnimationFrame(animate)
      
      if (controls.value) controls.value.update()
      if (stats.value) stats.value.update()
      
      if (renderer.value && scene.value && camera.value) {
        renderer.value.render(scene.value, camera.value)
      }
    }
    
    const handleResize = () => {
      if (!containerRef.value || !camera.value || !renderer.value) return
      
      const width = containerRef.value.clientWidth
      const height = containerRef.value.clientHeight
      
      camera.value.aspect = width / height
      camera.value.updateProjectionMatrix()
      
      renderer.value.setSize(width, height)
    }

    // ROS 订阅逻辑
    // 监听连接状态，启动必要的订阅
    watch(() => connectionStore.isConnected, (connected) => {
        if (connected) {
            setupSubscriptions()
        }
    })
    
    const setupSubscriptions = () => {
        // 使用 connectionStore 提供的 rosbridge 实例
        const rosbridge = connectionStore.rosbridge
        if (!rosbridge) return

        // 订阅点云 (使用 TF 转换后的 map 帧话题)
        rosbridge.subscribe('/cloud_registered_map', 'sensor_msgs/msg/PointCloud2', (msg) => {
           if (pointCloudTools) {
             pointCloudTools.updatePointCloud('/cloud_registered_map', msg)
           }
        })

        // 订阅 TF 或 Odometry 更新机器人位置
        // 这里简化演示，实际应解析 /tf 或 /odom
        rosbridge.subscribe('/odom', 'nav_msgs/msg/Odometry', (msg) => {
           if (robotModel && msg.pose && msg.pose.pose) {
             const p = msg.pose.pose.position
             const o = msg.pose.pose.orientation
             robotModel.updateRobotPose(p, o)
           }
        })
    }
    
    // 暴露给外部的方法
    const toggleGrid = () => { 
       // Implement grid toggle logic
    }
    const toggleAxes = () => { 
       // Implement axes toggle logic
    }
    const resetCamera = () => {
       if (camera.value && controls.value) {
           camera.value.position.set(-5, 5, 5)
           controls.value.reset()
       }
    }

    onMounted(() => {
      initScene()
      window.addEventListener('resize', handleResize)
    })
    
    onUnmounted(() => {
      window.removeEventListener('resize', handleResize)
      // Cleanup Three.js resources
      if (renderer.value) renderer.value.dispose()
      if (pointCloudTools) pointCloudTools.clearPointClouds()
    })
    
    return {
      containerRef,
      canvasRef,
      loading,
      handleResize,
      resetCamera,
      toggleGrid,
      toggleAxes
    }
  }
}
</script>

<style scoped>
.scene-container {
  width: 100%;
  height: 100%;
  position: relative;
  overflow: hidden;
  background: #f8fafc;
}

.scene-canvas {
  width: 100%;
  height: 100%;
  outline: none;
}

.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: rgba(255, 255, 255, 0.9);
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  z-index: 20;
}

.spinner {
  width: 40px;
  height: 40px;
  border: 3px solid #e2e8f0;
  border-top: 3px solid #2563eb;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

.loading-text {
  margin-top: 15px;
  font-size: 14px;
  color: #64748b;
  font-weight: 500;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}
</style>

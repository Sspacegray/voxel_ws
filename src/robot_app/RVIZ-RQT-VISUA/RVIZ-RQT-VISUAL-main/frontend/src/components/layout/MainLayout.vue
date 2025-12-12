<template>
  <div class="pro-layout">
    <!-- Header Area -->
    <header class="layout-header">
      <div class="brand-area">
        <el-icon class="logo"><Monitor /></el-icon>
        <span class="app-name">ROBOT VIZ <span class="version">PRO</span></span>
      </div>
      <div class="header-status">
         <!-- Simple Status Indicators -->
         <span class="status-item" :class="{ ok: isConnected }">
           <div class="dot"></div> {{ isConnected ? 'CONNECTED' : 'DISCONNECTED' }}
         </span>
         <span class="status-item battery">
           <el-icon><Lightning /></el-icon> 98%
         </span>
      </div>
      <div class="header-actions">
        <button class="estop-btn-small" @click="triggerEStop">STOP</button>
      </div>
    </header>

    <!-- Main Workspace -->
    <div class="layout-body">
      <!-- Left Nav -->
      <LeftSidebar :currentMode="currentMode" @mode-change="setMode" />
      
      <!-- Center 3D -->
      <div class="center-viewport">
        <Scene3D ref="sceneRef" />
        
        <!-- Overlay for warnings/notifications only -->
        <div class="viewport-overlay" v-if="!isConnected">
           <div class="disconnect-warning">SYSTEM DISCONNECTED</div>
        </div>
      </div>
      
      <!-- Right Controls -->
      <RightSidebar :mode="currentMode" />
    </div>

    <!-- Footer -->
    <footer class="layout-footer">
      <div class="log-ticker">
        <span class="log-time">[10:42:05]</span>
        <span class="log-msg">System initialized. Ready for operation.</span>
      </div>
      <div class="footer-meta">
        <span>Latency: {{ latency }}ms</span>
        <span>FPS: 60</span>
      </div>
    </footer>
  </div>
</template>

<script>
import { ref, computed } from 'vue'
import { Monitor, Lightning } from '@element-plus/icons-vue'
import Scene3D from '../RViz/Scene3D.vue'
import LeftSidebar from './LeftSidebar.vue'
import RightSidebar from './RightSidebar.vue'
import { useConnectionStore } from '../../composables/useConnectionStore'
import { ElMessage } from 'element-plus'

export default {
  name: 'MainLayout', // Keeping name to avoid router refactor
  components: { Scene3D, LeftSidebar, RightSidebar, Monitor, Lightning },
  setup() {
    const connectionStore = useConnectionStore()
    const currentMode = ref('monitor')

    const setMode = (mode) => {
      currentMode.value = mode
    }

    const triggerEStop = () => {
      ElMessage.error('EMERGENCY STOP TRIGGERED')
    }

    return {
      isConnected: computed(() => connectionStore.isConnected),
      latency: computed(() => connectionStore.latency || 0),
      currentMode,
      setMode,
      triggerEStop
    }
  }
}
</script>

<style scoped>
.pro-layout {
  display: flex;
  flex-direction: column;
  width: 100vw;
  height: 100vh;
  background: #0b1121; /* Deep Space */
  color: #e2e8f0;
  overflow: hidden;
  font-family: 'Inter', sans-serif;
}

/* Header */
.layout-header {
  height: 50px;
  background: #111827;
  border-bottom: 1px solid #1f2937;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 20px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
  z-index: 30;
}

.brand-area {
  display: flex;
  align-items: center;
  gap: 10px;
  font-weight: 700;
  color: #f8fafc;
  letter-spacing: 0.5px;
}

.brand-area .logo {
  color: #06b6d4;
  font-size: 20px;
}

.version {
  background: #06b6d4;
  color: #000;
  font-size: 10px;
  padding: 2px 4px;
  border-radius: 2px;
  margin-left: 5px;
}

.header-status {
  display: flex;
  gap: 20px;
  font-size: 12px;
  font-weight: 600;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 6px;
  color: #64748b;
}

.status-item.ok {
  color: #10b981;
}

.dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: currentColor;
  box-shadow: 0 0 8px currentColor;
}

.battery {
  color: #f59e0b;
}

.estop-btn-small {
  background: #dc2626;
  color: white;
  border: none;
  padding: 6px 16px;
  border-radius: 4px;
  font-weight: 700;
  font-size: 12px;
  cursor: pointer;
  letter-spacing: 1px;
}

.estop-btn-small:hover {
  background: #ef4444;
}

/* Body */
.layout-body {
  flex: 1;
  display: flex;
  overflow: hidden;
}

.center-viewport {
  flex: 1;
  position: relative;
  background: #000; /* Will be covered by canvas */
}

/* Footer */
.layout-footer {
  height: 30px;
  background: #111827;
  border-top: 1px solid #1f2937;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 10px;
  font-size: 11px;
  color: #64748b;
  font-family: 'JetBrains Mono', monospace;
  z-index: 30;
}

.log-ticker {
  display: flex;
  gap: 8px;
}

.log-time {
  color: #06b6d4;
}

.footer-meta {
  display: flex;
  gap: 15px;
}

.viewport-overlay {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
  display: flex;
  align-items: center;
  justify-content: center;
}

.disconnect-warning {
  background: rgba(220, 38, 38, 0.9);
  color: white;
  padding: 10px 20px;
  border-radius: 4px;
  font-weight: 700;
  backdrop-filter: blur(4px);
}
</style>

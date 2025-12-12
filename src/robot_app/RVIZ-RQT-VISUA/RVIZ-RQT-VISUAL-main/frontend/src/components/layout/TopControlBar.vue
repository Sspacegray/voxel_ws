<template>
  <div class="top-control-bar">
    <!-- Left: Brand & Status -->
    <div class="bar-section left-section">
      <div class="brand">
        <el-icon class="brand-logo"><Monitor /></el-icon>
        <span class="brand-text">ROBOT VIZ</span>
      </div>
      <div class="divider"></div>
      <div class="status-indicator" :class="{ 'connected': isConnected }">
        <div class="status-dot"></div>
        <div class="status-detail">
          <span class="status-label">SYSTEM</span>
          <span class="status-value">{{ isConnected ? 'ONLINE' : 'OFFLINE' }}</span>
        </div>
      </div>
    </div>

    <!-- Center: Function Modes (Tab Style) -->
    <div class="bar-section center-section">
      <div 
        v-for="mode in modes" 
        :key="mode.id"
        class="nav-tab"
        :class="{ 'active': currentMode === mode.id }"
        @click="setMode(mode.id)"
      >
        <el-icon class="tab-icon"><component :is="mode.icon" /></el-icon>
        <span class="tab-label">{{ mode.label }}</span>
        <div class="active-line"></div>
      </div>
    </div>

    <!-- Right: Actions -->
    <div class="bar-section right-section">
      <div class="battery-status">
        <el-icon><Lightning /></el-icon>
        <span>98%</span>
      </div>
      
      <button class="estop-btn" @click="triggerEStop">
        <el-icon><SwitchButton /></el-icon>
        <span>STOP</span>
      </button>
      
      <button class="settings-btn" @click="$emit('toggle-settings')">
        <el-icon><Setting /></el-icon>
      </button>
    </div>
  </div>
</template>

<script>
import { ref, computed } from 'vue'
import { useConnectionStore } from '../../composables/useConnectionStore'
import { 
  Monitor, 
  VideoPlay, 
  MapLocation, 
  Position, 
  EditPen,
  SwitchButton,
  Setting,
  Lightning
} from '@element-plus/icons-vue'

export default {
  name: 'TopControlBar',
  components: {
    Monitor, VideoPlay, MapLocation, Position, EditPen, SwitchButton, Setting, Lightning
  },
  emits: ['mode-change', 'toggle-settings', 'estop'],
  setup(props, { emit }) {
    const connectionStore = useConnectionStore()
    const currentMode = ref('monitor')

    const modes = [
      { id: 'monitor', label: 'MONITOR', icon: 'Monitor' },
      { id: 'manual', label: 'MANUAL', icon: 'VideoPlay' },
      { id: 'mapping', label: 'MAPPING', icon: 'MapLocation' },
      { id: 'navigation', label: 'NAV', icon: 'Position' },
      { id: 'waypoint', label: 'EDITOR', icon: 'EditPen' }
    ]

    const setMode = (modeId) => {
      currentMode.value = modeId
      emit('mode-change', modeId)
    }

    const triggerEStop = () => {
      emit('estop')
    }

    return {
      isConnected: computed(() => connectionStore.isConnected),
      currentMode,
      modes,
      setMode,
      triggerEStop
    }
  }
}
</script>

<style scoped>
.top-control-bar {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 64px;
  background: rgba(15, 23, 42, 0.9);
  backdrop-filter: blur(10px);
  border-bottom: 1px solid rgba(6, 182, 212, 0.2);
  display: flex;
  justify-content: space-between;
  align-items: stretch;
  padding: 0 24px;
  z-index: 100;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
}

.bar-section {
  display: flex;
  align-items: center;
  gap: 20px;
}

/* Left Section */
.brand {
  display: flex;
  align-items: center;
  gap: 10px;
  color: #06b6d4;
  font-weight: 700;
  letter-spacing: 1px;
  font-size: 18px;
}

.divider {
  width: 1px;
  height: 24px;
  background: rgba(148, 163, 184, 0.2);
}

.status-indicator {
  display: flex;
  align-items: center;
  gap: 10px;
}

.status-dot {
  width: 8px;
  height: 8px;
  background: #ef4444;
  border-radius: 50%;
  box-shadow: 0 0 10px #ef4444;
}

.status-indicator.connected .status-dot {
  background: #10b981;
  box-shadow: 0 0 10px #10b981;
}

.status-detail {
  display: flex;
  flex-direction: column;
  line-height: 1;
}

.status-label {
  font-size: 10px;
  color: #64748b;
  font-weight: 600;
}

.status-value {
  font-size: 14px;
  font-weight: 700;
  color: #cbd5e1;
}

/* Center Section */
.center-section {
  gap: 4px;
}

.nav-tab {
  position: relative;
  display: flex;
  align-items: center;
  gap: 8px;
  height: 100%;
  padding: 0 24px;
  color: #94a3b8;
  cursor: pointer;
  transition: all 0.2s;
  font-weight: 500;
  letter-spacing: 0.5px;
  font-size: 14px;
}

.nav-tab:hover {
  background: rgba(255, 255, 255, 0.03);
  color: #fff;
}

.nav-tab.active {
  color: #06b6d4;
  background: rgba(6, 182, 212, 0.05);
}

.active-line {
  position: absolute;
  bottom: 0;
  left: 0;
  width: 100%;
  height: 3px;
  background: transparent;
  transition: all 0.2s;
}

.nav-tab.active .active-line {
  background: #06b6d4;
  box-shadow: 0 -2px 10px rgba(6, 182, 212, 0.5);
}

/* Right Section */
.battery-status {
  display: flex;
  align-items: center;
  gap: 6px;
  color: #10b981;
  font-weight: 600;
  background: rgba(16, 185, 129, 0.1);
  padding: 6px 12px;
  border-radius: 4px;
  border: 1px solid rgba(16, 185, 129, 0.2);
}

.estop-btn {
  display: flex;
  align-items: center;
  gap: 8px;
  background: rgba(239, 68, 68, 0.9);
  color: white;
  border: none;
  padding: 8px 16px;
  border-radius: 2px;
  font-weight: 700;
  cursor: pointer;
  box-shadow: 0 0 15px rgba(239, 68, 68, 0.4);
  transition: all 0.2s;
}

.estop-btn:hover {
  background: #ef4444;
  transform: scale(1.05);
}

.settings-btn {
  width: 36px;
  height: 36px;
  background: transparent;
  border: 1px solid rgba(148, 163, 184, 0.2);
  color: #94a3b8;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 4px;
  transition: all 0.2s;
}

.settings-btn:hover {
  border-color: #06b6d4;
  color: #06b6d4;
}
</style>

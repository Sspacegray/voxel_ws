<template>
  <div class="left-sidebar">
    <div 
      v-for="item in menuItems" 
      :key="item.id"
      class="nav-item"
      :class="{ active: currentMode === item.id }"
      @click="$emit('mode-change', item.id)"
    >
      <el-icon class="nav-icon"><component :is="item.icon" /></el-icon>
      <span class="nav-label">{{ item.label }}</span>
      <div class="active-bar"></div>
    </div>
  </div>
</template>

<script>
import { Monitor, VideoPlay, MapLocation,  Position, EditPen, Setting } from '@element-plus/icons-vue'

export default {
  name: 'LeftSidebar',
  components: { Monitor, VideoPlay, MapLocation, Position, EditPen, Setting },
  props: {
    currentMode: String
  },
  setup() {
    const menuItems = [
      { id: 'monitor', label: 'MONITOR', icon: 'Monitor' },
      { id: 'manual', label: 'MANUAL', icon: 'VideoPlay' },
      { id: 'mapping', label: 'MAPPING', icon: 'MapLocation' },
      { id: 'navigation', label: 'NAVIGATE', icon: 'Position' },
      { id: 'waypoint', label: 'EDITOR', icon: 'EditPen' },
      { id: 'settings', label: 'CONFIG', icon: 'Setting' }
    ]

    return { menuItems }
  }
}
</script>

<style scoped>
.left-sidebar {
  width: 80px;
  height: 100%;
  background: #111827; /* Gray 900 */
  border-right: 1px solid #1f2937; /* Gray 800 */
  display: flex;
  flex-direction: column;
  align-items: center;
  padding-top: 10px;
  z-index: 20;
}

.nav-item {
  width: 100%;
  height: 70px;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  cursor: pointer;
  color: #94a3b8;
  position: relative;
  transition: all 0.2s;
  gap: 4px;
}

.nav-item:hover {
  background: #1f2937;
  color: #f1f5f9;
}

.nav-item.active {
  color: #06b6d4; /* Cyan */
  background: rgba(6, 182, 212, 0.05);
}

.nav-icon {
  font-size: 24px;
}

.nav-label {
  font-size: 10px;
  font-weight: 600;
  letter-spacing: 0.5px;
}

.active-bar {
  position: absolute;
  left: 0;
  top: 50%;
  transform: translateY(-50%);
  width: 3px;
  height: 0;
  background: #06b6d4;
  transition: height 0.2s;
}

.nav-item.active .active-bar {
  height: 70%;
}
</style>

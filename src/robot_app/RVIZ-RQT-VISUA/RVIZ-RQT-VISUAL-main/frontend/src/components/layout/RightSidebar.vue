<template>
  <div class="right-sidebar">
    <!-- Context Panel (Dynamic) -->
    <div class="sidebar-section context-section">
      <div class="section-header">
        <span>ACTIVE TOOL</span>
      </div>
      <div class="section-content">
        <MapManagerPanel v-if="mode === 'mapping' || mode === 'navigation'" />
        <WaypointPanel v-else-if="mode === 'waypoint'" />
        <SpeedControlPanel v-else-if="mode === 'manual'" />
        <div v-else class="empty-state">
           <el-icon class="empty-icon"><Monitor /></el-icon>
           <span>System Monitoring</span>
        </div>
      </div>
    </div>

    <div class="sidebar-divider"></div>

    <!-- Layer Control (Global) -->
    <div class="sidebar-section">
      <div class="section-header">
        <span>LAYERS</span>
      </div>
      <div class="layer-list">
        <div class="layer-item" v-for="layer in layersList" :key="layer.id" @click="uiStore.toggleLayer(layer.id)">
          <el-icon :class="uiStore.layers[layer.id] ? 'icon-active' : 'icon-inactive'">
             <component :is="uiStore.layers[layer.id] ? 'View' : 'Hide'" />
          </el-icon>
          <span>{{ layer.name }}</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { ref } from 'vue'
import { useUIStore } from '../../composables/useUIStore'
import MapManagerPanel from '../panels/MapManagerPanel.vue'
import WaypointPanel from '../panels/WaypointPanel.vue'
import SpeedControlPanel from '../panels/SpeedControlPanel.vue'
import { Monitor, View, Hide } from '@element-plus/icons-vue'

export default {
  name: 'RightSidebar',
  components: { MapManagerPanel, WaypointPanel, SpeedControlPanel, Monitor, View, Hide },
  props: {
    mode: String
  },
  setup() {
    const uiStore = useUIStore()

    // Map store state to array for rendering
    const layersList = [
      { id: 'robot', name: 'Robot Model' },
      { id: 'laser', name: 'Laser Scan' },
      { id: 'map', name: 'Global Map' },
      { id: 'points', name: 'Point Cloud' },
      { id: 'tf', name: 'TF Frames' },
      { id: 'grid', name: 'Reference Grid' }
    ]

    return { 
      uiStore,
      layersList
    }
  }
}
</script>

<style scoped>
.right-sidebar {
  width: 320px;
  height: 100%;
  background: #111827;
  border-left: 1px solid #1f2937;
  display: flex;
  flex-direction: column;
  z-index: 20;
}

.sidebar-section {
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.context-section {
  flex: 1; /* Allow this to grow */
  border-bottom: 1px solid #1f2937;
  overflow-y: auto;
}

.section-header {
  font-size: 11px;
  font-weight: 700;
  color: #64748b;
  letter-spacing: 1px;
  text-transform: uppercase;
}

.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 12px;
  height: 100px;
  color: #475569;
}

.empty-icon {
  font-size: 32px;
  opacity: 0.5;
}

.layer-list {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.layer-item {
  display: flex;
  align-items: center;
  gap: 10px;
  color: #94a3b8;
  cursor: pointer;
  padding: 8px;
  border-radius: 4px;
  background: rgba(255, 255, 255, 0.03);
  font-size: 13px;
}

.layer-item:hover {
  background: rgba(255, 255, 255, 0.05);
  color: #fff;
}

.icon-active {
  color: #06b6d4;
}

.icon-inactive {
  color: #475569;
}
</style>
